/*
 * String printing Visitor
 *
 * Copyright Red Hat, Inc. 2012
 *
 * Author: Paolo Bonzini <pbonzini@redhat.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#include "qemu-common.h"
#include "qapi/string-output-visitor.h"
#include "qapi/visitor-impl.h"
#include "qapi/qmp/qerror.h"
#include "qemu/host-utils.h"
#include <math.h>

enum ListMode {
    LM_NONE,             /* not traversing a list of repeated options */
    LM_STARTED,          /* start_list() succeeded */

    LM_IN_PROGRESS,      /* next_list() has been called.
                          *
                          * Generating the next list link will consume the most
                          * recently parsed QemuOpt instance of the repeated
                          * option.
                          *
                          * Parsing a value into the list link will examine the
                          * next QemuOpt instance of the repeated option, and
                          * possibly enter LM_SIGNED_INTERVAL or
                          * LM_UNSIGNED_INTERVAL.
                          */

    LM_SIGNED_INTERVAL,  /* next_list() has been called.
                          *
                          * Generating the next list link will consume the most
                          * recently stored element from the signed interval,
                          * parsed from the most recent QemuOpt instance of the
                          * repeated option. This may consume QemuOpt itself
                          * and return to LM_IN_PROGRESS.
                          *
                          * Parsing a value into the list link will store the
                          * next element of the signed interval.
                          */

    LM_UNSIGNED_INTERVAL,/* Same as above, only for an unsigned interval. */

    LM_END
};

typedef enum ListMode ListMode;

struct StringOutputVisitor
{
    Visitor visitor;
    bool human;
    bool head;
    char *string;
    ListMode list_mode;
    /* When parsing a list of repeating options as integers, values of the form
     * "a-b", representing a closed interval, are allowed. Elements in the
     * range are generated individually.
     */
    union {
        int64_t s;
        uint64_t u;
    } range_start, range_end;

};

static void string_output_set(StringOutputVisitor *sov, char *string)
{
    g_free(sov->string);
    sov->string = string;
}

static void print_type_int(Visitor *v, int64_t *obj, const char *name,
                           Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);
    char *out = NULL;

    switch (sov->list_mode) {
    case LM_NONE:
        if (sov->human) {
            out = g_strdup_printf("%lld (%#llx)", (long long) *obj,
                                  (long long) *obj);
        } else {
            out = g_strdup_printf("%lld", (long long) *obj);
        }
        sov->list_mode = LM_END;
        break;

    case LM_STARTED:
        sov->range_start.s = *obj;
        sov->range_end.s = *obj;
        sov->list_mode = LM_IN_PROGRESS;
        break;

    case LM_IN_PROGRESS:
        assert(sov->range_end.s + 1 == *obj);
        sov->range_end.s++;
        break;

    case LM_END:
        assert(sov->range_end.s + 1 == *obj);
        sov->range_end.s++;
        if (sov->range_end.s == sov->range_start.s) {
            if (sov->human) {
                out = g_strdup_printf("%lld (%#llx)",
                                      (long long)sov->range_start.s,
                                      (long long)sov->range_start.s);
            } else {
                out = g_strdup_printf("%lld", (long long)sov->range_start.s);
            }
        } else {
            if (sov->human) {
                out = g_strdup_printf("%lld(%#llx)-%lld(%#llx)",
                                      (long long) sov->range_start.s,
                                      (long long) sov->range_start.s,
                                      (long long) sov->range_end.s,
                                      (long long) sov->range_end.s);
            } else {
                out = g_strdup_printf("%lld-%lld",
                                      (long long) sov->range_start.s,
                                      (long long) sov->range_end.s);
            }
        }
        break;

    default:
        abort();
    }

    string_output_set(sov, out);
}

static void print_type_size(Visitor *v, uint64_t *obj, const char *name,
                           Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);
    static const char suffixes[] = { 'B', 'K', 'M', 'G', 'T', 'P', 'E' };
    uint64_t div, val;
    char *out;
    int i;

    if (!sov->human) {
        out = g_strdup_printf("%"PRIu64, *obj);
        string_output_set(sov, out);
        return;
    }

    val = *obj;

    /* The exponent (returned in i) minus one gives us
     * floor(log2(val * 1024 / 1000).  The correction makes us
     * switch to the higher power when the integer part is >= 1000.
     */
    frexp(val / (1000.0 / 1024.0), &i);
    i = (i - 1) / 10;
    assert(i < ARRAY_SIZE(suffixes));
    div = 1ULL << (i * 10);

    out = g_strdup_printf("%"PRIu64" (%0.3g %c%s)", val,
                          (double)val/div, suffixes[i], i ? "iB" : "");
    string_output_set(sov, out);
}

static void print_type_bool(Visitor *v, bool *obj, const char *name,
                            Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);
    string_output_set(sov, g_strdup(*obj ? "true" : "false"));
}

static void print_type_str(Visitor *v, char **obj, const char *name,
                           Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);
    char *out;

    if (sov->human) {
        out = *obj ? g_strdup_printf("\"%s\"", *obj) : g_strdup("<null>");
    } else {
        out = g_strdup(*obj ? *obj : "");
    }
    string_output_set(sov, out);
}

static void print_type_number(Visitor *v, double *obj, const char *name,
                              Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);
    string_output_set(sov, g_strdup_printf("%f", *obj));
}

static void
start_list(Visitor *v, const char *name, Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);

    /* we can't traverse a list in a list */
    assert(sov->list_mode == LM_NONE);
    sov->list_mode = LM_STARTED;
    sov->head = true;
}

static GenericList *
next_list(Visitor *v, GenericList **list, Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);
    GenericList *ret = NULL;
    if (*list) {
        if (sov->head) {
            ret = *list;
        } else {
            ret = (*list)->next;
        }

        if (sov->head) {
            if (ret && ret->next == NULL) {
                sov->list_mode = LM_NONE;
            }
            sov->head = false;
        } else {
            if (ret && ret->next == NULL) {
                sov->list_mode = LM_END;
            }
        }
    }

    return ret;
}

static void
end_list(Visitor *v, Error **errp)
{
    StringOutputVisitor *sov = DO_UPCAST(StringOutputVisitor, visitor, v);

    assert(sov->list_mode == LM_STARTED ||
           sov->list_mode == LM_END ||
           sov->list_mode == LM_IN_PROGRESS);
    sov->list_mode = LM_NONE;
    sov->head = true;
}

char *string_output_get_string(StringOutputVisitor *sov)
{
    char *string = sov->string;
    sov->string = NULL;
    return string;
}

Visitor *string_output_get_visitor(StringOutputVisitor *sov)
{
    return &sov->visitor;
}

void string_output_visitor_cleanup(StringOutputVisitor *sov)
{
    g_free(sov->string);
    g_free(sov);
}

StringOutputVisitor *string_output_visitor_new(bool human)
{
    StringOutputVisitor *v;

    v = g_malloc0(sizeof(*v));

    v->human = human;
    v->visitor.type_enum = output_type_enum;
    v->visitor.type_int = print_type_int;
    v->visitor.type_size = print_type_size;
    v->visitor.type_bool = print_type_bool;
    v->visitor.type_str = print_type_str;
    v->visitor.type_number = print_type_number;
    v->visitor.start_list = start_list;
    v->visitor.next_list = next_list;
    v->visitor.end_list = end_list;

    return v;
}
