/*
 * String parsing visitor
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
#include "qapi/string-input-visitor.h"
#include "qapi/visitor-impl.h"
#include "qapi/qmp/qerror.h"

enum ListMode
{
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

struct StringInputVisitor
{
    Visitor visitor;

    ListMode list_mode;

    /* When parsing a list of repeating options as integers, values of the form
     * "a-b", representing a closed interval, are allowed. Elements in the
     * range are generated individually.
     */
    union {
        int64_t s;
        uint64_t u;
    } range_next, range_limit;

    const char *string;
};

static void
start_list(Visitor *v, const char *name, Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);

    /* we can't traverse a list in a list */
    assert(siv->list_mode == LM_NONE);
    siv->list_mode = LM_STARTED;
}

static GenericList *
next_list(Visitor *v, GenericList **list, Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);
    GenericList **link;

    switch (siv->list_mode) {
    case LM_STARTED:
        siv->list_mode = LM_IN_PROGRESS;
        link = list;
        break;

    case LM_SIGNED_INTERVAL:
    case LM_UNSIGNED_INTERVAL:
        link = &(*list)->next;

        if (siv->list_mode == LM_SIGNED_INTERVAL) {
            if (siv->range_next.s < siv->range_limit.s) {
                ++siv->range_next.s;
                break;
            }
        } else if (siv->range_next.u < siv->range_limit.u) {
            ++siv->range_next.u;
            break;
        }
        siv->list_mode = LM_END;
        /* range has been completed, fall through */

    case LM_END:
        return NULL;

    case LM_IN_PROGRESS:
        link = &(*list)->next;
        break;

    default:
        abort();
    }

    *link = g_malloc0(sizeof **link);
    return *link;
}

static void
end_list(Visitor *v, Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);

    assert(siv->list_mode == LM_STARTED ||
           siv->list_mode == LM_END ||
           siv->list_mode == LM_IN_PROGRESS ||
           siv->list_mode == LM_SIGNED_INTERVAL ||
           siv->list_mode == LM_UNSIGNED_INTERVAL);
    siv->list_mode = LM_NONE;
}

static void parse_type_int(Visitor *v, int64_t *obj, const char *name,
                           Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);
    char *str = (char *) siv->string;
    long long val;
    char *endptr;

    if (siv->list_mode == LM_SIGNED_INTERVAL) {
        *obj = siv->range_next.s;
        return;
    }

    if (!siv->string) {
        error_set(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
                  "integer");
        return;
    }

    errno = 0;
    val = strtoll(siv->string, &endptr, 0);

    if (errno == 0 && endptr > str && INT64_MIN <= val && val <= INT64_MAX) {
        if (*endptr == '\0') {
            *obj = val;
            siv->list_mode = LM_END;
            return;
        }
        if (*endptr == '-' && siv->list_mode == LM_IN_PROGRESS) {
            long long val2;

            str = endptr + 1;
            val2 = strtoll(str, &endptr, 0);
            if (errno == 0 && endptr > str && *endptr == '\0' &&
                INT64_MIN <= val2 && val2 <= INT64_MAX && val <= val2 &&
                (val > INT64_MAX - 65536 ||
                 val2 < val + 65536)) {
                siv->range_next.s = val;
                siv->range_limit.s = val2;
                siv->list_mode = LM_SIGNED_INTERVAL;

                /* as if entering on the top */
                *obj = siv->range_next.s;
                return;
            }
        }
    }
    error_set(errp, QERR_INVALID_PARAMETER_VALUE, name,
              (siv->list_mode == LM_NONE) ? "an int64 value" :
                                           "an int64 value or range");
}

static void parse_type_bool(Visitor *v, bool *obj, const char *name,
                            Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);

    if (siv->string) {
        if (!strcasecmp(siv->string, "on") ||
            !strcasecmp(siv->string, "yes") ||
            !strcasecmp(siv->string, "true")) {
            *obj = true;
            return;
        }
        if (!strcasecmp(siv->string, "off") ||
            !strcasecmp(siv->string, "no") ||
            !strcasecmp(siv->string, "false")) {
            *obj = false;
            return;
        }
    }

    error_set(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
              "boolean");
}

static void parse_type_str(Visitor *v, char **obj, const char *name,
                           Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);
    if (siv->string) {
        *obj = g_strdup(siv->string);
    } else {
        error_set(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
                  "string");
    }
}

static void parse_type_number(Visitor *v, double *obj, const char *name,
                              Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);
    char *endp = (char *) siv->string;
    double val;

    errno = 0;
    if (siv->string) {
        val = strtod(siv->string, &endp);
    }
    if (!siv->string || errno || endp == siv->string || *endp) {
        error_set(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
                  "number");
        return;
    }

    *obj = val;
}

static void parse_type_size(Visitor *v, uint64_t *obj, const char *name,
                            Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);
    int64_t val;
    char *endp;

    val = strtosz_suffix(siv->string ? siv->string : "", &endp,
                         STRTOSZ_DEFSUFFIX_B);
    if (val < 0 || *endp != '\0') {
        error_set(errp, QERR_INVALID_PARAMETER_VALUE, name,
                  "a size value representible as a non-negative int64");
        return;
    }
    *obj = val;
}

static void parse_start_optional(Visitor *v, bool *present,
                                 const char *name, Error **errp)
{
    StringInputVisitor *siv = DO_UPCAST(StringInputVisitor, visitor, v);

    if (!siv->string) {
        *present = false;
        return;
    }

    *present = true;
}

Visitor *string_input_get_visitor(StringInputVisitor *v)
{
    return &v->visitor;
}

void string_input_visitor_cleanup(StringInputVisitor *v)
{
    g_free(v);
}

StringInputVisitor *string_input_visitor_new(const char *str)
{
    StringInputVisitor *v;

    v = g_malloc0(sizeof(*v));

    v->visitor.type_enum = input_type_enum;
    v->visitor.type_int = parse_type_int;
    v->visitor.type_bool = parse_type_bool;
    v->visitor.type_str = parse_type_str;
    v->visitor.type_number = parse_type_number;
    v->visitor.type_size = parse_type_size;
    v->visitor.start_list = start_list;
    v->visitor.next_list = next_list;
    v->visitor.end_list = end_list;
    v->visitor.start_optional = parse_start_optional;

    v->string = str;
    return v;
}
