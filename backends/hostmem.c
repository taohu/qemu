/*
 * QEMU Host Memory Backend
 *
 * Copyright (C) 2013 Red Hat Inc
 *
 * Authors:
 *   Igor Mammedov <imammedo@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#include "sysemu/hostmem.h"
#include "qapi/visitor.h"
#include "qapi-visit.h"
#include "qapi/qmp/qerror.h"
#include "qemu/config-file.h"
#include "qom/object_interfaces.h"
#ifdef CONFIG_NUMA
#include <numaif.h>
#endif
#include "qmp-commands.h"

static void
hostmemory_backend_get_size(Object *obj, Visitor *v, void *opaque,
                            const char *name, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);
    uint64_t value = backend->size;

    visit_type_size(v, &value, name, errp);
}

static void
hostmemory_backend_set_size(Object *obj, Visitor *v, void *opaque,
                            const char *name, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);
    uint64_t value;

    if (memory_region_size(&backend->mr)) {
        error_setg(errp, "cannot change property value\n");
        return;
    }

    visit_type_size(v, &value, name, errp);
    if (error_is_set(errp)) {
        return;
    }
    if (!value) {
        error_setg(errp, "Property '%s.%s' doesn't take value '%" PRIu64 "'",
                   object_get_typename(obj), name , value);
        return;
    }
    backend->size = value;
}

static void
get_host_nodes(Object *obj, Visitor *v, void *opaque, const char *name,
               Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);
    uint16List *host_nodes = NULL;
    uint16List **node = &host_nodes;
    unsigned long value;

    value = find_first_bit(backend->host_nodes, MAX_NODES);
    if (value == MAX_NODES) {
        return;
    }

    *node = g_malloc0(sizeof(**node));
    (*node)->value = value;
    node = &(*node)->next;

    do {
        value = find_next_bit(backend->host_nodes, MAX_NODES, value + 1);
        if (value == MAX_NODES) {
            break;
        }

        *node = g_malloc0(sizeof(**node));
        (*node)->value = value;
        node = &(*node)->next;
    } while (true);

    visit_type_uint16List(v, &host_nodes, name, errp);
}

static void
set_host_nodes(Object *obj, Visitor *v, void *opaque, const char *name,
               Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);
    uint16List *l = NULL;

    visit_type_uint16List(v, &l, name, errp);

    while (l) {
        bitmap_set(backend->host_nodes, l->value, 1);
        l = l->next;
    }
}

static void
get_policy(Object *obj, Visitor *v, void *opaque, const char *name,
           Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);
    int policy = backend->policy;

    visit_type_enum(v, &policy, HostMemPolicy_lookup, NULL, name, errp);
}

static void
set_policy(Object *obj, Visitor *v, void *opaque, const char *name,
           Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);
    int policy;

    visit_type_enum(v, &policy, HostMemPolicy_lookup, NULL, name, errp);
    backend->policy = policy;
}

static void hostmemory_backend_initfn(Object *obj)
{
    object_property_add(obj, "size", "int",
                        hostmemory_backend_get_size,
                        hostmemory_backend_set_size, NULL, NULL, NULL);
    object_property_add(obj, "host-nodes", "int",
                        get_host_nodes,
                        set_host_nodes, NULL, NULL, NULL);
    object_property_add(obj, "policy", "str",
                        get_policy,
                        set_policy, NULL, NULL, NULL);
}

static void hostmemory_backend_finalize(Object *obj)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);

    if (memory_region_size(&backend->mr)) {
        memory_region_destroy(&backend->mr);
    }
}

static void
hostmemory_backend_memory_init(UserCreatable *uc, Error **errp)
{
#ifdef CONFIG_NUMA
    HostMemoryBackend *backend = MEMORY_BACKEND(uc);
    void *p = memory_region_get_ram_ptr(&backend->mr);
    unsigned long maxnode = find_last_bit(backend->host_nodes, MAX_NODES);

    /* This is a workaround for a long standing bug in Linux'
     * mbind implementation, which cuts off the last specified
     * node.
     */
    if (mbind(p, backend->size, backend->policy, backend->host_nodes,
              maxnode + 2, 0)) {
        error_setg_errno(errp, errno,
                         "cannot bind memory to host NUMA nodes\n");
        return;
    }
#endif
}

MemoryRegion *
host_memory_backend_get_memory(HostMemoryBackend *backend, Error **errp)
{
    return memory_region_size(&backend->mr) ? &backend->mr : NULL;
}

static void
hostmemory_backend_class_init(ObjectClass *oc, void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);

    ucc->complete = hostmemory_backend_memory_init;
}

static const TypeInfo hostmemory_backend_info = {
    .name = TYPE_MEMORY_BACKEND,
    .parent = TYPE_OBJECT,
    .abstract = true,
    .class_size = sizeof(HostMemoryBackendClass),
    .class_init = hostmemory_backend_class_init,
    .instance_size = sizeof(HostMemoryBackend),
    .instance_init = hostmemory_backend_initfn,
    .instance_finalize = hostmemory_backend_finalize,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_USER_CREATABLE },
        { }
    }
};

static void register_types(void)
{
    type_register_static(&hostmemory_backend_info);
}

type_init(register_types);
