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
#include "sysemu/sysemu.h"
#include "qemu/bitmap.h"
#include "qapi-visit.h"
#include "qemu/config-file.h"
#include "qapi/opts-visitor.h"

#define TYPE_MEMORY_BACKEND_RAM "memory-ram"
#define MEMORY_BACKEND_RAM(obj) \
    OBJECT_CHECK(HostMemoryBackendRam, (obj), TYPE_MEMORY_BACKEND_RAM)

typedef struct HostMemoryBackendRam HostMemoryBackendRam;

/**
 * @HostMemoryBackendRam
 *
 * @parent: opaque parent object container
 * @host_nodes: host nodes bitmap used for memory policy
 * @policy: host memory policy
 * @relative: if the host nodes bitmap is relative
 */
struct HostMemoryBackendRam {
    /* private */
    HostMemoryBackend parent;

    DECLARE_BITMAP(host_nodes, MAX_NODES);
    HostMemPolicy policy;
    bool relative;
};

static void
get_host_nodes(Object *obj, Visitor *v, void *opaque, const char *name,
               Error **errp)
{
    HostMemoryBackendRam *ram_backend = MEMORY_BACKEND_RAM(obj);
    uint16List *host_nodes = NULL;
    uint16List **node = &host_nodes;
    unsigned long value;

    value = find_first_bit(ram_backend->host_nodes, MAX_NODES);
    if (value == MAX_NODES) {
        return;
    }

    *node = g_malloc0(sizeof(**node));
    (*node)->value = value;
    node = &(*node)->next;

    do {
        value = find_next_bit(ram_backend->host_nodes, MAX_NODES, value + 1);
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
    HostMemoryBackendRam *ram_backend = MEMORY_BACKEND_RAM(obj);
    uint16List * l = NULL;

    visit_type_uint16List(v, &l, name, errp);

    while (l) {
        bitmap_set(ram_backend->host_nodes, l->value, 1);
        l = l->next;
    }
}

static const char *policies[HOST_MEM_POLICY_MAX + 1] = {
    [HOST_MEM_POLICY_DEFAULT] = "default",
    [HOST_MEM_POLICY_PREFERRED] = "preferred",
    [HOST_MEM_POLICY_MEMBIND] = "membind",
    [HOST_MEM_POLICY_INTERLEAVE] = "interleave",
    [HOST_MEM_POLICY_MAX] = NULL,
};

static void
get_policy(Object *obj, Visitor *v, void *opaque, const char *name,
           Error **errp)
{
    HostMemoryBackendRam *ram_backend = MEMORY_BACKEND_RAM(obj);
    int policy = ram_backend->policy;

    visit_type_enum(v, &policy, policies, NULL, name, errp);
}

static void
set_policy(Object *obj, Visitor *v, void *opaque, const char *name,
           Error **errp)
{
    HostMemoryBackendRam *ram_backend = MEMORY_BACKEND_RAM(obj);
    int policy;

    visit_type_enum(v, &policy, policies, NULL, name, errp);
    ram_backend->policy = policy;
}


static bool get_relative(Object *obj, Error **errp)
{
    HostMemoryBackendRam *ram_backend = MEMORY_BACKEND_RAM(obj);

    return ram_backend->relative;
}

static void set_relative(Object *obj, bool value, Error **errp)
{
    HostMemoryBackendRam *ram_backend = MEMORY_BACKEND_RAM(obj);

    ram_backend->relative = value;
}

#include <sys/syscall.h>
#include <numaif.h>
#ifndef MPOL_F_RELATIVE_NODES
#define MPOL_F_RELATIVE_NODES (1 << 14)
#define MPOL_F_STATIC_NODES   (1 << 15)
#endif

static void
ram_backend_memory_init(HostMemoryBackend *backend, Error **errp)
{
    HostMemoryBackendRam *ram_backend = MEMORY_BACKEND_RAM(backend);
    int mode = ram_backend->policy;
    void *p;
    unsigned long maxnode;

    if (!memory_region_size(&backend->mr)) {
        memory_region_init_ram(&backend->mr, OBJECT(backend),
                               object_get_canonical_path(OBJECT(backend)),
                               backend->size);

        p = memory_region_get_ram_ptr(&backend->mr);
        maxnode = find_last_bit(ram_backend->host_nodes, MAX_NODES);

        mode |= ram_backend->relative ? MPOL_F_RELATIVE_NODES :
            MPOL_F_STATIC_NODES;

        syscall(SYS_mbind, p, backend->size, mode,
                ram_backend->host_nodes, maxnode + 2, 0);
    }
}

static void
ram_backend_initfn(Object *obj)
{
    object_property_add(obj, "host-nodes", "int",
                        get_host_nodes,
                        set_host_nodes, NULL, NULL, NULL);
    object_property_add(obj, "policy", "string",
                        get_policy,
                        set_policy, NULL, NULL, NULL);
    object_property_add_bool(obj, "relative",
                             get_relative, set_relative, NULL);
}

static void
ram_backend_class_init(ObjectClass *oc, void *data)
{
    HostMemoryBackendClass *bc = MEMORY_BACKEND_CLASS(oc);

    bc->memory_init = ram_backend_memory_init;
}

static const TypeInfo ram_backend_info = {
    .name = TYPE_MEMORY_BACKEND_RAM,
    .parent = TYPE_MEMORY_BACKEND,
    .class_init = ram_backend_class_init,
    .instance_size = sizeof(HostMemoryBackendRam),
    .instance_init = ram_backend_initfn,
};

static void register_types(void)
{
    type_register_static(&ram_backend_info);
}

type_init(register_types);
