/*
 * QEMU Host Memory Backend
 *
 * Copyright (C) 2013-2014 Red Hat Inc
 *
 * Authors:
 *   Igor Mammedov <imammedo@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#include "sysemu/hostmem.h"
#include "sysemu/sysemu.h"
#include "qapi/visitor.h"
#include "qapi/qmp/qerror.h"
#include "qemu/config-file.h"
#include "qom/object_interfaces.h"

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
    Error *local_err = NULL;
    uint64_t value;

    if (memory_region_size(&backend->mr)) {
        error_setg(&local_err, "cannot change property value");
        goto out;
    }

    visit_type_size(v, &value, name, &local_err);
    if (local_err) {
        goto out;
    }
    if (!value) {
        error_setg(&local_err, "Property '%s.%s' doesn't take value '%"
                   PRIu64 "'", object_get_typename(obj), name, value);
        goto out;
    }
    backend->size = value;
out:
    error_propagate(errp, local_err);
}

static bool host_memory_backend_get_merge(Object *obj, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);

    return backend->merge;
}

static void host_memory_backend_set_merge(Object *obj, bool value, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);

    if (!memory_region_size(&backend->mr)) {
        backend->merge = value;
        return;
    }

    if (value != backend->merge) {
        void *ptr = memory_region_get_ram_ptr(&backend->mr);
        uint64_t sz = memory_region_size(&backend->mr);

        qemu_madvise(ptr, sz,
                     value ? QEMU_MADV_MERGEABLE : QEMU_MADV_UNMERGEABLE);
        backend->merge = value;
    }
}

static bool host_memory_backend_get_dump(Object *obj, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);

    return backend->dump;
}

static void host_memory_backend_set_dump(Object *obj, bool value, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);

    if (!memory_region_size(&backend->mr)) {
        backend->dump = value;
        return;
    }

    if (value != backend->dump) {
        void *ptr = memory_region_get_ram_ptr(&backend->mr);
        uint64_t sz = memory_region_size(&backend->mr);

        qemu_madvise(ptr, sz,
                     value ? QEMU_MADV_DODUMP : QEMU_MADV_DONTDUMP);
        backend->dump = value;
    }
}

static void hostmemory_backend_init(Object *obj)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);

    backend->merge = qemu_opt_get_bool(qemu_get_machine_opts(),
                                       "mem-merge", true);
    backend->dump = qemu_opt_get_bool(qemu_get_machine_opts(),
                                      "dump-guest-core", true);

    object_property_add_bool(obj, "merge",
                        host_memory_backend_get_merge,
                        host_memory_backend_set_merge, NULL);
    object_property_add_bool(obj, "dump",
                        host_memory_backend_get_dump,
                        host_memory_backend_set_dump, NULL);
    object_property_add(obj, "size", "int",
                        hostmemory_backend_get_size,
                        hostmemory_backend_set_size, NULL, NULL, NULL);
}

static void hostmemory_backend_finalize(Object *obj)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(obj);

    if (memory_region_size(&backend->mr)) {
        memory_region_destroy(&backend->mr);
    }
}

MemoryRegion *
host_memory_backend_get_memory(HostMemoryBackend *backend, Error **errp)
{
    return memory_region_size(&backend->mr) ? &backend->mr : NULL;
}

static void
host_memory_backend_memory_init(UserCreatable *uc, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(uc);
    HostMemoryBackendClass *bc = MEMORY_BACKEND_GET_CLASS(uc);
    Error *local_err = NULL;
    void *ptr;
    uint64_t sz;

    if (bc->alloc) {
        bc->alloc(backend, &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }

        ptr = memory_region_get_ram_ptr(&backend->mr);
        sz = memory_region_size(&backend->mr);

        if (backend->merge) {
            qemu_madvise(ptr, sz, QEMU_MADV_MERGEABLE);
        }
        if (!backend->dump) {
            qemu_madvise(ptr, sz, QEMU_MADV_DONTDUMP);
        }
    }
}

static void
host_memory_backend_class_init(ObjectClass *oc, void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);

    ucc->complete = host_memory_backend_memory_init;
}

static const TypeInfo hostmemory_backend_info = {
    .name = TYPE_MEMORY_BACKEND,
    .parent = TYPE_OBJECT,
    .abstract = true,
    .class_size = sizeof(HostMemoryBackendClass),
    .class_init = host_memory_backend_class_init,
    .instance_size = sizeof(HostMemoryBackend),
    .instance_init = hostmemory_backend_init,
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
