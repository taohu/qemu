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
#include "qom/object_interfaces.h"

#define TYPE_MEMORY_BACKEND_RAM "memory-ram"
#define MEMORY_BACKEND_RAM(obj) \
    OBJECT_CHECK(HostMemoryBackendRam, (obj), TYPE_MEMORY_BACKEND_RAM)
#define MEMORY_BACKEND_RAM_GET_CLASS(obj) \
    OBJECT_GET_CLASS(HostMemoryBackendRamClass, (obj), \
                     TYPE_MEMORY_BACKEND_RAM)
#define MEMORY_BACKEND_RAM_CLASS(klass) \
    OBJECT_CLASS_CHECK(HostMemoryBackendRamClass, (klass), \
                       TYPE_MEMORY_BACKEND_RAM)

/**
 * @HostMemoryBackendRamClass:
 * @parent_complete: The parent class complete handler.
 */
typedef struct HostMemoryBackendRamClass {
    /*< private >*/
    HostMemoryBackendClass parent_class;
    /*< public >*/

    void (*parent_complete)(UserCreatable *uc, Error **errp);
} HostMemoryBackendRamClass;

static void
ram_backend_memory_init(UserCreatable *uc, Error **errp)
{
    HostMemoryBackend *backend = MEMORY_BACKEND(uc);
    HostMemoryBackendRamClass *mbrc = MEMORY_BACKEND_RAM_GET_CLASS(uc);

    if (!backend->size) {
        error_setg(errp, "can't create backend with size 0");
        return;
    }

    memory_region_init_ram(&backend->mr, OBJECT(backend),
                           object_get_canonical_path(OBJECT(backend)),
                           backend->size);
    mbrc->parent_complete(uc, errp);
}

static void
ram_backend_class_init(ObjectClass *oc, void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);
    HostMemoryBackendRamClass *mbrc = MEMORY_BACKEND_RAM_CLASS(oc);

    mbrc->parent_complete = ucc->complete;
    ucc->complete = ram_backend_memory_init;

}

static const TypeInfo ram_backend_info = {
    .name = TYPE_MEMORY_BACKEND_RAM,
    .parent = TYPE_MEMORY_BACKEND,
    .class_size = sizeof(HostMemoryBackendRamClass),
    .class_init = ram_backend_class_init,
};

static void register_types(void)
{
    type_register_static(&ram_backend_info);
}

type_init(register_types);
