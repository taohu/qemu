#ifndef QEMU_DIMM_H
#define QEMU_DIMM_H

#include "qemu-common.h"
#include "exec/memory.h"
#include "hw/sysbus.h"
#include "qapi-types.h"
#include "sysemu/cpus.h"
#define MAX_DIMMS 255
#define DIMM_BITMAP_BYTES ((MAX_DIMMS + 7) / 8)
#define DEFAULT_DIMMSIZE (1024*1024*1024)

typedef enum {
    DIMM_REMOVE_SUCCESS = 0,
    DIMM_REMOVE_FAIL = 1,
    DIMM_ADD_SUCCESS = 2,
    DIMM_ADD_FAIL = 3
} dimm_hp_result_code;

typedef enum {
    DIMM_NO_PENDING = 0,
    DIMM_ADD_PENDING = 1,
    DIMM_REMOVE_PENDING = 2,
} dimm_hp_pending_code;

#define TYPE_DIMM "dimm"
#define DIMM(obj) \
    OBJECT_CHECK(DimmDevice, (obj), TYPE_DIMM)
#define DIMM_CLASS(klass) \
    OBJECT_CLASS_CHECK(DimmDeviceClass, (klass), TYPE_DIMM)
#define DIMM_GET_CLASS(obj) \
    OBJECT_GET_CLASS(DimmDeviceClass, (obj), TYPE_DIMM)

typedef struct DimmDevice DimmDevice;
typedef QTAILQ_HEAD(DimmConfiglist, DimmConfig) DimmConfiglist;

typedef struct DimmDeviceClass {
    DeviceClass parent_class;

    int (*init)(DimmDevice *dev);
} DimmDeviceClass;

struct DimmDevice {
    DeviceState qdev;
    uint32_t idx; /* index in memory hotplug register/bitmap */
    ram_addr_t start; /* starting physical address */
    ram_addr_t size;
    uint32_t node; /* numa node proximity */
    uint32_t populated; /* 1 means device has been hotplugged. Default is 0. */
    MemoryRegion *mr; /* MemoryRegion for this slot. !NULL only if populated */
    dimm_hp_pending_code pending; /* pending hot operation for this dimm */
    QTAILQ_ENTRY(DimmDevice) nextdimm;
};

typedef struct DimmConfig {
    const char *name;
    uint32_t idx; /* index in linear memory hotplug bitmap */
    const char *bus_name;
    ram_addr_t start; /* starting physical address */
    ram_addr_t size;
    uint32_t node; /* numa node proximity */
    uint32_t populated; /* 1 means device has been hotplugged. Default is 0. */
    QTAILQ_ENTRY(DimmConfig) nextdimmcfg;
} DimmConfig;

typedef int (*dimm_hotplug_fn)(DeviceState *qdev, DimmDevice *dev, int add);
typedef hwaddr(*dimm_calcoffset_fn)(DeviceState *dev, uint64_t size);

#define TYPE_DIMM_BUS "dimmbus"
#define DIMM_BUS(obj) OBJECT_CHECK(DimmBus, (obj), TYPE_DIMM_BUS)

typedef struct DimmBus {
    BusState qbus;
    DeviceState *dimm_hotplug_qdev;
    dimm_hotplug_fn dimm_hotplug;
    DimmConfiglist dimmconfig_list;
    dimm_hotplug_fn dimm_revert;
    QTAILQ_HEAD(Dimmlist, DimmDevice) dimmlist;
    QTAILQ_HEAD(dimm_hp_result_head, dimm_hp_result)  dimm_hp_result_queue;
    QLIST_ENTRY(DimmBus) next;
} DimmBus;

struct dimm_hp_result {
    const char *dimmname;
    dimm_hp_result_code ret;
    QTAILQ_ENTRY(dimm_hp_result) next;
};

void dimm_bus_hotplug(dimm_hotplug_fn hotplug, dimm_hotplug_fn revert,
                      DeviceState *qdev);
void dimm_setup_fwcfg_layout(uint64_t *fw_cfg_slots);
DimmBus *dimm_bus_create(Object *parent, const char *name, uint32_t max_dimms,
    dimm_calcoffset_fn pmc_set_offset);
void dimm_config_create(char *id, uint64_t size, const char *bus, uint64_t node,
        uint32_t dimm_idx, uint32_t populated);
uint64_t get_hp_memory_total(void);
void dimm_notify(uint32_t idx, uint32_t event);

#endif
