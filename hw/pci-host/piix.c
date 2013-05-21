/*
 * QEMU i440FX/PIIX3 PCI Bridge Emulation
 *
 * Copyright (c) 2006 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/isa/isa.h"
#include "hw/sysbus.h"
#include "qemu/range.h"
#include "hw/xen/xen.h"
#include "hw/pci-host/pam.h"
#include "sysemu/sysemu.h"
#include "hw/timer/hpet.h"
#include "hw/timer/mc146818rtc.h"
#include "hw/audio/pcspk.h"
#include "hw/timer/i8254.h"

/*
 * I440FX chipset data sheet.
 * http://download.intel.com/design/chipsets/datashts/29054901.pdf
 */

#define PIIX_NUM_PIC_IRQS       16      /* i8259 * 2 */
#define PIIX_NUM_PIRQS          4ULL    /* PIRQ[A-D] */
#define XEN_PIIX_NUM_PIRQS      128ULL
#define PIIX_PIRQC              0x60

/*
 * Reset Control Register: PCI-accessible ISA-Compatible Register at address
 * 0xcf9, provided by the PCI/ISA bridge (PIIX3 PCI function 0, 8086:7000).
 */
#define RCR_IOPORT 0xcf9

#define TYPE_PIIX3 "PIIX3"
#define PIIX3(obj) OBJECT_CHECK(PIIX3State, (obj), TYPE_PIIX3)

typedef struct PIIX3State {
    PCIDevice dev;

    /*
     * bitmap to track pic levels.
     * The pic level is the logical OR of all the PCI irqs mapped to it
     * So one PIC level is tracked by PIIX_NUM_PIRQS bits.
     *
     * PIRQ is mapped to PIC pins, we track it by
     * PIIX_NUM_PIRQS * PIIX_NUM_PIC_IRQS = 64 bits with
     * pic_irq * PIIX_NUM_PIRQS + pirq
     */
#if PIIX_NUM_PIC_IRQS * PIIX_NUM_PIRQS > 64
#error "unable to encode pic state in 64bit in pic_levels."
#endif
    uint64_t pic_levels;

    ISABus *bus;
    DeviceState *hpet;
    ISADevice *rtc;
    ISADevice *pit;
    ISADevice *pcspk;

    qemu_irq *pic;

    /* This member isn't used. Just for save/load compatibility */
    int32_t pci_irq_levels_vmstate[PIIX_NUM_PIRQS];

    /* Reset Control Register contents */
    uint8_t rcr;

    /* IO memory region for Reset Control Register (RCR_IOPORT) */
    MemoryRegion rcr_mem;
} PIIX3State;

#define TYPE_I440FX_PMC_DEVICE "i440FX-PMC"
#define I440FX_PMC_DEVICE(obj) \
    OBJECT_CHECK(I440FXPMCState, (obj), TYPE_I440FX_PMC_DEVICE)

struct I440FXPMCState {
    PCIDevice dev;
    MemoryRegion *system_memory;
    MemoryRegion *pci_address_space;
    MemoryRegion *ram_memory;
    MemoryRegion pci_hole;
    MemoryRegion pci_hole_64bit;
    PAMMemoryRegion pam_regions[13];
    MemoryRegion smram_region;
    uint8_t smm_enabled;
};

#define TYPE_I440FX_DEVICE "i440FX"
#define I440FX_DEVICE(obj) \
    OBJECT_CHECK(I440FXState, (obj), TYPE_I440FX_DEVICE)

typedef struct I440FXState {
    PCIHostState parent_obj;
    MemoryRegion *address_space_io;
    MemoryRegion *pci_address_space;

    PIIX3State piix3;
} I440FXState;

#define I440FX_PAM      0x59
#define I440FX_PAM_SIZE 7
#define I440FX_SMRAM    0x72

static void piix3_set_irq(void *opaque, int pirq, int level);
static PCIINTxRoute piix3_route_intx_pin_to_irq(void *opaque, int pci_intx);
static void piix3_write_config_xen(PCIDevice *dev,
                               uint32_t address, uint32_t val, int len);

/* return the global irq number corresponding to a given device irq
   pin. We could also use the bus number to have a more precise
   mapping. */
static int pci_slot_get_pirq(PCIDevice *pci_dev, int pci_intx)
{
    int slot_addend;
    slot_addend = (pci_dev->devfn >> 3) - 1;
    return (pci_intx + slot_addend) & 3;
}

static void i440fx_pmc_update_memory_mappings(I440FXPMCState *d)
{
    int i;

    memory_region_transaction_begin();
    for (i = 0; i < 13; i++) {
        pam_update(&d->pam_regions[i], i,
                   d->dev.config[I440FX_PAM + ((i + 1) / 2)]);
    }
    smram_update(&d->smram_region, d->dev.config[I440FX_SMRAM], d->smm_enabled);
    memory_region_transaction_commit();
}

static void i440fx_set_smm(int val, void *arg)
{
    I440FXPMCState *d = arg;

    memory_region_transaction_begin();
    smram_set_smm(&d->smm_enabled, val, d->dev.config[I440FX_SMRAM],
                  &d->smram_region);
    memory_region_transaction_commit();
}


static void i440fx_write_config(PCIDevice *dev,
                                uint32_t address, uint32_t val, int len)
{
    I440FXPMCState *d = I440FX_PMC_DEVICE(dev);

    /* XXX: implement SMRAM.D_LOCK */
    pci_default_write_config(dev, address, val, len);
    if (ranges_overlap(address, len, I440FX_PAM, I440FX_PAM_SIZE) ||
        range_covers_byte(address, len, I440FX_SMRAM)) {
        i440fx_pmc_update_memory_mappings(d);
    }
}

static int i440fx_load_old(QEMUFile* f, void *opaque, int version_id)
{
    I440FXPMCState *d = opaque;
    int ret, i;

    ret = pci_device_load(&d->dev, f);
    if (ret < 0)
        return ret;
    i440fx_pmc_update_memory_mappings(d);
    qemu_get_8s(f, &d->smm_enabled);

    if (version_id == 2) {
        for (i = 0; i < PIIX_NUM_PIRQS; i++) {
            qemu_get_be32(f); /* dummy load for compatibility */
        }
    }

    return 0;
}

static int i440fx_post_load(void *opaque, int version_id)
{
    I440FXPMCState *d = opaque;

    i440fx_pmc_update_memory_mappings(d);
    return 0;
}

static const VMStateDescription vmstate_i440fx_pmc = {
    .name = TYPE_I440FX_PMC_DEVICE,
    .version_id = 3,
    .minimum_version_id = 3,
    .minimum_version_id_old = 1,
    .load_state_old = i440fx_load_old,
    .post_load = i440fx_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_PCI_DEVICE(dev, I440FXPMCState),
        VMSTATE_UINT8(smm_enabled, I440FXPMCState),
        VMSTATE_END_OF_LIST()
    }
};

static int i440fx_realize(SysBusDevice *dev)
{
    PCIHostState *s = PCI_HOST_BRIDGE(dev);
    I440FXState *f = I440FX_DEVICE(dev);

    s->bus = pci_bus_new(DEVICE(f), NULL, f->pci_address_space,
                         f->address_space_io, 0, TYPE_PCI_BUS);

    memory_region_init_io(&s->conf_mem, &pci_host_conf_le_ops, s,
                          "pci-conf-idx", 4);
    sysbus_add_io(dev, 0xcf8, &s->conf_mem);
    sysbus_init_ioports(&s->busdev, 0xcf8, 4);

    memory_region_init_io(&s->data_mem, &pci_host_data_le_ops, s,
                          "pci-conf-data", 4);
    sysbus_add_io(dev, 0xcfc, &s->data_mem);
    sysbus_init_ioports(&s->busdev, 0xcfc, 4);

    qdev_set_parent_bus(DEVICE(&f->piix3), BUS(s->bus));
    qdev_init_nofail(DEVICE(&f->piix3));

    if (xen_enabled()) {
        pci_bus_irqs(s->bus, xen_piix3_set_irq, xen_pci_slot_get_pirq,
                     &f->piix3, XEN_PIIX_NUM_PIRQS);
    } else {
        pci_bus_irqs(s->bus, piix3_set_irq, pci_slot_get_pirq, &f->piix3,
                     PIIX_NUM_PIRQS);
        pci_bus_set_route_irq_fn(s->bus, piix3_route_intx_pin_to_irq);
    }

    return 0;
}

static void i440fx_initfn(Object *obj)
{
    I440FXState *f = I440FX_DEVICE(obj);

    /* Xen supports additional interrupt routes from the PCI devices to
     * the IOAPIC: the four pins of each PCI device on the bus are also
     * connected to the IOAPIC directly.
     * These additional routes can be discovered through ACPI. */
    if (xen_enabled()) {
        object_initialize(&f->piix3, "PIIX3-xen");
    } else {
        object_initialize(&f->piix3, "PIIX3");
    }
    qdev_prop_set_uint32(DEVICE(&f->piix3), "addr", PCI_DEVFN(1, 0));
    qdev_prop_set_bit(DEVICE(&f->piix3), "multifunction", true);
    object_property_add_child(OBJECT(f), "piix3", OBJECT(&f->piix3), NULL);
}

static int i440fx_pmc_initfn(PCIDevice *dev)
{
    I440FXPMCState *d = I440FX_PMC_DEVICE(dev);

    d->dev.config[I440FX_SMRAM] = 0x02;

    cpu_smm_register(&i440fx_set_smm, d);
    return 0;
}

static PCIBus *i440fx_common_init(const char *device_name,
                                  int *piix3_devfn,
                                  ISABus **isa_bus, qemu_irq *pic,
                                  MemoryRegion *address_space_mem,
                                  MemoryRegion *address_space_io,
                                  ram_addr_t ram_size,
                                  hwaddr pci_hole_start,
                                  hwaddr pci_hole_size,
                                  hwaddr pci_hole64_start,
                                  hwaddr pci_hole64_size,
                                  MemoryRegion *pci_address_space,
                                  MemoryRegion *ram_memory)
{
    PCIDevice *d;
    PCIHostState *s;
    PIIX3State *piix3;
    I440FXPMCState *f;
    I440FXState *i440fx;
    unsigned i;

    i440fx = I440FX_DEVICE(object_new(TYPE_I440FX_DEVICE));
    s = PCI_HOST_BRIDGE(i440fx);

    i440fx->address_space_io = address_space_io;
    i440fx->pci_address_space = pci_address_space;

    piix3 = &i440fx->piix3;
    piix3->pic = pic;

    object_property_add_child(qdev_get_machine(), "i440fx",
                              OBJECT(i440fx), NULL);
    qdev_set_parent_bus(DEVICE(i440fx), sysbus_get_default());
    qdev_init_nofail(DEVICE(i440fx));

    d = pci_create_simple(s->bus, 0, device_name);
    f = I440FX_PMC_DEVICE(d);
    f->system_memory = address_space_mem;
    f->pci_address_space = pci_address_space;
    f->ram_memory = ram_memory;
    memory_region_init_alias(&f->pci_hole, "pci-hole", f->pci_address_space,
                             pci_hole_start, pci_hole_size);
    memory_region_add_subregion(f->system_memory, pci_hole_start, &f->pci_hole);
    memory_region_init_alias(&f->pci_hole_64bit, "pci-hole64",
                             f->pci_address_space,
                             pci_hole64_start, pci_hole64_size);
    if (pci_hole64_size) {
        memory_region_add_subregion(f->system_memory, pci_hole64_start,
                                    &f->pci_hole_64bit);
    }
    memory_region_init_alias(&f->smram_region, "smram-region",
                             f->pci_address_space, 0xa0000, 0x20000);
    memory_region_add_subregion_overlap(f->system_memory, 0xa0000,
                                        &f->smram_region, 1);
    memory_region_set_enabled(&f->smram_region, false);
    init_pam(f->ram_memory, f->system_memory, f->pci_address_space,
             &f->pam_regions[0], PAM_BIOS_BASE, PAM_BIOS_SIZE);
    for (i = 0; i < 12; ++i) {
        init_pam(f->ram_memory, f->system_memory, f->pci_address_space,
                 &f->pam_regions[i+1], PAM_EXPAN_BASE + i * PAM_EXPAN_SIZE,
                 PAM_EXPAN_SIZE);
    }

    *isa_bus = piix3->bus;
    *piix3_devfn = piix3->dev.devfn;

    ram_size = ram_size / 8 / 1024 / 1024;
    if (ram_size > 255)
        ram_size = 255;
    f->dev.config[0x57] = ram_size;

    i440fx_pmc_update_memory_mappings(f);

    return s->bus;
}

PCIBus *i440fx_init(int *piix3_devfn,
                    ISABus **isa_bus, qemu_irq *pic,
                    MemoryRegion *address_space_mem,
                    MemoryRegion *address_space_io,
                    ram_addr_t ram_size,
                    hwaddr pci_hole_start,
                    hwaddr pci_hole_size,
                    hwaddr pci_hole64_start,
                    hwaddr pci_hole64_size,
                    MemoryRegion *pci_memory, MemoryRegion *ram_memory)

{
    PCIBus *b;

    b = i440fx_common_init(TYPE_I440FX_PMC_DEVICE,
                           piix3_devfn, isa_bus, pic,
                           address_space_mem, address_space_io, ram_size,
                           pci_hole_start, pci_hole_size,
                           pci_hole64_start, pci_hole64_size,
                           pci_memory, ram_memory);
    return b;
}

/* PIIX3 PCI to ISA bridge */
static void piix3_set_irq_pic(PIIX3State *piix3, int pic_irq)
{
    qemu_set_irq(piix3->pic[pic_irq],
                 !!(piix3->pic_levels &
                    (((1ULL << PIIX_NUM_PIRQS) - 1) <<
                     (pic_irq * PIIX_NUM_PIRQS))));
}

static void piix3_set_irq_level(PIIX3State *piix3, int pirq, int level)
{
    int pic_irq;
    uint64_t mask;

    pic_irq = piix3->dev.config[PIIX_PIRQC + pirq];
    if (pic_irq >= PIIX_NUM_PIC_IRQS) {
        return;
    }

    mask = 1ULL << ((pic_irq * PIIX_NUM_PIRQS) + pirq);
    piix3->pic_levels &= ~mask;
    piix3->pic_levels |= mask * !!level;

    piix3_set_irq_pic(piix3, pic_irq);
}

static void piix3_set_irq(void *opaque, int pirq, int level)
{
    PIIX3State *piix3 = opaque;
    piix3_set_irq_level(piix3, pirq, level);
}

static PCIINTxRoute piix3_route_intx_pin_to_irq(void *opaque, int pin)
{
    PIIX3State *piix3 = opaque;
    int irq = piix3->dev.config[PIIX_PIRQC + pin];
    PCIINTxRoute route;

    if (irq < PIIX_NUM_PIC_IRQS) {
        route.mode = PCI_INTX_ENABLED;
        route.irq = irq;
    } else {
        route.mode = PCI_INTX_DISABLED;
        route.irq = -1;
    }
    return route;
}

/* irq routing is changed. so rebuild bitmap */
static void piix3_update_irq_levels(PIIX3State *piix3)
{
    int pirq;

    piix3->pic_levels = 0;
    for (pirq = 0; pirq < PIIX_NUM_PIRQS; pirq++) {
        piix3_set_irq_level(piix3, pirq,
                            pci_bus_get_irq_level(piix3->dev.bus, pirq));
    }
}

static void piix3_write_config(PCIDevice *dev,
                               uint32_t address, uint32_t val, int len)
{
    pci_default_write_config(dev, address, val, len);
    if (ranges_overlap(address, len, PIIX_PIRQC, 4)) {
        PIIX3State *piix3 = DO_UPCAST(PIIX3State, dev, dev);
        int pic_irq;

        pci_bus_fire_intx_routing_notifier(piix3->dev.bus);
        piix3_update_irq_levels(piix3);
        for (pic_irq = 0; pic_irq < PIIX_NUM_PIC_IRQS; pic_irq++) {
            piix3_set_irq_pic(piix3, pic_irq);
        }
    }
}

static void piix3_write_config_xen(PCIDevice *dev,
                               uint32_t address, uint32_t val, int len)
{
    xen_piix_pci_write_config_client(address, val, len);
    piix3_write_config(dev, address, val, len);
}

static void piix3_reset(DeviceState *dev)
{
    PIIX3State *d = PIIX3(dev);
    uint8_t *pci_conf = d->dev.config;

    pci_conf[0x04] = 0x07; /* master, memory and I/O */
    pci_conf[0x05] = 0x00;
    pci_conf[0x06] = 0x00;
    pci_conf[0x07] = 0x02; /* PCI_status_devsel_medium */
    pci_conf[0x4c] = 0x4d;
    pci_conf[0x4e] = 0x03;
    pci_conf[0x4f] = 0x00;
    pci_conf[0x60] = 0x80;
    pci_conf[0x61] = 0x80;
    pci_conf[0x62] = 0x80;
    pci_conf[0x63] = 0x80;
    pci_conf[0x69] = 0x02;
    pci_conf[0x70] = 0x80;
    pci_conf[0x76] = 0x0c;
    pci_conf[0x77] = 0x0c;
    pci_conf[0x78] = 0x02;
    pci_conf[0x79] = 0x00;
    pci_conf[0x80] = 0x00;
    pci_conf[0x82] = 0x00;
    pci_conf[0xa0] = 0x08;
    pci_conf[0xa2] = 0x00;
    pci_conf[0xa3] = 0x00;
    pci_conf[0xa4] = 0x00;
    pci_conf[0xa5] = 0x00;
    pci_conf[0xa6] = 0x00;
    pci_conf[0xa7] = 0x00;
    pci_conf[0xa8] = 0x0f;
    pci_conf[0xaa] = 0x00;
    pci_conf[0xab] = 0x00;
    pci_conf[0xac] = 0x00;
    pci_conf[0xae] = 0x00;

    d->pic_levels = 0;
    d->rcr = 0;
}

static int piix3_post_load(void *opaque, int version_id)
{
    PIIX3State *piix3 = opaque;
    piix3_update_irq_levels(piix3);
    return 0;
}

static void piix3_pre_save(void *opaque)
{
    int i;
    PIIX3State *piix3 = opaque;

    for (i = 0; i < ARRAY_SIZE(piix3->pci_irq_levels_vmstate); i++) {
        piix3->pci_irq_levels_vmstate[i] =
            pci_bus_get_irq_level(piix3->dev.bus, i);
    }
}

static bool piix3_rcr_needed(void *opaque)
{
    PIIX3State *piix3 = opaque;

    return (piix3->rcr != 0);
}

static const VMStateDescription vmstate_piix3_rcr = {
    .name = "PIIX3/rcr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField []) {
        VMSTATE_UINT8(rcr, PIIX3State),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_piix3 = {
    .name = TYPE_PIIX3,
    .version_id = 3,
    .minimum_version_id = 2,
    .minimum_version_id_old = 2,
    .post_load = piix3_post_load,
    .pre_save = piix3_pre_save,
    .fields      = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, PIIX3State),
        VMSTATE_INT32_ARRAY_V(pci_irq_levels_vmstate, PIIX3State,
                              PIIX_NUM_PIRQS, 3),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (VMStateSubsection[]) {
        {
            .vmsd = &vmstate_piix3_rcr,
            .needed = piix3_rcr_needed,
        },
        { 0 }
    }
};


static void rcr_write(void *opaque, hwaddr addr, uint64_t val, unsigned len)
{
    PIIX3State *d = opaque;

    if (val & 4) {
        qemu_system_reset_request();
        return;
    }
    d->rcr = val & 2; /* keep System Reset type only */
}

static uint64_t rcr_read(void *opaque, hwaddr addr, unsigned len)
{
    PIIX3State *d = opaque;

    return d->rcr;
}

static const MemoryRegionOps rcr_ops = {
    .read = rcr_read,
    .write = rcr_write,
    .endianness = DEVICE_LITTLE_ENDIAN
};

static int piix3_realize(PCIDevice *dev)
{
    PIIX3State *s = PIIX3(dev);
    qemu_irq pit_irq = NULL;
    qemu_irq rtc_irq;

    s->bus = isa_bus_new(DEVICE(s), pci_address_space_io(dev));

    /* Realize the RTC */
    qdev_set_parent_bus(DEVICE(s->rtc), BUS(s->bus));
    qdev_init_nofail(DEVICE(s->rtc));

    /* Realize HPET */
    if (s->hpet) {
        int i;

        /* We need to introduce a proper IRQ and Memory QOM infrastructure
         * so that the HPET isn't a sysbus device */
        qdev_set_parent_bus(s->hpet, sysbus_get_default());
        qdev_init_nofail(s->hpet);

        sysbus_mmio_map(SYS_BUS_DEVICE(s->hpet), 0, HPET_BASE);
        for (i = 0; i < GSI_NUM_PINS; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(s->hpet), i, s->pic[i]);
        }
        rtc_irq = qdev_get_gpio_in(s->hpet, HPET_LEGACY_RTC_INT);
        pit_irq = qdev_get_gpio_in(s->hpet, HPET_LEGACY_PIT_INT);
    } else {
        isa_init_irq(s->rtc, &rtc_irq, RTC_ISA_IRQ);
    }

    rtc_set_irq(s->rtc, rtc_irq);

    /* Realize the PIT */
    qdev_set_parent_bus(DEVICE(s->pit), BUS(s->bus));
    qdev_init_nofail(DEVICE(s->pit));

    if (!pit_irq) {
        pit_irq = isa_get_irq(s->pit, 0);
    }
    if (!kvm_irqchip_in_kernel()) {
        qdev_connect_gpio_out(DEVICE(s->pit), 0, pit_irq);
    }

    if (s->hpet) {
        qdev_connect_gpio_out(DEVICE(s->hpet), 0,
                              qdev_get_gpio_in(DEVICE(s->pit), 0));
    }

    /* Realize pcspk */
    qdev_set_parent_bus(DEVICE(s->pcspk), BUS(s->bus));
    qdev_init_nofail(DEVICE(s->pcspk));

    memory_region_init_io(&s->rcr_mem, &rcr_ops, s, "piix3-reset-control", 1);
    memory_region_add_subregion_overlap(pci_address_space_io(dev), RCR_IOPORT,
                                        &s->rcr_mem, 1);

    return 0;
}

static void piix3_initfn(Object *obj)
{
    PIIX3State *s = PIIX3(obj);

    /*
     * Check if an HPET shall be created.
     *
     * Without KVM_CAP_PIT_STATE2, we cannot switch off the in-kernel PIT
     * when the HPET wants to take over. Thus we have to disable the latter.
     */
    if (!no_hpet && (!kvm_irqchip_in_kernel() || kvm_has_pit_state2())) {
        s->hpet = DEVICE(object_new(TYPE_HPET));
        object_property_add_child(obj, "hpet", OBJECT(s->hpet), NULL);
    }

    s->rtc = ISA_DEVICE(object_new(TYPE_MC146818_RTC));
    qdev_prop_set_int32(DEVICE(s->rtc), "base_year", 2000);
    object_property_add_child(obj, "rtc", OBJECT(s->rtc), NULL);

    if (kvm_irqchip_in_kernel()) {
        s->pit = ISA_DEVICE(object_new(TYPE_KVM_PIT));
    } else {
        s->pit = ISA_DEVICE(object_new(TYPE_PIT));
    }
    object_property_add_child(obj, "pit", OBJECT(s->pit), NULL);
    qdev_prop_set_uint32(DEVICE(s->pit), "iobase", 0x40);

    s->pcspk = ISA_DEVICE(object_new(TYPE_PC_SPEAKER));
    qdev_prop_set_uint32(&s->pcspk->qdev, "iobase", 0x61);
    qdev_prop_set_ptr(&s->pcspk->qdev, "pit", s->pit);
}

static void piix3_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    dc->desc        = "ISA bridge";
    dc->vmsd        = &vmstate_piix3;
    dc->no_user     = 1;
    dc->reset       = piix3_reset;
    k->no_hotplug   = 1;
    k->init         = piix3_realize;
    k->config_write = piix3_write_config;
    k->vendor_id    = PCI_VENDOR_ID_INTEL;
    /* 82371SB PIIX3 PCI-to-ISA bridge (Step A1) */
    k->device_id    = PCI_DEVICE_ID_INTEL_82371SB_0;
    k->class_id     = PCI_CLASS_BRIDGE_ISA;
}

static const TypeInfo piix3_info = {
    .name          = TYPE_PIIX3,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PIIX3State),
    .instance_init = piix3_initfn,
    .class_init    = piix3_class_init,
};

static void piix3_xen_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->config_write = piix3_write_config_xen;
};

static const TypeInfo piix3_xen_info = {
    .name          = "PIIX3-xen",
    .parent        = "PIIX3",
    .instance_size = sizeof(PIIX3State),
    .class_init    = piix3_xen_class_init,
};

static void i440fx_pmc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->no_hotplug = 1;
    k->init = i440fx_pmc_initfn;
    k->config_write = i440fx_write_config;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_82441;
    k->revision = 0x02;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    dc->desc = "Host bridge";
    dc->no_user = 1;
    dc->vmsd = &vmstate_i440fx_pmc;
}

static const TypeInfo i440fx_pmc_info = {
    .name          = TYPE_I440FX_PMC_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(I440FXPMCState),
    .class_init    = i440fx_pmc_class_init,
};

static void i440fx_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = i440fx_realize;
    dc->fw_name = "pci";
    dc->no_user = 1;
}

static const TypeInfo i440fx_info = {
    .name          = TYPE_I440FX_DEVICE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(I440FXState),
    .instance_init = i440fx_initfn,
    .class_init    = i440fx_class_init,
};

static void i440fx_register_types(void)
{
    type_register_static(&i440fx_pmc_info);
    type_register_static(&piix3_info);
    type_register_static(&piix3_xen_info);
    type_register_static(&i440fx_info);
}

type_init(i440fx_register_types)
