/*
 * QEMU System Emulator
 *
 * Copyright (c) 2013 Fujitsu Ltd.
 * Author: Wanlong Gao <gaowanlong@cn.fujitsu.com>
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

#include "sysemu/sysemu.h"
#include "qapi-visit.h"
#include "qapi/opts-visitor.h"
#include "qapi/dealloc-visitor.h"
#include "qapi/qmp/qerror.h"

QemuOptsList qemu_numa_opts = {
    .name = "numa",
    .implied_opt_name = "type",
    .head = QTAILQ_HEAD_INITIALIZER(qemu_numa_opts.head),
    .desc = { { 0 } } /* validated with OptsVisitor */
};

static int have_memdevs = -1;

static int numa_node_parse(NumaNodeOptions *opts)
{
    uint16_t nodenr;
    uint16List *cpus = NULL;
    Error *local_err = NULL;

    if (opts->has_nodeid) {
        nodenr = opts->nodeid;
    } else {
        nodenr = nb_numa_nodes;
    }

    if (nodenr >= MAX_NODES) {
        fprintf(stderr, "qemu: Max number of NUMA nodes reached: %"
                PRIu16 "\n", nodenr);
        return -1;
    }

    for (cpus = opts->cpus; cpus; cpus = cpus->next) {
        if (cpus->value > MAX_CPUMASK_BITS) {
            fprintf(stderr, "qemu: cpu number %" PRIu16 " is bigger than %d",
                    cpus->value, MAX_CPUMASK_BITS);
            continue;
        }
        bitmap_set(numa_info[nodenr].node_cpu, cpus->value, 1);
    }

    if (opts->has_mem && opts->has_memdev) {
        fprintf(stderr, "qemu: cannot specify both mem= and memdev=\n");
        return -1;
    }

    if (have_memdevs == -1) {
        have_memdevs = opts->has_memdev;
    }
    if (opts->has_memdev != have_memdevs) {
        fprintf(stderr, "qemu: memdev option must be specified for either "
                "all or no nodes\n");
    }

    if (opts->has_mem) {
        int64_t mem_size;
        char *endptr;
        mem_size = strtosz(opts->mem, &endptr);
        if (mem_size < 0 || *endptr) {
            fprintf(stderr, "qemu: invalid numa mem size: %s\n", opts->mem);
            return -1;
        }
        numa_info[nodenr].node_mem = mem_size;
    }
    if (opts->has_memdev) {
        Object *o;
        o = object_resolve_path_type(opts->memdev, TYPE_MEMORY_BACKEND, NULL);
        if (!o) {
            error_setg(&local_err, "memdev=%s is ambiguous", opts->memdev);
            qerror_report_err(local_err);
            return -1;
        }

        object_ref(o);
        numa_info[nodenr].node_mem = object_property_get_int(o, "size", NULL);
        numa_info[nodenr].node_memdev = MEMORY_BACKEND(o);
    }
    return 0;
}

int numa_init_func(QemuOpts *opts, void *opaque)
{
    NumaOptions *object = NULL;
    Error *err = NULL;
    int ret = 0;

    {
        OptsVisitor *ov = opts_visitor_new(opts);
        visit_type_NumaOptions(opts_get_visitor(ov), &object, NULL, &err);
        opts_visitor_cleanup(ov);
    }

    if (error_is_set(&err)) {
        fprintf(stderr, "qemu: %s\n", error_get_pretty(err));
        error_free(err);
        ret = -1;
        goto error;
    }

    switch (object->kind) {
    case NUMA_OPTIONS_KIND_NODE:
        ret = numa_node_parse(object->node);
        if (ret) {
            goto error;
        }
        nb_numa_nodes++;
        break;
    default:
        fprintf(stderr, "qemu: Invalid NUMA options type.\n");
        ret = -1;
    }

error:
    if (object) {
        QapiDeallocVisitor *dv = qapi_dealloc_visitor_new();
        visit_type_NumaOptions(qapi_dealloc_get_visitor(dv),
                               &object, NULL, NULL);
        qapi_dealloc_visitor_cleanup(dv);
    }

    return ret;
}

void set_numa_nodes(void)
{
    if (nb_numa_nodes > 0) {
        int i;

        if (nb_numa_nodes > MAX_NODES) {
            nb_numa_nodes = MAX_NODES;
        }

        /* If no memory size if given for any node, assume the default case
         * and distribute the available memory equally across all nodes
         */
        for (i = 0; i < nb_numa_nodes; i++) {
            if (numa_info[i].node_mem != 0)
                break;
        }
        if (i == nb_numa_nodes) {
            uint64_t usedmem = 0;

            /* On Linux, the each node's border has to be 8MB aligned,
             * the final node gets the rest.
             */
            for (i = 0; i < nb_numa_nodes - 1; i++) {
                numa_info[i].node_mem = (ram_size / nb_numa_nodes) &
                                        ~((1 << 23UL) - 1);
                usedmem += numa_info[i].node_mem;
            }
            numa_info[i].node_mem = ram_size - usedmem;
        }

        uint64_t numa_total = 0;
        for (i = 0; i < nb_numa_nodes; i++) {
            numa_total += numa_info[i].node_mem;
        }
        if (numa_total != ram_size) {
            fprintf(stderr, "qemu: numa nodes total memory size "
                            "should equal to ram_size\n");
            exit(1);
        }

        for (i = 0; i < nb_numa_nodes; i++) {
            if (!bitmap_empty(numa_info[i].node_cpu, MAX_CPUMASK_BITS)) {
                break;
            }
        }
        /* assigning the VCPUs round-robin is easier to implement, guest OSes
         * must cope with this anyway, because there are BIOSes out there in
         * real machines which also use this scheme.
         */
        if (i == nb_numa_nodes) {
            for (i = 0; i < max_cpus; i++) {
                set_bit(i, numa_info[i % nb_numa_nodes].node_cpu);
            }
        }
    }
}

void set_numa_modes(void)
{
    CPUState *cpu;
    int i;

    CPU_FOREACH(cpu) {
        for (i = 0; i < nb_numa_nodes; i++) {
            if (test_bit(cpu->cpu_index, numa_info[i].node_cpu)) {
                cpu->numa_node = i;
            }
        }
    }
}

static void allocate_system_memory_nonnuma(MemoryRegion *mr, Object *owner,
                                           const char *name,
                                           QEMUMachineInitArgs *args)
{
    uint64_t ram_size = args->ram_size;

    memory_region_init_ram(mr, owner, name, ram_size);
    vmstate_register_ram_global(mr);
}

void memory_region_allocate_system_memory(MemoryRegion *mr, Object *owner,
                                          const char *name,
                                          QEMUMachineInitArgs *args)
{
    uint64_t ram_size = args->ram_size;
    uint64_t addr = 0;
    int i;

    if (nb_numa_nodes == 0 || !have_memdevs) {
        allocate_system_memory_nonnuma(mr, owner, name, args);
        return;
    }

    memory_region_init(mr, owner, name, ram_size);
    for (i = 0; i < nb_numa_nodes; i++) {
        Error *local_err = NULL;
        uint64_t size = numa_info[i].node_mem;
        HostMemoryBackend *backend = numa_info[i].node_memdev;
        MemoryRegion *seg = host_memory_backend_get_memory(backend, &local_err);
        if (local_err) {
            qerror_report_err(local_err);
            exit(1);
        }

        memory_region_add_subregion(mr, addr, seg);
        vmstate_register_ram_global(seg);
        addr += size;
    }
}
