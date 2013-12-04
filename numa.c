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
#include "exec/memory.h"

#ifdef __linux__
#include <sys/syscall.h>
#ifndef MPOL_F_RELATIVE_NODES
#define MPOL_F_RELATIVE_NODES (1 << 14)
#define MPOL_F_STATIC_NODES   (1 << 15)
#endif
#endif

QemuOptsList qemu_numa_opts = {
    .name = "numa",
    .implied_opt_name = "type",
    .head = QTAILQ_HEAD_INITIALIZER(qemu_numa_opts.head),
    .desc = { { 0 } } /* validated with OptsVisitor */
};

static int numa_node_parse(NumaNodeOptions *opts)
{
    uint16_t nodenr;
    uint16List *cpus = NULL;

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

    return 0;
}

static int numa_mem_parse(NumaMemOptions *opts)
{
    uint16_t nodenr;
    uint64_t mem_size;
    uint16List *nodes;

    if (opts->has_nodeid) {
        nodenr = opts->nodeid;
    } else {
        nodenr = nb_numa_mem_nodes;
    }

    if (nodenr >= MAX_NODES) {
        fprintf(stderr, "qemu: Max number of NUMA nodes reached: %"
                PRIu16 "\n", nodenr);
        return -1;
    }

    if (opts->has_size) {
        mem_size = opts->size;
        numa_info[nodenr].node_mem = mem_size;
    }

    if (opts->has_policy) {
        numa_info[nodenr].policy = opts->policy;
    }

    if (opts->has_relative) {
        numa_info[nodenr].relative = opts->relative;
    }

    for (nodes = opts->host_nodes; nodes; nodes = nodes->next) {
        if (nodes->value > MAX_NODES) {
            fprintf(stderr, "qemu: node number %" PRIu16 " is bigger than %d\n",
                    nodes->value, MAX_NODES);
            continue;
        }
        bitmap_set(numa_info[nodenr].host_mem, nodes->value, 1);
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
    case NUMA_OPTIONS_KIND_MEM:
        ret = numa_mem_parse(object->mem);
        if (ret) {
            goto error;
        }
        nb_numa_mem_nodes++;
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
    if (nb_numa_mem_nodes > nb_numa_nodes) {
        nb_numa_nodes = nb_numa_mem_nodes;
    }

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

#ifdef __linux__
static int node_parse_bind_mode(unsigned int nodeid)
{
    int bind_mode;

    switch (numa_info[nodeid].policy) {
    case NUMA_NODE_POLICY_DEFAULT:
    case NUMA_NODE_POLICY_PREFERRED:
    case NUMA_NODE_POLICY_MEMBIND:
    case NUMA_NODE_POLICY_INTERLEAVE:
        bind_mode = numa_info[nodeid].policy;
        break;
    default:
        bind_mode = NUMA_NODE_POLICY_DEFAULT;
        return bind_mode;
    }

    bind_mode |= numa_info[nodeid].relative ?
        MPOL_F_RELATIVE_NODES : MPOL_F_STATIC_NODES;

    return bind_mode;
}

static int node_set_mem_policy(void *ram_ptr, ram_addr_t length, int nodeid)
{
    int bind_mode = node_parse_bind_mode(nodeid);
    unsigned long *nodes = numa_info[nodeid].host_mem;

    /* This is a workaround for a long standing bug in Linux'
     * mbind implementation, which cuts off the last specified
     * node. To stay compatible should this bug be fixed, we
     * specify one more node and zero this one out.
     */
    unsigned long maxnode = find_last_bit(nodes, MAX_NODES);
    if (syscall(SYS_mbind, ram_ptr, length, bind_mode,
                nodes, maxnode + 2, 0)) {
            perror("mbind");
            return -1;
    }

    return 0;
}
#endif

int memory_region_set_mem_policy(MemoryRegion *mr,
                                 ram_addr_t start, ram_addr_t length,
                                 ram_addr_t offset)
{
#ifdef __linux__
    ram_addr_t len = 0;
    int i;
    for (i = 0; i < nb_numa_nodes; i++) {
        len += numa_info[i].node_mem;
        if (offset < len) {
            break;
        }
    }
    if (i == nb_numa_nodes) {
        return -1;
    }

    void *ptr = memory_region_get_ram_ptr(mr);
    for (; i < nb_numa_nodes; i++ ) {
        if (offset + length <= len) {
            if (node_set_mem_policy(ptr + start, length, i)) {
                return -1;
            }
            break;
        } else {
            ram_addr_t tmp_len = len - offset;
            offset += tmp_len;
            length -= tmp_len;
            if (node_set_mem_policy(ptr + start, tmp_len, i)) {
                return -1;
            }
            start += tmp_len;
        }

        len += numa_info[i].node_mem;
    }

    if (i == nb_numa_nodes) {
        return -1;
    }
#endif

    return 0;
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
