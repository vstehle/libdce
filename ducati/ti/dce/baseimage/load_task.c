/*
 * Monitor load and trace any change.
 * Author: Vincent Stehlé <v-stehle@ti.com>, copied from ping_tasks.c
 *
 * Copyright (c) 2011, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/smp/Load.h>
#include <ti/sysbios/gates/GateAll.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "profile.h"

/*
 * Time to sleep between load reporting attempts, in ticks.
 * On TI platforms, 1 tick == 1 ms.
 */
#define SLEEP_TICKS 1000

/*
 * Load reporting "threshold". When the new load is within previous reported
 * load +- this value, we do not report it.
 */
#define THRESHOLD 1

/* Compute a load, in percent. We nicely handle the case where the load is
 * negative or superior to 100%, in which case we return 0. This happens when
 * the profiler is reset between two measurements (e.g. player exit &
 * relaunched). Also, we handle div by zero, just in case. */
static unsigned compute_load(unsigned long delta_time, unsigned long *prev_val, unsigned long val)
{
    unsigned load;
    long delta = (long)val - (long)*prev_val;

    if (delta < 0 || delta > delta_time || !delta_time)
        load = 0;

    else
        load = 100 * delta / delta_time;

    *prev_val = val;
    return load;
}

/* Compute IVA load from KPI profiler, in percent. TODO: Better integration
 * into the KPI profiler. No need to protect the load "reading" as we
 * atomically read a single value. */
static unsigned compute_iva_load(unsigned long delta_time, unsigned long *prev_ivahd_t_tot)
{
    extern unsigned long get_ivahd_t_tot(void);
    unsigned long ivahd_t_tot = get_ivahd_t_tot();
    return compute_load(delta_time, prev_ivahd_t_tot, ivahd_t_tot);
}

/* Compute core load from KPI profiler, in percent. TODO: Better integration
 * into the KPI profiler. We protect the load "reading" for sanity of the
 * value. */
static unsigned compute_core_load(unsigned core, unsigned long delta_time,
    unsigned long *prev_core_total, GateAll_Handle gate)
{
    extern unsigned long get_core_total(unsigned core);
    unsigned long core_total;
    IArg key;

    /* CRITICAL SECTION */
    key = GateAll_enter(gate);
    core_total = get_core_total(core);
    GateAll_leave(gate, key);

    return compute_load(delta_time, prev_core_total, core_total);
}

/* Trace a load if delta is above threshold. */
static void trace_if_above(unsigned load, unsigned *prev_load, const char *name)
{
    unsigned delta;

    /* Trace if changed and delta above threshold. */
    delta = abs((int)load - (int)*prev_load);

    if (delta > THRESHOLD) {
        System_printf("loadTask: %s load = %u%%\n", name, load);
        *prev_load = load;
    }
}

/* Monitor load and trace any change. */
static Void loadTaskFxn(UArg arg0, UArg arg1)
{
    UInt32 prev_bios_load = 0;
    unsigned prev_iva_load = 0, prev_core0_load = 0, prev_core1_load = 0;
    unsigned long prev_time = 0, prev_ivahd_t_tot = 0, prev_core0_total = 0,
        prev_core1_total = 0;
    GateAll_Handle gate;

    /* Suppress warnings. */
    (void)arg0;
    (void)arg1;

    /* Prepare our Gate. */
    gate = GateAll_create(NULL, NULL);

    System_printf(
        "loadTask: started\n"
        "  SLEEP_TICKS: %u\n"
        "  Load_hwiEnabled: %d\n"
        "  Load_swiEnabled: %d\n"
        "  Load_taskEnabled: %d\n"
        "  Load_updateInIdle: %d\n"
        "  Load_windowInMs: %u\n"
        ,
        SLEEP_TICKS,
        Load_hwiEnabled,
        Load_swiEnabled,
        Load_taskEnabled,
        Load_updateInIdle,
        Load_windowInMs
    );

    /* Infinite loop to trace loads. */
    for (;;) {
        extern unsigned long get_32k(void);
        UInt32 bios_load;
        unsigned iva_load, core0_load, core1_load;
        unsigned long time, delta_time;

        /* Get BIOS load and trace if delta above threshold. */
        bios_load = Load_getCPULoad();
        trace_if_above(bios_load, &prev_bios_load, "BIOS");

        /* Compute 32k delta only once. */
        time = get_32k();
        delta_time = time - prev_time;
        prev_time = time;

        /* Get IVA load, cores loads. "Gating" is done inside each core load
         * computation, and preemption can happen between them. */
        iva_load = compute_iva_load(delta_time, &prev_ivahd_t_tot);
        core0_load = compute_core_load(0, delta_time, &prev_core0_total, gate);
        core1_load = compute_core_load(1, delta_time, &prev_core1_total, gate);

        /* Trace if delta above threshold. */
        trace_if_above(iva_load, &prev_iva_load, "IVA");
        trace_if_above(core0_load, &prev_core0_load, "core0");
        trace_if_above(core1_load, &prev_core1_load, "core1");

        /* Delay. */
        Task_sleep(SLEEP_TICKS);
    }
}

void start_load_task(void)
{
    Task_Params params;

    /* Monitor load and trace any change. */
    Task_Params_init(&params);
    params.instance->name = "loadtsk";
    params.priority = 1;

    if(!Task_create(loadTaskFxn, &params, NULL))
        System_printf("Could not create load task!\n");
}
