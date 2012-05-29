/*
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
#include <xdc/runtime/Diags.h>

#include <ti/ipc/MultiProc.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/ipc/rpmsg/VirtQueue.h>

#include <ti/grcm/RcmTypes.h>
#include <ti/grcm/RcmServer.h>

#include <ti/sdo/fc/global/FCSettings.h>
#include <ti/sdo/ce/global/CESettings.h>
#include <ti/dce/dce_priv.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    UInt16 hostId;

    /* Set up interprocessor notifications */
    System_printf("%s starting..\n", MultiProc_getName(MultiProc_self()));

    // Debug!
    DEBUG("more_debug");
    FCSettings_init();
    Diags_setMask(FCSETTINGS_MODNAME"+12345678LEXAIZFS");
    CESettings_init();
    Diags_setMask(CESETTINGS_MODNAME"+12345678LEXAIZFS");
    Diags_setMask("ti.ipc.rpmsg.MessageQCopy+12345678LEXAIZFS");
    Diags_setMask("ti.ipc.rpmsg.VirtQueue+12345678LEXAIZFS");
    Diags_setMask("ti.sysbios.knl.Semaphore+12345678LEXAIZFS");
    Diags_setMask("ti.sysbios.ipc.Semaphore+12345678LEXAIZFS");

    hostId = MultiProc_getId("HOST");
    MessageQCopy_init(hostId);

    BIOS_start();

    return 0;
}
