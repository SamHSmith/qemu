/*
 * QEMU 16550A UART emulation
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
 * Copyright (c) 2008 Citrix Systems, Inc.
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

#ifndef HW_VIEWER_SERIAL_H
#define HW_VIEWER_SERIAL_H

#include "chardev/char-fe.h"
#include "exec/memory.h"
#include "qemu/fifo8.h"
#include "chardev/char.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "ui/console.h"
#include "ui/input.h"

#define UART_FIFO_LENGTH    16      /* 16550A Fifo Length */

struct ViewerState {
    DeviceState parent;

    QemuConsole* con;
    uint8_t* framebuffer;
    uint8_t* old_framebuffer;
    uint64_t framebuffer_index;
    uint64_t framebuffer_size;
    float* fb;

    uint64_t recieve_data[2];
    uint64_t recieve_count;

    uint64_t recieve_data2[2];
    uint64_t recieve_count2;

    uint8_t send_buffer[4096];
    uint64_t send_count;

    uint32_t mouse_x;
    uint32_t mouse_y;

    QemuInputHandlerState* input_handler;

    uint16_t divider;
    uint8_t rbr; /* receive register */
    uint8_t thr; /* transmit holding register */
    uint8_t tsr; /* transmit shift register */
    uint8_t ier;
    uint8_t iir; /* read only */
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr; /* read only */
    uint8_t msr; /* read only */
    uint8_t scr;
    uint8_t fcr;
    uint8_t fcr_vmstate; /* we can't write directly this value
                            it has side effects */
    /* NOTE: this hidden state is necessary for tx irq generation as
       it can be reset while reading iir */
    int thr_ipending;
    qemu_irq irq;
    CharBackend chr;
    int last_break_enable;
    uint32_t baudbase;
    uint32_t tsr_retry;
    guint watch_tag;
    bool wakeup;

    /* Time when the last byte was successfully sent out of the tsr */
    uint64_t last_xmit_ts;
    Fifo8 recv_fifo;
    Fifo8 xmit_fifo;
    /* Interrupt trigger level for recv_fifo */
    uint8_t recv_fifo_itl;

    QEMUTimer *fifo_timeout_timer;
    int timeout_ipending;           /* timeout interrupt pending state */

    uint64_t char_transmit_time;    /* time to transmit a char in ticks */
    int poll_msl;

    QEMUTimer *modem_status_poll;
    MemoryRegion io;
};
typedef struct ViewerState ViewerState;

struct ViewerMM {
    SysBusDevice parent;

    ViewerState viewer;

    uint8_t regshift;
    uint8_t endianness;
};

extern const VMStateDescription vmstate_viewer;
extern const MemoryRegionOps viewer_io_ops;

void viewer_set_frequency(ViewerState *s, uint32_t frequency);

#define TYPE_VIEWER "viewer"
OBJECT_DECLARE_SIMPLE_TYPE(ViewerState, VIEWER)

#define TYPE_VIEWER_MM "viewer-mm"
OBJECT_DECLARE_SIMPLE_TYPE(ViewerMM, VIEWER_MM)

ViewerMM *viewer_mm_init(MemoryRegion *address_space,
                         hwaddr base, int regshift,
                         qemu_irq irq, int baudbase,
                         Chardev *chr, enum device_endian end);

/* viewer-isa.c */

#define MAX_ISA_VIEWER_PORTS 4

#define TYPE_ISA_VIEWER "isa-viewer"
void viewer_hds_isa_init(ISABus *bus, int from, int to);

#endif
