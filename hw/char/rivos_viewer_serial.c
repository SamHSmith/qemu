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

#include "qemu/osdep.h"
#include "hw/char/rivos_viewer_serial.h"
#include "ui/console.h"
#include "sysemu/dma.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "chardev/char-serial.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "qemu/error-report.h"
#include "trace.h"
#include "hw/qdev-properties.h"

#define UART_LCR_DLAB	0x80	/* Divisor latch access bit */

#define UART_IER_MSI	0x08	/* Enable Modem status interrupt */
#define UART_IER_RLSI	0x04	/* Enable receiver line status interrupt */
#define UART_IER_THRI	0x02	/* Enable Transmitter holding register int. */
#define UART_IER_RDI	0x01	/* Enable receiver data interrupt */

#define UART_IIR_NO_INT	0x01	/* No interrupts pending */
#define UART_IIR_ID	0x06	/* Mask for the interrupt ID */

#define UART_IIR_MSI	0x00	/* Modem status interrupt */
#define UART_IIR_THRI	0x02	/* Transmitter holding register empty */
#define UART_IIR_RDI	0x04	/* Receiver data interrupt */
#define UART_IIR_RLSI	0x06	/* Receiver line status interrupt */
#define UART_IIR_CTI    0x0C    /* Character Timeout Indication */

#define UART_IIR_FENF   0x80    /* Fifo enabled, but not functionning */
#define UART_IIR_FE     0xC0    /* Fifo enabled */

/*
 * These are the definitions for the Modem Control Register
 */
#define UART_MCR_LOOP	0x10	/* Enable loopback test mode */
#define UART_MCR_OUT2	0x08	/* Out2 complement */
#define UART_MCR_OUT1	0x04	/* Out1 complement */
#define UART_MCR_RTS	0x02	/* RTS complement */
#define UART_MCR_DTR	0x01	/* DTR complement */

/*
 * These are the definitions for the Modem Status Register
 */
#define UART_MSR_DCD	0x80	/* Data Carrier Detect */
#define UART_MSR_RI	0x40	/* Ring Indicator */
#define UART_MSR_DSR	0x20	/* Data Set Ready */
#define UART_MSR_CTS	0x10	/* Clear to Send */
#define UART_MSR_DDCD	0x08	/* Delta DCD */
#define UART_MSR_TERI	0x04	/* Trailing edge ring indicator */
#define UART_MSR_DDSR	0x02	/* Delta DSR */
#define UART_MSR_DCTS	0x01	/* Delta CTS */
#define UART_MSR_ANY_DELTA 0x0F	/* Any of the delta bits! */

#define UART_LSR_TEMT	0x40	/* Transmitter empty */
#define UART_LSR_THRE	0x20	/* Transmit-hold-register empty */
#define UART_LSR_BI	0x10	/* Break interrupt indicator */
#define UART_LSR_FE	0x08	/* Frame error indicator */
#define UART_LSR_PE	0x04	/* Parity error indicator */
#define UART_LSR_OE	0x02	/* Overrun error indicator */
#define UART_LSR_DR	0x01	/* Receiver data ready */
#define UART_LSR_INT_ANY 0x1E	/* Any of the lsr-interrupt-triggering status bits */

/* Interrupt trigger levels. The byte-counts are for 16550A - in newer UARTs the byte-count for each ITL is higher. */

#define UART_FCR_ITL_1      0x00 /* 1 byte ITL */
#define UART_FCR_ITL_2      0x40 /* 4 bytes ITL */
#define UART_FCR_ITL_3      0x80 /* 8 bytes ITL */
#define UART_FCR_ITL_4      0xC0 /* 14 bytes ITL */

#define UART_FCR_DMS        0x08    /* DMA Mode Select */
#define UART_FCR_XFR        0x04    /* XMIT Fifo Reset */
#define UART_FCR_RFR        0x02    /* RCVR Fifo Reset */
#define UART_FCR_FE         0x01    /* FIFO Enable */

#define MAX_XMIT_RETRY      4

static void viewer_receive1(void *opaque, const uint8_t *buf, int size);
static void viewer_xmit(ViewerState *s);

static inline void recv_fifo_put(ViewerState *s, uint8_t chr)
{
    /* Receive overruns do not overwrite FIFO contents. */
    if (!fifo8_is_full(&s->recv_fifo)) {
        fifo8_push(&s->recv_fifo, chr);
    } else {
        s->lsr |= UART_LSR_OE;
    }
}

static void viewer_update_irq(ViewerState *s)
{
    uint8_t tmp_iir = UART_IIR_NO_INT;

    if ((s->ier & UART_IER_RLSI) && (s->lsr & UART_LSR_INT_ANY)) {
        tmp_iir = UART_IIR_RLSI;
    } else if ((s->ier & UART_IER_RDI) && s->timeout_ipending) {
        /* Note that(s->ier & UART_IER_RDI) can mask this interrupt,
         * this is not in the specification but is observed on existing
         * hardware.  */
        tmp_iir = UART_IIR_CTI;
    } else if ((s->ier & UART_IER_RDI) && (s->lsr & UART_LSR_DR) &&
               (!(s->fcr & UART_FCR_FE) ||
                s->recv_fifo.num >= s->recv_fifo_itl)) {
        tmp_iir = UART_IIR_RDI;
    } else if ((s->ier & UART_IER_THRI) && s->thr_ipending) {
        tmp_iir = UART_IIR_THRI;
    } else if ((s->ier & UART_IER_MSI) && (s->msr & UART_MSR_ANY_DELTA)) {
        tmp_iir = UART_IIR_MSI;
    }

    s->iir = tmp_iir | (s->iir & 0xF0);

    if (tmp_iir != UART_IIR_NO_INT) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void viewer_update_parameters(ViewerState *s)
{
    float speed;
    int parity, data_bits, stop_bits, frame_size;
    QEMUSerialSetParams ssp;

    /* Start bit. */
    frame_size = 1;
    if (s->lcr & 0x08) {
        /* Parity bit. */
        frame_size++;
        if (s->lcr & 0x10)
            parity = 'E';
        else
            parity = 'O';
    } else {
            parity = 'N';
    }
    if (s->lcr & 0x04) {
        stop_bits = 2;
    } else {
        stop_bits = 1;
    }

    data_bits = (s->lcr & 0x03) + 5;
    frame_size += data_bits + stop_bits;
    /* Zero divisor should give about 3500 baud */
    speed = (s->divider == 0) ? 3500 : (float) s->baudbase / s->divider;
    ssp.speed = speed;
    ssp.parity = parity;
    ssp.data_bits = data_bits;
    ssp.stop_bits = stop_bits;
    s->char_transmit_time =  (NANOSECONDS_PER_SECOND / speed) * frame_size;
    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);
    trace_serial_update_parameters(speed, parity, data_bits, stop_bits);
}

static void viewer_update_msl(ViewerState *s)
{
    uint8_t omsr;
    int flags;

    timer_del(s->modem_status_poll);

    if (qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_GET_TIOCM,
                          &flags) == -ENOTSUP) {
        s->poll_msl = -1;
        return;
    }

    omsr = s->msr;

    s->msr = (flags & CHR_TIOCM_CTS) ? s->msr | UART_MSR_CTS : s->msr & ~UART_MSR_CTS;
    s->msr = (flags & CHR_TIOCM_DSR) ? s->msr | UART_MSR_DSR : s->msr & ~UART_MSR_DSR;
    s->msr = (flags & CHR_TIOCM_CAR) ? s->msr | UART_MSR_DCD : s->msr & ~UART_MSR_DCD;
    s->msr = (flags & CHR_TIOCM_RI) ? s->msr | UART_MSR_RI : s->msr & ~UART_MSR_RI;

    if (s->msr != omsr) {
         /* Set delta bits */
         s->msr = s->msr | ((s->msr >> 4) ^ (omsr >> 4));
         /* UART_MSR_TERI only if change was from 1 -> 0 */
         if ((s->msr & UART_MSR_TERI) && !(omsr & UART_MSR_RI))
             s->msr &= ~UART_MSR_TERI;
         viewer_update_irq(s);
    }

    /* The real 16550A apparently has a 250ns response latency to line status changes.
       We'll be lazy and poll only every 10ms, and only poll it at all if MSI interrupts are turned on */

    if (s->poll_msl) {
        timer_mod(s->modem_status_poll, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / 100);
    }
}

static gboolean viewer_watch_cb(GIOChannel *chan, GIOCondition cond,
                                void *opaque)
{
    ViewerState *s = opaque;
    s->watch_tag = 0;
    viewer_xmit(s);
    return FALSE;
}

static void viewer_xmit(ViewerState *s)
{
    do {
        assert(!(s->lsr & UART_LSR_TEMT));
        if (s->tsr_retry == 0) {
            assert(!(s->lsr & UART_LSR_THRE));

            if (s->fcr & UART_FCR_FE) {
                assert(!fifo8_is_empty(&s->xmit_fifo));
                s->tsr = fifo8_pop(&s->xmit_fifo);
                if (!s->xmit_fifo.num) {
                    s->lsr |= UART_LSR_THRE;
                }
            } else {
                s->tsr = s->thr;
                s->lsr |= UART_LSR_THRE;
            }
            if ((s->lsr & UART_LSR_THRE) && !s->thr_ipending) {
                s->thr_ipending = 1;
                viewer_update_irq(s);
            }
        }

        if (s->mcr & UART_MCR_LOOP) {
            /* in loopback mode, say that we just received a char */
            viewer_receive1(s, &s->tsr, 1);
        } else {
            int rc = qemu_chr_fe_write(&s->chr, &s->tsr, 1);

            if ((rc == 0 ||
                 (rc == -1 && errno == EAGAIN)) &&
                s->tsr_retry < MAX_XMIT_RETRY) {
                assert(s->watch_tag == 0);
                s->watch_tag =
                    qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                          viewer_watch_cb, s);
                if (s->watch_tag > 0) {
                    s->tsr_retry++;
                    return;
                }
            }
        }
        s->tsr_retry = 0;

        /* Transmit another byte if it is already available. It is only
           possible when FIFO is enabled and not empty. */
    } while (!(s->lsr & UART_LSR_THRE));

    s->last_xmit_ts = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->lsr |= UART_LSR_TEMT;
}

/* Setter for FCR.
   is_load flag means, that value is set while loading VM state
   and interrupt should not be invoked */
static void viewer_write_fcr(ViewerState *s, uint8_t val)
{
    /* Set fcr - val only has the bits that are supposed to "stick" */
    s->fcr = val;

    if (val & UART_FCR_FE) {
        s->iir |= UART_IIR_FE;
        /* Set recv_fifo trigger Level */
        switch (val & 0xC0) {
        case UART_FCR_ITL_1:
            s->recv_fifo_itl = 1;
            break;
        case UART_FCR_ITL_2:
            s->recv_fifo_itl = 4;
            break;
        case UART_FCR_ITL_3:
            s->recv_fifo_itl = 8;
            break;
        case UART_FCR_ITL_4:
            s->recv_fifo_itl = 14;
            break;
        }
    } else {
        s->iir &= ~UART_IIR_FE;
    }
}

static void viewer_update_tiocm(ViewerState *s)
{
    int flags;

    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_GET_TIOCM, &flags);

    flags &= ~(CHR_TIOCM_RTS | CHR_TIOCM_DTR);

    if (s->mcr & UART_MCR_RTS) {
        flags |= CHR_TIOCM_RTS;
    }
    if (s->mcr & UART_MCR_DTR) {
        flags |= CHR_TIOCM_DTR;
    }

    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_TIOCM, &flags);
}

static void viewer_ioport_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    ViewerState *s = opaque;
    ViewerState* viewer = s;
//    printf("super fancy uart write\n");

    *(((uint8_t*)viewer->recieve_data) + viewer->recieve_count) = (uint8_t)val;
    viewer->recieve_count += 1;

    if(viewer->recieve_count >= 8*2)
    {
        uint64_t data_count = viewer->recieve_data[0];
        uint64_t data_ptr = viewer->recieve_data[1];
        viewer->recieve_count = 0;

        float* buf = g_malloc(data_count);

        dma_memory_read(&address_space_memory, data_ptr,
                        buf,
                        data_count);
        for(uint64_t i = 0; i < viewer->framebuffer_size/3; i++)
        {
            *(viewer->framebuffer + i*3 + 0) = (uint8_t)(buf[i*4 + 0] * 255.0);
            *(viewer->framebuffer + i*3 + 1) = (uint8_t)(buf[i*4 + 1] * 255.0);
            *(viewer->framebuffer + i*3 + 2) = (uint8_t)(buf[i*4 + 2] * 255.0);
        }
        viewer->framebuffer_index = viewer->framebuffer_size;
        g_free(buf);
    }

    return;


    assert(size == 1 && addr < 8);
    trace_serial_write(addr, val);
    switch(addr) {
    default:
    case 0:
        if (s->lcr & UART_LCR_DLAB) {
            if (size == 1) {
                s->divider = (s->divider & 0xff00) | val;
            } else {
                s->divider = val;
            }
            viewer_update_parameters(s);
        } else {
            s->thr = (uint8_t) val;
            if(s->fcr & UART_FCR_FE) {
                /* xmit overruns overwrite data, so make space if needed */
                if (fifo8_is_full(&s->xmit_fifo)) {
                    fifo8_pop(&s->xmit_fifo);
                }
                fifo8_push(&s->xmit_fifo, s->thr);
            }
            s->thr_ipending = 0;
            s->lsr &= ~UART_LSR_THRE;
            s->lsr &= ~UART_LSR_TEMT;
            viewer_update_irq(s);
            if (s->tsr_retry == 0) {
                viewer_xmit(s);
            }
        }
        break;
    case 1:
        if (s->lcr & UART_LCR_DLAB) {
            s->divider = (s->divider & 0x00ff) | (val << 8);
            viewer_update_parameters(s);
        } else {
            uint8_t changed = (s->ier ^ val) & 0x0f;
            s->ier = val & 0x0f;
            /* If the backend device is a real viewer port, turn polling of the modem
             * status lines on physical port on or off depending on UART_IER_MSI state.
             */
            if ((changed & UART_IER_MSI) && s->poll_msl >= 0) {
                if (s->ier & UART_IER_MSI) {
                     s->poll_msl = 1;
                     viewer_update_msl(s);
                } else {
                     timer_del(s->modem_status_poll);
                     s->poll_msl = 0;
                }
            }

            /* Turning on the THRE interrupt on IER can trigger the interrupt
             * if LSR.THRE=1, even if it had been masked before by reading IIR.
             * This is not in the datasheet, but Windows relies on it.  It is
             * unclear if THRE has to be resampled every time THRI becomes
             * 1, or only on the rising edge.  Bochs does the latter, and Windows
             * always toggles IER to all zeroes and back to all ones, so do the
             * same.
             *
             * If IER.THRI is zero, thr_ipending is not used.  Set it to zero
             * so that the thr_ipending subsection is not migrated.
             */
            if (changed & UART_IER_THRI) {
                if ((s->ier & UART_IER_THRI) && (s->lsr & UART_LSR_THRE)) {
                    s->thr_ipending = 1;
                } else {
                    s->thr_ipending = 0;
                }
            }

            if (changed) {
                viewer_update_irq(s);
            }
        }
        break;
    case 2:
        /* Did the enable/disable flag change? If so, make sure FIFOs get flushed */
        if ((val ^ s->fcr) & UART_FCR_FE) {
            val |= UART_FCR_XFR | UART_FCR_RFR;
        }

        /* FIFO clear */

        if (val & UART_FCR_RFR) {
            s->lsr &= ~(UART_LSR_DR | UART_LSR_BI);
            timer_del(s->fifo_timeout_timer);
            s->timeout_ipending = 0;
            fifo8_reset(&s->recv_fifo);
        }

        if (val & UART_FCR_XFR) {
            s->lsr |= UART_LSR_THRE;
            s->thr_ipending = 1;
            fifo8_reset(&s->xmit_fifo);
        }

        viewer_write_fcr(s, val & 0xC9);
        viewer_update_irq(s);
        break;
    case 3:
        {
            int break_enable;
            s->lcr = val;
            viewer_update_parameters(s);
            break_enable = (val >> 6) & 1;
            if (break_enable != s->last_break_enable) {
                s->last_break_enable = break_enable;
                qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK,
                                  &break_enable);
            }
        }
        break;
    case 4:
        {
            int old_mcr = s->mcr;
            s->mcr = val & 0x1f;
            if (val & UART_MCR_LOOP)
                break;

            if (s->poll_msl >= 0 && old_mcr != s->mcr) {
                viewer_update_tiocm(s);
                /* Update the modem status after a one-character-send wait-time, since there may be a response
                   from the device/computer at the other end of the viewer line */
                timer_mod(s->modem_status_poll, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time);
            }
        }
        break;
    case 5:
        break;
    case 6:
        break;
    case 7:
        s->scr = val;
        break;
    }
}

static uint64_t viewer_ioport_read(void *opaque, hwaddr addr, unsigned size)
{
    ViewerState *s = opaque;
    ViewerState* viewer = s;
    uint32_t ret;

    if(addr == 1)
    {
        return viewer->send_count > 0;
    }
    else if(addr == 0)
    {
        uint8_t ret = 0;
        if(viewer->send_count > 0) { viewer->send_count -= 1; }
        for(int64_t i = viewer->send_count; i >= 0; i--)
        {
            uint8_t temp = viewer->send_buffer[i];
            viewer->send_buffer[i] = ret;
            ret = temp;
        }
        return ret;
    }
    return 0;


    assert(size == 1 && addr < 8);
    switch(addr) {
    default:
    case 0:
        if (s->lcr & UART_LCR_DLAB) {
            ret = s->divider & 0xff;
        } else {
            if(s->fcr & UART_FCR_FE) {
                ret = fifo8_is_empty(&s->recv_fifo) ?
                            0 : fifo8_pop(&s->recv_fifo);
                if (s->recv_fifo.num == 0) {
                    s->lsr &= ~(UART_LSR_DR | UART_LSR_BI);
                } else {
                    timer_mod(s->fifo_timeout_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time * 4);
                }
                s->timeout_ipending = 0;
            } else {
                ret = s->rbr;
                s->lsr &= ~(UART_LSR_DR | UART_LSR_BI);
            }
            viewer_update_irq(s);
            if (!(s->mcr & UART_MCR_LOOP)) {
                /* in loopback mode, don't receive any data */
                qemu_chr_fe_accept_input(&s->chr);
            }
        }
        break;
    case 1:
        if (s->lcr & UART_LCR_DLAB) {
            ret = (s->divider >> 8) & 0xff;
        } else {
            ret = s->ier;
        }
        break;
    case 2:
        ret = s->iir;
        if ((ret & UART_IIR_ID) == UART_IIR_THRI) {
            s->thr_ipending = 0;
            viewer_update_irq(s);
        }
        break;
    case 3:
        ret = s->lcr;
        break;
    case 4:
        ret = s->mcr;
        break;
    case 5:
        ret = s->lsr;
        /* Clear break and overrun interrupts */
        if (s->lsr & (UART_LSR_BI|UART_LSR_OE)) {
            s->lsr &= ~(UART_LSR_BI|UART_LSR_OE);
            viewer_update_irq(s);
        }
        break;
    case 6:
        if (s->mcr & UART_MCR_LOOP) {
            /* in loopback, the modem output pins are connected to the
               inputs */
            ret = (s->mcr & 0x0c) << 4;
            ret |= (s->mcr & 0x02) << 3;
            ret |= (s->mcr & 0x01) << 5;
        } else {
            if (s->poll_msl >= 0)
                viewer_update_msl(s);
            ret = s->msr;
            /* Clear delta bits & msr int after read, if they were set */
            if (s->msr & UART_MSR_ANY_DELTA) {
                s->msr &= 0xF0;
                viewer_update_irq(s);
            }
        }
        break;
    case 7:
        ret = s->scr;
        break;
    }
    trace_serial_read(addr, ret);
    return ret;
}

static void viewer_display_update(void* dev)
{
    ViewerState* viewer = VIEWER(dev);
    if(!viewer->framebuffer || viewer->framebuffer_index >= viewer->framebuffer_size)
    {
        if(viewer->framebuffer)
        {
            if(viewer->old_framebuffer)
            {
                g_free(viewer->old_framebuffer);
            }
            viewer->old_framebuffer = viewer->framebuffer;
            dpy_gfx_replace_surface(viewer->con, viewer->surface);
            dpy_gfx_update_full(viewer->con);
        }
        int width = qemu_console_get_width(viewer->con, 1280);
        int height = qemu_console_get_height(viewer->con, 1024);
        uint8_t* framebuffer = g_malloc(width*height*3);
        DisplaySurface* surface =
            qemu_create_displaysurface_from(width, height, PIXMAN_r8g8b8, width * 3, framebuffer);

        viewer->framebuffer = framebuffer;
        viewer->framebuffer_index = 0;
        viewer->framebuffer_size = width*height*3;
        viewer->surface = surface;

        if(viewer->send_count + 1 + 4*2 < 4096)
        {
            viewer->send_buffer[viewer->send_count] = 1;
            viewer->send_count += 1;
            uint32_t* b = (uint32_t*)(viewer->send_buffer + viewer->send_count);
            viewer->send_count += 4*2;
            *b = (uint32_t)width;
            b++;
            *b = (uint32_t)height;
        }
    }
    else
    {
        printf("Frame dropped\n");
    }
}

static int viewer_can_receive(ViewerState *s)
{
    if(s->fcr & UART_FCR_FE) {
        if (s->recv_fifo.num < UART_FIFO_LENGTH) {
            /*
             * Advertise (fifo.itl - fifo.count) bytes when count < ITL, and 1
             * if above. If UART_FIFO_LENGTH - fifo.count is advertised the
             * effect will be to almost always fill the fifo completely before
             * the guest has a chance to respond, effectively overriding the ITL
             * that the guest has set.
             */
            return (s->recv_fifo.num <= s->recv_fifo_itl) ?
                        s->recv_fifo_itl - s->recv_fifo.num : 1;
        } else {
            return 0;
        }
    } else {
        return !(s->lsr & UART_LSR_DR);
    }
}

static void viewer_receive_break(ViewerState *s)
{
    s->rbr = 0;
    /* When the LSR_DR is set a null byte is pushed into the fifo */
    recv_fifo_put(s, '\0');
    s->lsr |= UART_LSR_BI | UART_LSR_DR;
    viewer_update_irq(s);
}

/* There's data in recv_fifo and s->rbr has not been read for 4 char transmit times */
static void fifo_timeout_int (void *opaque) {
    ViewerState *s = opaque;
    if (s->recv_fifo.num) {
        s->timeout_ipending = 1;
        viewer_update_irq(s);
    }
}

static int viewer_can_receive1(void *opaque)
{
    ViewerState *s = opaque;
    return viewer_can_receive(s);
}

static void viewer_receive1(void *opaque, const uint8_t *buf, int size)
{
    ViewerState *s = opaque;

    if (s->wakeup) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    }
    if(s->fcr & UART_FCR_FE) {
        int i;
        for (i = 0; i < size; i++) {
            recv_fifo_put(s, buf[i]);
        }
        s->lsr |= UART_LSR_DR;
        /* call the timeout receive callback in 4 char transmit time */
        timer_mod(s->fifo_timeout_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time * 4);
    } else {
        if (s->lsr & UART_LSR_DR)
            s->lsr |= UART_LSR_OE;
        s->rbr = buf[0];
        s->lsr |= UART_LSR_DR;
    }
    viewer_update_irq(s);
}

static void viewer_event(void *opaque, QEMUChrEvent event)
{
    ViewerState *s = opaque;
    if (event == CHR_EVENT_BREAK)
        viewer_receive_break(s);
}

static int viewer_pre_save(void *opaque)
{
    ViewerState *s = opaque;
    s->fcr_vmstate = s->fcr;

    return 0;
}

static int viewer_pre_load(void *opaque)
{
    ViewerState *s = opaque;
    s->thr_ipending = -1;
    s->poll_msl = -1;
    return 0;
}

static int viewer_post_load(void *opaque, int version_id)
{
    ViewerState *s = opaque;

    if (version_id < 3) {
        s->fcr_vmstate = 0;
    }
    if (s->thr_ipending == -1) {
        s->thr_ipending = ((s->iir & UART_IIR_ID) == UART_IIR_THRI);
    }

    if (s->tsr_retry > 0) {
        /* tsr_retry > 0 implies LSR.TEMT = 0 (transmitter not empty).  */
        if (s->lsr & UART_LSR_TEMT) {
            error_report("inconsistent state in viewer device "
                         "(tsr empty, tsr_retry=%d", s->tsr_retry);
            return -1;
        }

        if (s->tsr_retry > MAX_XMIT_RETRY) {
            s->tsr_retry = MAX_XMIT_RETRY;
        }

        assert(s->watch_tag == 0);
        s->watch_tag = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                             viewer_watch_cb, s);
    } else {
        /* tsr_retry == 0 implies LSR.TEMT = 1 (transmitter empty).  */
        if (!(s->lsr & UART_LSR_TEMT)) {
            error_report("inconsistent state in viewer device "
                         "(tsr not empty, tsr_retry=0");
            return -1;
        }
    }

    s->last_break_enable = (s->lcr >> 6) & 1;
    /* Initialize fcr via setter to perform essential side-effects */
    viewer_write_fcr(s, s->fcr_vmstate);
    viewer_update_parameters(s);
    return 0;
}

static bool viewer_thr_ipending_needed(void *opaque)
{
    ViewerState *s = opaque;

    if (s->ier & UART_IER_THRI) {
        bool expected_value = ((s->iir & UART_IIR_ID) == UART_IIR_THRI);
        return s->thr_ipending != expected_value;
    } else {
        /* LSR.THRE will be sampled again when the interrupt is
         * enabled.  thr_ipending is not used in this case, do
         * not migrate it.
         */
        return false;
    }
}

static const VMStateDescription vmstate_viewer_thr_ipending = {
    .name = "viewer/thr_ipending",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = viewer_thr_ipending_needed,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(thr_ipending, ViewerState),
        VMSTATE_END_OF_LIST()
    }
};

static bool viewer_tsr_needed(void *opaque)
{
    ViewerState *s = (ViewerState *)opaque;
    return s->tsr_retry != 0;
}

static const VMStateDescription vmstate_viewer_tsr = {
    .name = "viewer/tsr",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = viewer_tsr_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(tsr_retry, ViewerState),
        VMSTATE_UINT8(thr, ViewerState),
        VMSTATE_UINT8(tsr, ViewerState),
        VMSTATE_END_OF_LIST()
    }
};

static bool viewer_recv_fifo_needed(void *opaque)
{
    ViewerState *s = (ViewerState *)opaque;
    return !fifo8_is_empty(&s->recv_fifo);

}

static const VMStateDescription vmstate_viewer_recv_fifo = {
    .name = "viewer/recv_fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = viewer_recv_fifo_needed,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(recv_fifo, ViewerState, 1, vmstate_fifo8, Fifo8),
        VMSTATE_END_OF_LIST()
    }
};

static bool viewer_xmit_fifo_needed(void *opaque)
{
    ViewerState *s = (ViewerState *)opaque;
    return !fifo8_is_empty(&s->xmit_fifo);
}

static const VMStateDescription vmstate_viewer_xmit_fifo = {
    .name = "viewer/xmit_fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = viewer_xmit_fifo_needed,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(xmit_fifo, ViewerState, 1, vmstate_fifo8, Fifo8),
        VMSTATE_END_OF_LIST()
    }
};

static bool viewer_fifo_timeout_timer_needed(void *opaque)
{
    ViewerState *s = (ViewerState *)opaque;
    return timer_pending(s->fifo_timeout_timer);
}

static const VMStateDescription vmstate_viewer_fifo_timeout_timer = {
    .name = "viewer/fifo_timeout_timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = viewer_fifo_timeout_timer_needed,
    .fields = (VMStateField[]) {
        VMSTATE_TIMER_PTR(fifo_timeout_timer, ViewerState),
        VMSTATE_END_OF_LIST()
    }
};

static bool viewer_timeout_ipending_needed(void *opaque)
{
    ViewerState *s = (ViewerState *)opaque;
    return s->timeout_ipending != 0;
}

static const VMStateDescription vmstate_viewer_timeout_ipending = {
    .name = "viewer/timeout_ipending",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = viewer_timeout_ipending_needed,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(timeout_ipending, ViewerState),
        VMSTATE_END_OF_LIST()
    }
};

static bool viewer_poll_needed(void *opaque)
{
    ViewerState *s = (ViewerState *)opaque;
    return s->poll_msl >= 0;
}

static const VMStateDescription vmstate_viewer_poll = {
    .name = "viewer/poll",
    .version_id = 1,
    .needed = viewer_poll_needed,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(poll_msl, ViewerState),
        VMSTATE_TIMER_PTR(modem_status_poll, ViewerState),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_viewer = {
    .name = "viewer",
    .version_id = 3,
    .minimum_version_id = 2,
    .pre_save = viewer_pre_save,
    .pre_load = viewer_pre_load,
    .post_load = viewer_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16_V(divider, ViewerState, 2),
        VMSTATE_UINT8(rbr, ViewerState),
        VMSTATE_UINT8(ier, ViewerState),
        VMSTATE_UINT8(iir, ViewerState),
        VMSTATE_UINT8(lcr, ViewerState),
        VMSTATE_UINT8(mcr, ViewerState),
        VMSTATE_UINT8(lsr, ViewerState),
        VMSTATE_UINT8(msr, ViewerState),
        VMSTATE_UINT8(scr, ViewerState),
        VMSTATE_UINT8_V(fcr_vmstate, ViewerState, 3),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {
        &vmstate_viewer_thr_ipending,
        &vmstate_viewer_tsr,
        &vmstate_viewer_recv_fifo,
        &vmstate_viewer_xmit_fifo,
        &vmstate_viewer_fifo_timeout_timer,
        &vmstate_viewer_timeout_ipending,
        &vmstate_viewer_poll,
        NULL
    }
};

static void viewer_reset(void *opaque)
{
    ViewerState *s = opaque;

    if (s->watch_tag > 0) {
        g_source_remove(s->watch_tag);
        s->watch_tag = 0;
    }

    s->rbr = 0;
    s->ier = 0;
    s->iir = UART_IIR_NO_INT;
    s->lcr = 0;
    s->lsr = UART_LSR_TEMT | UART_LSR_THRE;
    s->msr = UART_MSR_DCD | UART_MSR_DSR | UART_MSR_CTS;
    /* Default to 9600 baud, 1 start bit, 8 data bits, 1 stop bit, no parity. */
    s->divider = 0x0C;
    s->mcr = UART_MCR_OUT2;
    s->scr = 0;
    s->tsr_retry = 0;
    s->char_transmit_time = (NANOSECONDS_PER_SECOND / 9600) * 10;
    s->poll_msl = 0;

    s->timeout_ipending = 0;
    timer_del(s->fifo_timeout_timer);
    timer_del(s->modem_status_poll);

    fifo8_reset(&s->recv_fifo);
    fifo8_reset(&s->xmit_fifo);

    s->last_xmit_ts = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->thr_ipending = 0;
    s->last_break_enable = 0;
    qemu_irq_lower(s->irq);

    viewer_update_msl(s);
    s->msr &= ~UART_MSR_ANY_DELTA;
}

static int viewer_be_change(void *opaque)
{
    ViewerState *s = opaque;

    qemu_chr_fe_set_handlers(&s->chr, viewer_can_receive1, viewer_receive1,
                             viewer_event, viewer_be_change, s, NULL, true);

    viewer_update_parameters(s);

    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK,
                      &s->last_break_enable);

    s->poll_msl = (s->ier & UART_IER_MSI) ? 1 : 0;
    viewer_update_msl(s);

    if (s->poll_msl >= 0 && !(s->mcr & UART_MCR_LOOP)) {
        viewer_update_tiocm(s);
    }

    if (s->watch_tag > 0) {
        g_source_remove(s->watch_tag);
        s->watch_tag = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                             viewer_watch_cb, s);
    }

    return 0;
}

static const GraphicHwOps wrapper_ops = {
    .gfx_update = viewer_display_update,
};

static void viewer_realize(DeviceState *dev, Error **errp)
{
    ViewerState *s = VIEWER(dev);

    s->con = graphic_console_init(dev, 0, &wrapper_ops, dev);
    s->framebuffer = 0;
    s->old_framebuffer = 0;
    s->framebuffer_size = 0;
    s->surface = 0;
    s->send_count = 0;
    s->recieve_count = 0;

    s->modem_status_poll = timer_new_ns(QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *) viewer_update_msl, s);

    s->fifo_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *) fifo_timeout_int, s);
    qemu_register_reset(viewer_reset, s);

    qemu_chr_fe_set_handlers(&s->chr, viewer_can_receive1, viewer_receive1,
                             viewer_event, viewer_be_change, s, NULL, true);
    fifo8_create(&s->recv_fifo, UART_FIFO_LENGTH);
    fifo8_create(&s->xmit_fifo, UART_FIFO_LENGTH);
    viewer_reset(s);
}

static void viewer_unrealize(DeviceState *dev)
{
    ViewerState *s = VIEWER(dev);

    qemu_chr_fe_deinit(&s->chr, false);

    timer_del(s->modem_status_poll);
    timer_free(s->modem_status_poll);

    timer_del(s->fifo_timeout_timer);
    timer_free(s->fifo_timeout_timer);

    fifo8_destroy(&s->recv_fifo);
    fifo8_destroy(&s->xmit_fifo);

    qemu_unregister_reset(viewer_reset, s);
}

/* Change the main reference oscillator frequency. */
void viewer_set_frequency(ViewerState *s, uint32_t frequency)
{
    s->baudbase = frequency;
    viewer_update_parameters(s);
}

const MemoryRegionOps viewer_io_ops = {
    .read = viewer_ioport_read,
    .write = viewer_ioport_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static Property viewer_properties[] = {
    DEFINE_PROP_CHR("chardev", ViewerState, chr),
    DEFINE_PROP_UINT32("baudbase", ViewerState, baudbase, 115200),
    DEFINE_PROP_BOOL("wakeup", ViewerState, wakeup, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void viewer_class_init(ObjectClass *klass, void* data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    /* internal device for viewerio/viewermm, not user-creatable */
    dc->user_creatable = false;
    dc->realize = viewer_realize;
    dc->unrealize = viewer_unrealize;
    device_class_set_props(dc, viewer_properties);
}

static const TypeInfo viewer_info = {
    .name = TYPE_VIEWER,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(ViewerState),
    .class_init = viewer_class_init,
};

/* Memory mapped interface */
static uint64_t viewer_mm_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    ViewerMM *s = VIEWER_MM(opaque);
    return viewer_ioport_read(&s->viewer, addr >> s->regshift, 1);
}

static void viewer_mm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    ViewerMM *s = VIEWER_MM(opaque);
    value &= 255;
    viewer_ioport_write(&s->viewer, addr >> s->regshift, value, 1);
}

static const MemoryRegionOps viewer_mm_ops[3] = {
    [DEVICE_NATIVE_ENDIAN] = {
        .read = viewer_mm_read,
        .write = viewer_mm_write,
        .endianness = DEVICE_NATIVE_ENDIAN,
        .valid.max_access_size = 8,
        .impl.max_access_size = 8,
    },
    [DEVICE_LITTLE_ENDIAN] = {
        .read = viewer_mm_read,
        .write = viewer_mm_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .valid.max_access_size = 8,
        .impl.max_access_size = 8,
    },
    [DEVICE_BIG_ENDIAN] = {
        .read = viewer_mm_read,
        .write = viewer_mm_write,
        .endianness = DEVICE_BIG_ENDIAN,
        .valid.max_access_size = 8,
        .impl.max_access_size = 8,
    },
};

static void viewer_mm_realize(DeviceState *dev, Error **errp)
{
    ViewerMM *smm = VIEWER_MM(dev);
    ViewerState *s = &smm->viewer;

    if (!qdev_realize(DEVICE(s), NULL, errp)) {
        return;
    }

    memory_region_init_io(&s->io, OBJECT(dev),
                          &viewer_mm_ops[smm->endianness], smm, "viewer",
                          8 << smm->regshift);
    sysbus_init_mmio(SYS_BUS_DEVICE(smm), &s->io);
    sysbus_init_irq(SYS_BUS_DEVICE(smm), &smm->viewer.irq);
}

static const VMStateDescription vmstate_viewer_mm = {
    .name = "viewer",
    .version_id = 3,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(viewer, ViewerMM, 0, vmstate_viewer, ViewerState),
        VMSTATE_END_OF_LIST()
    }
};

ViewerMM *viewer_mm_init(MemoryRegion *address_space,
                         hwaddr base, int regshift,
                         qemu_irq irq, int baudbase,
                         Chardev *chr, enum device_endian end)
{
    ViewerMM *smm = VIEWER_MM(qdev_new(TYPE_VIEWER_MM));
    MemoryRegion *mr;

    qdev_prop_set_uint8(DEVICE(smm), "regshift", regshift);
    qdev_prop_set_uint32(DEVICE(smm), "baudbase", baudbase);
    qdev_prop_set_chr(DEVICE(smm), "chardev", chr);
    qdev_set_legacy_instance_id(DEVICE(smm), base, 2);
    qdev_prop_set_uint8(DEVICE(smm), "endianness", end);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(smm), &error_fatal);

    sysbus_connect_irq(SYS_BUS_DEVICE(smm), 0, irq);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(smm), 0);
    memory_region_add_subregion(address_space, base, mr);

    return smm;
}

static void viewer_mm_instance_init(Object *o)
{
    ViewerMM *smm = VIEWER_MM(o);

    object_initialize_child(o, "viewer", &smm->viewer, TYPE_VIEWER);

    qdev_alias_all_properties(DEVICE(&smm->viewer), o);
}

static Property viewer_mm_properties[] = {
    /*
     * Set the spacing between adjacent memory-mapped UART registers.
     * Each register will be at (1 << regshift) bytes after the
     * previous one.
     */
    DEFINE_PROP_UINT8("regshift", ViewerMM, regshift, 0),
    DEFINE_PROP_UINT8("endianness", ViewerMM, endianness, DEVICE_NATIVE_ENDIAN),
    DEFINE_PROP_END_OF_LIST(),
};

static void viewer_mm_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, viewer_mm_properties);
    dc->realize = viewer_mm_realize;
    dc->vmsd = &vmstate_viewer_mm;
}

static const TypeInfo viewer_mm_info = {
    .name = TYPE_VIEWER_MM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .class_init = viewer_mm_class_init,
    .instance_init = viewer_mm_instance_init,
    .instance_size = sizeof(ViewerMM),
};

static void viewer_register_types(void)
{
    type_register_static(&viewer_info);
    type_register_static(&viewer_mm_info);
}

type_init(viewer_register_types)
