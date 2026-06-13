// comptech_wifi: MicroPython interface to the ESP8266 WiFi bridge over UARTE1.
//

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "nrf.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "peripheral_alloc_wrap.h"

/*
 * wifi_uart.c  --  UARTE1 driver for IoT:bit ESP8266 bridge
 *
 * micro:bit v2 edge connector mapping:
 *   P8  = nRF P0.18 = TX (micro:bit -> ESP8266 RX)
 *   P12 = nRF P0.20 = RX (ESP8266 TX -> micro:bit)
 *
 */


#define PIN_TX      10u
#define PIN_RX      12u
#define BAUD_115200 0x01D7E000UL

/* ------------------------------------------------------------------ */
/* Ring buffer                                                         */
/* ------------------------------------------------------------------ */

#define RXBUF_SIZE  1024u        /* must be a power of 2 */
#define RXBUF_MASK  (RXBUF_SIZE - 1u)

static uint8_t  ring[RXBUF_SIZE];
static volatile uint16_t rx_head = 0;   /* written by IRQ only */
static volatile uint16_t rx_tail = 0;   /* written by consumer only */
static uint8_t  dma_byte[1];            /* single-byte EasyDMA target; must be in RAM */

/* ------------------------------------------------------------------ */
/* IRQ handler                                                         */
/* ------------------------------------------------------------------ */

static volatile uint32_t irq_count = 0;

static void uarte1_irq_handler(void *userdata)
{
    irq_count++;
    (void)userdata;
    if (NRF_UARTE1->EVENTS_ENDRX) {
        NRF_UARTE1->EVENTS_ENDRX = 0;

        uint16_t next_head = (rx_head + 1u) & RXBUF_MASK;
        if (next_head != (rx_tail & RXBUF_MASK)) {
            ring[rx_head] = dma_byte[0];
            rx_head = next_head;
        }

        NRF_UARTE1->RXD.PTR    = (uint32_t)dma_byte;
        NRF_UARTE1->RXD.MAXCNT = 1;
        NRF_UARTE1->TASKS_STARTRX = 1;
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

static void
wifi_uart_init(void)
{
    void *p;
    p = wifi_alloc_peripheral(NRF_UARTE1);
    mp_printf(&mp_plat_print, "alloc=%p\n", p);

    NRF_UARTE1->ENABLE = 0;

    NRF_UARTE1->PSEL.TXD = PIN_TX;
    NRF_UARTE1->PSEL.RXD = PIN_RX;
    NRF_UARTE1->PSEL.RTS = 0xFFFFFFFFUL;
    NRF_UARTE1->PSEL.CTS = 0xFFFFFFFFUL;

    NRF_UARTE1->BAUDRATE = BAUD_115200;
    NRF_UARTE1->CONFIG   = 0;

    NRF_UARTE1->ENABLE = UARTE_ENABLE_ENABLE_Enabled;

    wifi_set_irq(NRF_UARTE1, uarte1_irq_handler, NULL);
    NVIC_SetPriority(UARTE1_IRQn, 2);
    NVIC_EnableIRQ(UARTE1_IRQn);
    mp_printf(&mp_plat_print, "IRQ enabled\n");

    NRF_UARTE1->INTENSET  = UARTE_INTENSET_ENDRX_Msk;

    NRF_UARTE1->RXD.PTR    = (uint32_t)dma_byte;
    NRF_UARTE1->RXD.MAXCNT = 1;
    NRF_UARTE1->EVENTS_ENDRX = 0;
    NRF_UARTE1->TASKS_STARTRX = 1;
}

/* ------------------------------------------------------------------ */
/* TX -- blocking EasyDMA                                             */
/* ------------------------------------------------------------------ */

static void
wifi_write(const char *s)
{
    static uint8_t txbuf[128];
    size_t len = 0;

    while (s[len] != '\0' && len < sizeof(txbuf)) {
        txbuf[len] = (uint8_t)s[len];
        len++;
    }

    NRF_UARTE1->TXD.PTR    = (uint32_t)txbuf;
    NRF_UARTE1->TXD.MAXCNT = (uint32_t)len;
    NRF_UARTE1->EVENTS_ENDTX = 0;
    NRF_UARTE1->TASKS_STARTTX = 1;
    while (!NRF_UARTE1->EVENTS_ENDTX);
    NRF_UARTE1->TASKS_STOPTX = 1;
}

static void
wifi_writeline(const char *line)
{
    wifi_write(line);
    wifi_write("\r\n");
}

/* ------------------------------------------------------------------ */
/* RX -- drain ring buffer with per-byte timeout                      */
/* ------------------------------------------------------------------ */

/*
 * Reads until '\n', timeout, or maxlen-1 bytes consumed.
 * Strips '\r'. Appends '\0'.
 * Returns number of bytes in buf (excluding '\0'), or -1 on timeout
 * with empty buffer.
 */
int
wifi_readline(char *buf, size_t maxlen, char c, uint32_t timeout_ms)
{
    size_t i = 0;

    while (i < maxlen - 1) {
        uint32_t t = timeout_ms * 6400u;    /* ~6400 spins/ms at 64 MHz */
        while (rx_head == rx_tail) {
            if (t-- == 0) {
                buf[i] = '\0';
                return (i > 0) ? (int)i : -1;
            }
        }

        uint8_t b = ring[rx_tail];
        rx_tail = (rx_tail + 1u) & RXBUF_MASK;

        if (b == '\r')
            continue;

        buf[i++] = (char)b;

        if (b == '\n' || (c != 0 && b == c))
            break;
    }

    buf[i] = '\0';
    return (int)i;
}

// set_server(ip, port) -- tell the bridge where to send UDP packets.
static mp_obj_t
comptech_wifi_set_server(mp_obj_t ip_in, mp_obj_t port_in)
{
    static int has_uart_init = 0;
    const char *ip = mp_obj_str_get_str(ip_in);
    mp_int_t port = mp_obj_get_int(port_in);
    char buf[128];
    mp_printf(&mp_plat_print, "hello!\n");

    if (has_uart_init == 0)
    {
        wifi_uart_init();
        has_uart_init = 1;

        wifi_writeline("AT");
mp_hal_delay_ms(1000);
mp_printf(&mp_plat_print, "irq_count=%lu rx_head=%u rx_tail=%u\n", irq_count, rx_head, rx_tail);
        int n = wifi_readline(buf, 128, 0, 5000);
        mp_printf(&mp_plat_print, "readline returned %d\n", n);
        if (n > 0)
            mp_printf(&mp_plat_print, "%s", buf);

        while (wifi_readline(buf, 128, 0, 1) > 0) {
            mp_printf(&mp_plat_print, buf);
        }
        wifi_writeline("AT+CWMODE=1");
        while (wifi_readline(buf, 128, 0, 10000) > 0) {
            mp_printf(&mp_plat_print, "%s", buf);
            if (strstr(buf, "OK"))
                break;
        }
        wifi_writeline("AT+CWJAP=\"comptech-net\",\"Turin2026\"");
        while (wifi_readline(buf, 128, 0, 10000) > 0) {
            mp_printf(&mp_plat_print, "%s", buf);
            if (strstr(buf, "GOT IP"))
                break;
        }
    }
    snprintf(buf, 128, "AT+CIPSTART=\"UDP\",\"%s\",%d,4210,0", ip, port);
    wifi_writeline(buf);
    while (wifi_readline(buf, 128, 0, 100) > 0) {
        mp_printf(&mp_plat_print, buf);
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(comptech_wifi_set_server_obj, comptech_wifi_set_server);

// send_udp(string) -- fire-and-forget a payload to the configured server.
static mp_obj_t
comptech_wifi_send_udp(mp_obj_t data_in)
{
    char buf[128];
    const char *data = mp_obj_str_get_str(data_in);

    snprintf(buf, 128, "AT+CIPSEND=%d", strlen(data) + 2);
    wifi_writeline(buf);

    while (wifi_readline(buf, 128, '>', 2000) > 0)
    {
        mp_printf(&mp_plat_print, buf);
        if (buf[0] == '>')
        {
            wifi_writeline(data);
            return mp_obj_new_int(0);
        }
    }
    return mp_obj_new_int(-1);
}
static MP_DEFINE_CONST_FUN_OBJ_1(comptech_wifi_send_udp_obj, comptech_wifi_send_udp);

// receive_udp() -- return a received payload as a str, or None if nothing waiting.
static mp_obj_t
comptech_wifi_receive_udp(void)
{
    // TODO: drain one complete message from the UARTE1 RX buffer.
    // return mp_obj_new_str(buf, len);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(comptech_wifi_receive_udp_obj, comptech_wifi_receive_udp);

static const mp_rom_map_elem_t comptech_wifi_module_globals_table[] =
{
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_comptech_wifi) },
    { MP_ROM_QSTR(MP_QSTR_set_server), MP_ROM_PTR(&comptech_wifi_set_server_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_udp), MP_ROM_PTR(&comptech_wifi_send_udp_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive_udp), MP_ROM_PTR(&comptech_wifi_receive_udp_obj) },
};
static MP_DEFINE_CONST_DICT(comptech_wifi_module_globals, comptech_wifi_module_globals_table);

const mp_obj_module_t comptech_wifi_module =
{
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&comptech_wifi_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_comptech_wifi, comptech_wifi_module);
