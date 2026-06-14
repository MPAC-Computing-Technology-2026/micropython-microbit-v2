// comptech_wifi: MicroPython interface to the ESP8266 WiFi bridge over UARTE1.
//

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include "nrf.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "peripheral_alloc_wrap.h"

/*
 * wifi_uart.c  --  UARTE1 driver for IoT:bit ESP8266 bridge
 *
 */

#define WIFI_DEBUG (1)

#if defined(WIFI_DEBUG)
#define DEBUG(fmt, ...) mp_printf(&mp_plat_print, fmt, ##__VA_ARGS__)
#else
#define DEBUG(x)
#endif

#define PIN_TX      10u
#define PIN_RX      12u
#define BAUD_115200 0x01D7E000UL

/* ------------------------------------------------------------------ */
/* Ring buffer                                                         */
/* ------------------------------------------------------------------ */

#define RXBUF_SIZE  2048u        /* must be a power of 2 */
#define RXBUF_MASK  (RXBUF_SIZE - 1u)

static uint8_t  ring[RXBUF_SIZE];
static volatile uint16_t rx_head = 0;   /* written by IRQ only */
static volatile uint16_t rx_tail = 0;   /* written by consumer only */
static uint8_t  dma_byte[1];            /* single-byte EasyDMA target; must be in RAM */

static int wifi_connected = 0;


#define QUEUE_MSG_SIZE (256)
#define QUEUE_SIZE (5)

struct wifi_uart_msg {
    size_t size;
    char buf[QUEUE_MSG_SIZE];
};
volatile static int wifi_uart_queue_rd = 0;
volatile static int wifi_uart_queue_wr = 0;
static struct wifi_uart_msg wifi_uart_queue[QUEUE_SIZE];

/* ------------------------------------------------------------------ */
/* IRQ handler                                                         */
/* ------------------------------------------------------------------ */

#if defined(WIFI_DEBUG)
static volatile uint32_t irq_count = 0;
#endif

static void uarte1_irq_handler(void *userdata)
{
#if defined(WIFI_DEBUG)
    irq_count++;
#endif
    (void)userdata;
    if (NRF_UARTE1->EVENTS_ENDRX)
    {
        NRF_UARTE1->EVENTS_ENDRX = 0;

        uint16_t next_head = (rx_head + 1u) & RXBUF_MASK;
        if (next_head != (rx_tail & RXBUF_MASK))
        {
            ring[rx_head] = dma_byte[0];
            rx_head = next_head;
        }

        NRF_UARTE1->RXD.PTR = (uint32_t)dma_byte;
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
    DEBUG("alloc=%p\n", p);

    NRF_UARTE1->ENABLE = 0;

    NRF_UARTE1->PSEL.TXD = PIN_TX;
    NRF_UARTE1->PSEL.RXD = PIN_RX;
    NRF_UARTE1->PSEL.RTS = 0xFFFFFFFFUL;
    NRF_UARTE1->PSEL.CTS = 0xFFFFFFFFUL;

    NRF_UARTE1->BAUDRATE = BAUD_115200;
    NRF_UARTE1->CONFIG = 0;

    NRF_UARTE1->ENABLE = UARTE_ENABLE_ENABLE_Enabled;

    wifi_set_irq(NRF_UARTE1, uarte1_irq_handler, NULL);
    NVIC_SetPriority(UARTE1_IRQn, 2);
    NVIC_EnableIRQ(UARTE1_IRQn);
    DEBUG("IRQ enabled\n");

    NRF_UARTE1->INTENSET = UARTE_INTENSET_ENDRX_Msk;

    NRF_UARTE1->RXD.PTR = (uint32_t)dma_byte;
    NRF_UARTE1->RXD.MAXCNT = 1;
    NRF_UARTE1->EVENTS_ENDRX = 0;
    NRF_UARTE1->TASKS_STARTRX = 1;
}

/* ------------------------------------------------------------------ */
/* TX -- blocking EasyDMA                                             */
/* ------------------------------------------------------------------ */

static uint8_t txbuf[512];

static void
wifi_write(const char *s)
{
    size_t len = 0;

    while (s[len] != '\0' && len < sizeof(txbuf)) {
        txbuf[len] = (uint8_t)s[len];
        len++;
    }

    NRF_UARTE1->TXD.PTR = (uint32_t)txbuf;
    NRF_UARTE1->TXD.MAXCNT = (uint32_t)len;
    NRF_UARTE1->EVENTS_ENDTX = 0;
    NRF_UARTE1->TASKS_STARTTX = 1;
    while (!NRF_UARTE1->EVENTS_ENDTX);
    NRF_UARTE1->TASKS_STOPTX = 1;
}

static void
wifi_write_n(const char *s, size_t len)
{
    size_t n = len < sizeof(txbuf) ? len : sizeof(txbuf);
    memcpy(txbuf, s, n);
    NRF_UARTE1->TXD.PTR = (uint32_t)txbuf;
    NRF_UARTE1->TXD.MAXCNT = (uint32_t)n;
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
 * Reads until '\n', c, timeout, or maxlen-1 bytes consumed.
 * Strips '\r'. Appends '\0'.
 * Returns number of bytes in buf (excluding '\0'), or -1 on timeout
 * with empty buffer.
 */
static int
wifi_readline(char *buf, size_t maxlen, char c, uint32_t timeout_ms)
{
    size_t i = 0;
    int ipd = 0;
    int ipd_len = 0;

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

        if (i == 5)
        {
            if (strncmp("+IPD,", buf, 5) == 0)
            {
                ipd = 1;
            }
        }
        else if (ipd == 1 && b == ':')
        {
            ipd_len = atoi(buf + 5);
            DEBUG("ipd_len = %d\n", ipd_len);
            if (ipd_len == 0)
                break;
        }
        else if (ipd == 1 && ipd_len > 0)
        {
            ipd_len--;
            if (ipd_len == 0)
                break;
        }

        if (b == '\n' || (c != 0 && b == c))
            break;
    }

    buf[i] = '\0';
    return (int)i;
}

static void
wifi_parse_ipd_cmd(char *buf, size_t len)
{
    int end;
    int msglen;
    int queue_wr;

    buf += 5;
    len -= 5;
    end = 0;
    while (end < len && buf[end] != ':')
        end++;
    if (buf[end] != ':' || end >= 5)
    {
        DEBUG("parse_ipd: malformed header\n");
        return;
    }
    buf[end] = '\0';
    end++;
    msglen = atoi(buf);
    if (msglen > QUEUE_MSG_SIZE)
    {
        DEBUG("parse_ipd: udp message larger than QUEUE_MSG_SIZE\n");
        return;
    }
    buf += end;
    len -= end;
    if (msglen > len)
    {
        DEBUG("parse_ipd: msglen larger than RX buffer\n");
        return;
    }

    queue_wr = (wifi_uart_queue_wr + 1) % QUEUE_SIZE;
    if (queue_wr == wifi_uart_queue_rd)
    {
        DEBUG("parse_ipd: msg queue full, dropping packet\n");
        return;
    }
    memcpy(wifi_uart_queue[queue_wr].buf, buf, msglen);
    wifi_uart_queue[queue_wr].size = msglen;
    wifi_uart_queue_wr = queue_wr;
}

static int
wifi_wait_for_command(const char *cmd, char *buf, size_t maxlen, char c, uint32_t timeout_ms)
{
    int ret = 0;
    int keep_running = 1;
    char tmp_buf[QUEUE_MSG_SIZE];
    int cmd_len = 0;

    if (cmd != NULL)
        cmd_len = strlen(cmd);

    while (keep_running != 0)
    {
        ret = wifi_readline(tmp_buf, QUEUE_MSG_SIZE, c, timeout_ms);
        DEBUG("UART received: %s\n", tmp_buf);
        if (ret > 0)
        {
            if (c != 0 && tmp_buf[ret - 1] == c)
            {
                memcpy(buf, tmp_buf, maxlen < (ret + 1) ? maxlen : ret + 1);
                return ret;
            }
            else if (cmd != NULL && strncmp(cmd, tmp_buf, cmd_len) == 0)
            {
                memcpy(buf, tmp_buf, maxlen < (ret + 1) ? maxlen : ret + 1);
                return ret;
            }
            else if (strncmp("+IPD,", tmp_buf, 5) == 0)
            {
                wifi_parse_ipd_cmd(tmp_buf, ret + 1);
                if (cmd == NULL && c == 0)
                    return ret;
            }
            else if (strncmp("ERROR", tmp_buf, 5) == 0)
            {
                return -2;
            }
            else if (strncmp("WIFI DISCONNECT", tmp_buf, 15) == 0)
            {
                wifi_connected = 0;
                wifi_uart_queue_rd = 0;
                wifi_uart_queue_wr = 0;
            }
        }
        if (ret < 0)
            keep_running = 0;
    }
    return ret;
}

static int
wifi_disconnect(void)
{
    char buf[8];
    wifi_connected = 0;
    wifi_uart_queue_rd = 0;
    wifi_uart_queue_wr = 0;
    wifi_writeline("AT+CWQAP");
    return wifi_wait_for_command("OK", buf, 8, 0, 1000);
}

static int
wifi_connect(const char *ssid, const char *passwd)
{
    int ret;
    char buf[256];

    wifi_writeline("AT+CWMODE=1");
    ret = wifi_wait_for_command("OK", buf, 256, 0, 1000);
    if (ret < 0)
        return -1;

    snprintf(buf, 256, "AT+CWJAP=\"%s\",\"%s\"", ssid, passwd);
    wifi_writeline(buf);
    ret = wifi_wait_for_command("OK", buf, 256, 0, 30000);
    if (ret < 0)
        return -1;
    wifi_connected = 1;

    return 0;
}

static int
wifi_set_server(const char *ip, int port)
{
    char buf[128];
    int ret;

    wifi_writeline("AT+CIPCLOSE");
    wifi_wait_for_command("OK", buf, 128, 0, 1000);

    snprintf(buf, 128, "AT+CIPSTART=\"UDP\",\"%s\",%d", ip, port);
    wifi_writeline(buf);
    ret = wifi_wait_for_command("OK", buf, 128, 0, 2000);
    return ret;
}

static int
wifi_send_udp(const char *data)
{
    char buf[512];
    int ret;
    size_t data_len = strlen(data);

    snprintf(buf, 512, "AT+CIPSEND=%d", data_len);
    DEBUG(buf);
    wifi_writeline(buf);

    ret = wifi_wait_for_command(NULL, buf, 512, '>', 2000);
    if (ret < 0)
        return ret;
    wifi_write_n(data, data_len);
    ret = wifi_wait_for_command("SEND OK", buf, 512, 0, 1000);
    return ret;
}

static int has_uart_init = 0;

// connect(ssid, passwd) -- connect to wifi AP
static mp_obj_t
comptech_wifi_connect(mp_obj_t ssid_in, mp_obj_t passwd_in)
{
    const char *ssid = mp_obj_str_get_str(ssid_in);
    const char *passwd = mp_obj_str_get_str(passwd_in);

    if (has_uart_init == 0)
    {
        wifi_uart_init();
        has_uart_init = 1;
    }
    wifi_disconnect();
    wifi_connect(ssid, passwd);
    if (wifi_connected != 0)
        return mp_const_true;
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_2(comptech_wifi_connect_obj, comptech_wifi_connect);

static mp_obj_t
comptech_wifi_connected(void)
{
    if (wifi_connected == 0)
        return mp_const_false;
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_0(comptech_wifi_connected_obj, comptech_wifi_connected);

static mp_obj_t
comptech_wifi_disconnect(void)
{
    if (has_uart_init == 0)
    {
        wifi_uart_init();
        has_uart_init = 1;
    }
    if (wifi_disconnect() < 0)
        return mp_const_false;
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_0(comptech_wifi_disconnect_obj, comptech_wifi_disconnect);

// set_server(ip, port) -- tell the bridge where to send UDP packets.
static mp_obj_t
comptech_wifi_set_server(mp_obj_t ip_in, mp_obj_t port_in)
{
    const char *ip = mp_obj_str_get_str(ip_in);
    int port = mp_obj_get_int(port_in);

    if (wifi_connected == 0)
        return mp_const_false;

    if (wifi_set_server(ip, port) < 0)
        return mp_const_false;

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_2(comptech_wifi_set_server_obj, comptech_wifi_set_server);

// send_udp(string) -- fire-and-forget a payload to the configured server.
static mp_obj_t
comptech_wifi_send_udp(mp_obj_t data_in)
{
    const char *data = mp_obj_str_get_str(data_in);

    if (wifi_connected == 0)
        return mp_const_false;

    if (wifi_send_udp(data) < 0)
        return mp_const_false;
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_1(comptech_wifi_send_udp_obj, comptech_wifi_send_udp);

static mp_obj_t
comptech_wifi_receive_udp(size_t n_args, const mp_obj_t *args)
{
    int rd;
    mp_obj_t str;
    uint32_t timeout_ms = 1;

    if (n_args > 0)
        timeout_ms = (uint32_t)mp_obj_get_int(args[0]);

    if (wifi_connected == 0)
        return mp_const_none;

    if (wifi_uart_queue_rd == wifi_uart_queue_wr)
        wifi_wait_for_command(NULL, NULL, 0, 0, timeout_ms);

    if (wifi_uart_queue_rd == wifi_uart_queue_wr)
        return mp_const_none;

    rd = (wifi_uart_queue_rd + 1) % QUEUE_SIZE;
    str = mp_obj_new_str(wifi_uart_queue[rd].buf, wifi_uart_queue[rd].size);
    wifi_uart_queue_rd = rd;

    return str;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(comptech_wifi_receive_udp_obj, 0, 1, comptech_wifi_receive_udp);

static const mp_rom_map_elem_t comptech_wifi_module_globals_table[] =
{
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_comptech_wifi) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&comptech_wifi_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&comptech_wifi_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_connected), MP_ROM_PTR(&comptech_wifi_connected_obj) },
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
