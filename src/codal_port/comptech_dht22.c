// comptech_wifi: MicroPython interface to the DHT22
//

#include "py/runtime.h"
#include "py/mphal.h"

static int
wait_for_level(int pin, int level, uint32_t timeout_us)
{
    uint32_t start = mp_hal_ticks_us();
    while (microbit_hal_pin_read(pin) != level)
    {
        if ((mp_hal_ticks_us() - start) > timeout_us)
            return -1;
    }
    return (int)(mp_hal_ticks_us() - start);
}

static int
dht22_read(int pin, float *temp, float *hum)
{
    uint8_t data[5] = {0};
    int i, raw_temp;
    microbit_hal_pin_set_pull(pin, MICROBIT_HAL_PIN_PULL_NONE);
    /* --- send start pulse --- */
    microbit_hal_pin_write(pin, 0);
    mp_hal_delay_ms(2);
    microbit_hal_pin_write(pin, 1);
    mp_hal_delay_us(40);

    /* --- sensor response --- */
    if (wait_for_level(pin, 0, 120) < 0) return -1;  /* high -> low */
    if (wait_for_level(pin, 1, 120) < 0) return -2;  /* low -> high */
    if (wait_for_level(pin, 0, 120) < 0) return -3;  /* high -> low (data start) */

    /* --- read 40 bits --- */
    for (i = 0; i < 40; i++) {
        int duration;

        if (wait_for_level(pin, 1, 120) < 0)
            return -4;  /* low -> high (bit start) */
        duration = wait_for_level(pin, 0, 120);       /* measure high pulse width */
        if (duration < 0)
            return -5;
        data[i / 8] <<= 1;
        if (duration > 40)                                /* >40us = 1, <40us = 0 */
            data[i / 8] |= 1;
    }

    /* --- checksum --- */
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
        return -6;

    /* --- decode --- */
    *hum  = ((data[0] << 8) | data[1]) / 10.0f;
    raw_temp = ((data[2] & 0x7F) << 8) | data[3];
    *temp = (data[2] & 0x80) ? -(raw_temp / 10.0f) : (raw_temp / 10.0f);

    return 0;
}

static mp_obj_t
comptech_dht22_read(mp_obj_t mp_pin)
{
    float temp, rh;
    int error;
    mp_int_t pin = mp_obj_get_int(mp_pin);

    uint32_t irq_state = MICROPY_BEGIN_ATOMIC_SECTION();
    error = dht22_read(pin, &temp, &rh);
    MICROPY_END_ATOMIC_SECTION(irq_state);
    if (error != 0)
    {
        mp_obj_t tuple[2] = {
            mp_const_none,
            mp_obj_new_int(error),
        };
        return mp_obj_new_tuple(2, tuple);
    }

    mp_obj_t tuple[2] = {
        mp_obj_new_float(temp),
        mp_obj_new_float(rh),
    };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(comptech_dht22_read_obj, comptech_dht22_read);

static const mp_rom_map_elem_t comptech_dht22_module_globals_table[] =
{
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_dht22) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&comptech_dht22_read_obj) },
};
static MP_DEFINE_CONST_DICT(comptech_dht22_module_globals, comptech_dht22_module_globals_table);

const mp_obj_module_t comptech_dht22_module =
{
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&comptech_dht22_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_dht22, comptech_dht22_module);
