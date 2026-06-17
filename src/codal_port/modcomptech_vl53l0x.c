/*
 * modcomptech_vl53l0x.c - micro:bit v2 (codal_port) backend + MicroPython module
 * for the shared VL53L0X driver (vl53l0x.c / vl53l0x.h).
 *
 * This file is the micro:bit-specific half:
 *   1. it implements the three port hooks (vl53l0x_port.h) against the
 *      micro:bit HAL, and
 *   2. it exposes the driver to Python as the `comptech_vl53l0x` module.
 *
 * The portable driver lives in vl53l0x.c and is shared verbatim with the ESP32
 * track. Add both vl53l0x.c and this file to SRC_C in src/codal_port/Makefile.
 *
 *   import comptech_vl53l0x as tof
 *   tof.init()
 *   while True:
 *       print(tof.read())   # mm, or raises OSError(ETIMEDOUT)
 *
 * Two intentional departures from the Arduino original, both for the classroom:
 *   - default read timeout is 500 ms (the original blocks forever)
 *   - read()/read_continuous() raise OSError(ETIMEDOUT) instead of returning 65535
 */

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "microbithal.h"

#include "vl53l0x.h"
#include "vl53l0x_port.h"

/* External I2C bus pins on the micro:bit v2 edge connector. */
#define PLAT_I2C_SCL    MICROBIT_HAL_PIN_P19
#define PLAT_I2C_SDA    MICROBIT_HAL_PIN_P20

/* ---- port hooks (the only code here that touches the micro:bit HAL) ---- */

int vl53l0x_i2c_write(uint8_t addr, const uint8_t *buf, size_t len, bool stop)
{
    return microbit_hal_i2c_writeto(addr, buf, len, stop);
}

int vl53l0x_i2c_read(uint8_t addr, uint8_t *buf, size_t len, bool stop)
{
    return microbit_hal_i2c_readfrom(addr, buf, len, stop);
}

uint32_t vl53l0x_millis(void)
{
    return mp_hal_ticks_ms();
}

/* ---- MicroPython glue ---- */

static mp_obj_t mod_init(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_io_2v8, ARG_freq };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_io_2v8, MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_freq,   MP_ARG_INT,  {.u_int = 400000} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    /* freq == 0 leaves the bus alone (use this if you set it up via microbit.i2c).
     * Otherwise (re)configure the shared external bus. This bypasses MicroPython's
     * pin-ownership tracking, which is fine when this sensor owns P19/P20. */
    if (args[ARG_freq].u_int > 0) {
        int ret = microbit_hal_i2c_init(PLAT_I2C_SCL, PLAT_I2C_SDA, args[ARG_freq].u_int);
        if (ret != 0)
            mp_raise_OSError(ret);
    }

    if (!vl53l0x_init(args[ARG_io_2v8].u_bool))
        mp_raise_OSError(MP_ENODEV); /* wrong/absent model id or calibration fail */

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_init_obj, 0, mod_init);

static mp_obj_t mod_read(void)
{
    uint16_t mm = vl53l0x_read_range_single_mm();
    if (vl53l0x_timeout_occurred())
        mp_raise_OSError(MP_ETIMEDOUT);
    return MP_OBJ_NEW_SMALL_INT(mm);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_read_obj, mod_read);

static mp_obj_t mod_start_continuous(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_period_ms };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_period_ms, MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    vl53l0x_start_continuous((uint32_t)args[ARG_period_ms].u_int);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_start_continuous_obj, 0, mod_start_continuous);

static mp_obj_t mod_stop_continuous(void)
{
    vl53l0x_stop_continuous();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_stop_continuous_obj, mod_stop_continuous);

static mp_obj_t mod_read_continuous(void)
{
    uint16_t mm = vl53l0x_read_range_continuous_mm();
    if (vl53l0x_timeout_occurred())
        mp_raise_OSError(MP_ETIMEDOUT);
    return MP_OBJ_NEW_SMALL_INT(mm);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_read_continuous_obj, mod_read_continuous);

static mp_obj_t mod_set_timeout(mp_obj_t ms_in)
{
    vl53l0x_set_timeout((uint16_t)mp_obj_get_int(ms_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_set_timeout_obj, mod_set_timeout);

static mp_obj_t mod_set_signal_rate_limit(mp_obj_t mcps_in)
{
    return mp_obj_new_bool(vl53l0x_set_signal_rate_limit(mp_obj_get_float(mcps_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_set_signal_rate_limit_obj, mod_set_signal_rate_limit);

static mp_obj_t mod_get_signal_rate_limit(void)
{
    return mp_obj_new_float(vl53l0x_get_signal_rate_limit());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_get_signal_rate_limit_obj, mod_get_signal_rate_limit);

static mp_obj_t mod_set_measurement_timing_budget(mp_obj_t us_in)
{
    return mp_obj_new_bool(vl53l0x_set_measurement_timing_budget((uint32_t)mp_obj_get_int(us_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_set_measurement_timing_budget_obj, mod_set_measurement_timing_budget);

static mp_obj_t mod_get_measurement_timing_budget(void)
{
    return mp_obj_new_int_from_uint(vl53l0x_get_measurement_timing_budget());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_get_measurement_timing_budget_obj, mod_get_measurement_timing_budget);

static mp_obj_t mod_set_address(mp_obj_t addr_in)
{
    vl53l0x_set_address((uint8_t)mp_obj_get_int(addr_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_set_address_obj, mod_set_address);

static const mp_rom_map_elem_t comptech_vl53l0x_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_comptech_vl53l0x) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mod_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_continuous), MP_ROM_PTR(&mod_start_continuous_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_continuous), MP_ROM_PTR(&mod_stop_continuous_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_continuous), MP_ROM_PTR(&mod_read_continuous_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_timeout), MP_ROM_PTR(&mod_set_timeout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_rate_limit), MP_ROM_PTR(&mod_set_signal_rate_limit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_rate_limit), MP_ROM_PTR(&mod_get_signal_rate_limit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_measurement_timing_budget), MP_ROM_PTR(&mod_set_measurement_timing_budget_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_measurement_timing_budget), MP_ROM_PTR(&mod_get_measurement_timing_budget_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_address), MP_ROM_PTR(&mod_set_address_obj) },
};
static MP_DEFINE_CONST_DICT(comptech_vl53l0x_globals, comptech_vl53l0x_globals_table);

const mp_obj_module_t comptech_vl53l0x_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&comptech_vl53l0x_globals,
};

MP_REGISTER_MODULE(MP_QSTR_comptech_vl53l0x, comptech_vl53l0x_module);
