/*
 * vl53l0x.c - pure-C VL53L0X driver. Platform-agnostic.
 *
 * Pure-C port of Pololu's VL53L0X Arduino library (MIT), based on ST's VL53L0X
 * API (STSW-IMG005), the API user manual (UM2039), and the datasheet. All I/O
 * goes through the three hooks in vl53l0x_port.h, so this file is identical on
 * every platform.
 *
 * Single instance: sensor state is file-scope.
 */

#include <string.h>

#include "vl53l0x.h"
#include "vl53l0x_port.h"

/* ======================================================================== */
/* Register map (from ST's vl53l0x_device.h, via the Pololu library)        */
/* ======================================================================== */

enum {
    SYSRANGE_START                              = 0x00,

    SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

    SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

    GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

    SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

    RESULT_INTERRUPT_STATUS                     = 0x13,
    RESULT_RANGE_STATUS                         = 0x14,

    I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

    MSRC_CONFIG_CONTROL                         = 0x60,

    PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,

    FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

    PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,

    FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,

    MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

    IDENTIFICATION_MODEL_ID                     = 0xC0,

    OSC_CALIBRATE_VAL                           = 0xF8,

    GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,

    GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,

    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

    ALGO_PHASECAL_LIM                           = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

enum vcsel_period_type {
    VCSEL_PERIOD_PRE_RANGE,
    VCSEL_PERIOD_FINAL_RANGE,
};

/* Record current time to check an upcoming timeout against */
#define start_timeout()        (s_timeout_start_ms = (uint16_t)vl53l0x_millis())

/* Timeout enabled (nonzero) and expired? */
#define check_timeout_expired() \
    (s_io_timeout > 0 && ((uint16_t)((uint16_t)vl53l0x_millis() - s_timeout_start_ms) > s_io_timeout))

#define decode_vcsel_period(reg_val)      (((reg_val) + 1) << 1)
#define encode_vcsel_period(period_pclks) (((period_pclks) >> 1) - 1)

#define calc_macro_period(vcsel_period_pclks) \
    ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/* ======================================================================== */
/* File-scope sensor state (single instance)                                */
/* ======================================================================== */

static uint8_t  s_address                      = VL53L0X_ADDRESS_DEFAULT;
static uint16_t s_io_timeout                    = 500;   /* ms; 0 = no timeout */
static bool     s_did_timeout                   = false;
static uint16_t s_timeout_start_ms              = 0;
static int      s_last_status                   = 0;
static uint8_t  s_stop_variable                 = 0;
static uint32_t s_measurement_timing_budget_us  = 0;

struct seq_step_enables {
    bool tcc, msrc, dss, pre_range, final_range;
};

struct seq_step_timeouts {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
};

/* internal forward declarations */
static void     write_reg(uint8_t reg, uint8_t value);
static void     write_reg16(uint8_t reg, uint16_t value);
static void     write_reg32(uint8_t reg, uint32_t value);
static uint8_t  read_reg(uint8_t reg);
static uint16_t read_reg16(uint8_t reg);
static void     write_multi(uint8_t reg, const uint8_t *src, uint8_t count);
static void     read_multi(uint8_t reg, uint8_t *dst, uint8_t count);
static uint8_t  get_vcsel_pulse_period(enum vcsel_period_type type);
static bool     get_spad_info(uint8_t *count, bool *type_is_aperture);
static void     get_sequence_step_enables(struct seq_step_enables *enables);
static void     get_sequence_step_timeouts(const struct seq_step_enables *enables,
                                           struct seq_step_timeouts *timeouts);
static uint16_t decode_timeout(uint16_t reg_val);
static uint16_t encode_timeout(uint32_t timeout_mclks);
static uint32_t timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static bool     perform_single_ref_calibration(uint8_t vhv_init_byte);

/* ======================================================================== */
/* Low-level register access (built on the port hooks)                      */
/* ======================================================================== */

static void write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    s_last_status = vl53l0x_i2c_write(s_address, buf, 2, true);
}

static void write_reg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)value };
    s_last_status = vl53l0x_i2c_write(s_address, buf, 3, true);
}

static void write_reg32(uint8_t reg, uint32_t value)
{
    uint8_t buf[5] = {
        reg,
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)value,
    };
    s_last_status = vl53l0x_i2c_write(s_address, buf, 5, true);
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t value = 0;
    s_last_status = vl53l0x_i2c_write(s_address, &reg, 1, true);
    vl53l0x_i2c_read(s_address, &value, 1, true);
    return value;
}

static uint16_t read_reg16(uint8_t reg)
{
    uint8_t buf[2] = { 0, 0 };
    s_last_status = vl53l0x_i2c_write(s_address, &reg, 1, true);
    vl53l0x_i2c_read(s_address, buf, 2, true);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void write_multi(uint8_t reg, const uint8_t *src, uint8_t count)
{
    uint8_t buf[1 + 32];
    if (count > 32)
        count = 32;
    buf[0] = reg;
    memcpy(&buf[1], src, count);
    s_last_status = vl53l0x_i2c_write(s_address, buf, (size_t)count + 1, true);
}

static void read_multi(uint8_t reg, uint8_t *dst, uint8_t count)
{
    s_last_status = vl53l0x_i2c_write(s_address, &reg, 1, true);
    vl53l0x_i2c_read(s_address, dst, count, true);
}

/* ======================================================================== */
/* Public API                                                               */
/* ======================================================================== */

int vl53l0x_last_status(void)
{
    return s_last_status;
}

uint8_t vl53l0x_get_address(void)
{
    return s_address;
}

void vl53l0x_set_address(uint8_t new_addr)
{
    write_reg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    s_address = new_addr;
}

void vl53l0x_set_timeout(uint16_t ms)
{
    s_io_timeout = ms;
}

uint16_t vl53l0x_get_timeout(void)
{
    return s_io_timeout;
}

bool vl53l0x_timeout_occurred(void)
{
    bool tmp = s_did_timeout;
    s_did_timeout = false;
    return tmp;
}

bool vl53l0x_set_signal_rate_limit(float limit_mcps)
{
    if (limit_mcps < 0 || limit_mcps > 511.99f)
        return false;

    /* Q9.7 fixed point format */
    write_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                (uint16_t)(limit_mcps * (1 << 7)));
    return true;
}

float vl53l0x_get_signal_rate_limit(void)
{
    return (float)read_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool vl53l0x_set_measurement_timing_budget(uint32_t budget_us)
{
    struct seq_step_enables enables;
    struct seq_step_timeouts timeouts;

    const uint16_t StartOverhead      = 1910;
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(&enables);
    get_sequence_step_timeouts(&enables, &timeouts);

    if (enables.tcc)
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

    if (enables.dss)
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc)
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

    if (enables.pre_range)
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        /* No room for the final range timeout -> error; else apply remainder. */
        if (used_budget_us > budget_us)
            return false;

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        uint32_t final_range_timeout_mclks =
            timeout_us_to_mclks(final_range_timeout_us,
                                timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
            final_range_timeout_mclks += timeouts.pre_range_mclks;

        write_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    encode_timeout(final_range_timeout_mclks));

        s_measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t vl53l0x_get_measurement_timing_budget(void)
{
    struct seq_step_enables enables;
    struct seq_step_timeouts timeouts;

    const uint16_t StartOverhead      = 1910;
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(&enables);
    get_sequence_step_timeouts(&enables, &timeouts);

    if (enables.tcc)
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

    if (enables.dss)
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc)
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

    if (enables.pre_range)
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);

    if (enables.final_range)
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);

    s_measurement_timing_budget_us = budget_us;
    return budget_us;
}

bool vl53l0x_init(bool io_2v8)
{
    if (read_reg(IDENTIFICATION_MODEL_ID) != 0xEE)
        return false;

    /* VL53L0X_DataInit() begin */

    if (io_2v8) {
        write_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                  read_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    }

    /* "Set I2C standard mode" */
    write_reg(0x88, 0x00);

    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    s_stop_variable = read_reg(0x91);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);

    /* disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) checks */
    write_reg(MSRC_CONFIG_CONTROL, read_reg(MSRC_CONFIG_CONTROL) | 0x12);

    vl53l0x_set_signal_rate_limit(0.25f);

    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* VL53L0X_DataInit() end */

    /* VL53L0X_StaticInit() begin */

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!get_spad_info(&spad_count, &spad_type_is_aperture))
        return false;

    uint8_t ref_spad_map[6];
    read_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* -- VL53L0X_set_reference_spads() begin */

    write_reg(0xFF, 0x01);
    write_reg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_reg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_reg(0xFF, 0x00);
    write_reg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    write_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* -- VL53L0X_set_reference_spads() end */

    /* -- VL53L0X_load_tuning_settings() begin (DefaultTuningSettings) */

    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);

    write_reg(0xFF, 0x00);
    write_reg(0x09, 0x00);
    write_reg(0x10, 0x00);
    write_reg(0x11, 0x00);

    write_reg(0x24, 0x01);
    write_reg(0x25, 0xFF);
    write_reg(0x75, 0x00);

    write_reg(0xFF, 0x01);
    write_reg(0x4E, 0x2C);
    write_reg(0x48, 0x00);
    write_reg(0x30, 0x20);

    write_reg(0xFF, 0x00);
    write_reg(0x30, 0x09);
    write_reg(0x54, 0x00);
    write_reg(0x31, 0x04);
    write_reg(0x32, 0x03);
    write_reg(0x40, 0x83);
    write_reg(0x46, 0x25);
    write_reg(0x60, 0x00);
    write_reg(0x27, 0x00);
    write_reg(0x50, 0x06);
    write_reg(0x51, 0x00);
    write_reg(0x52, 0x96);
    write_reg(0x56, 0x08);
    write_reg(0x57, 0x30);
    write_reg(0x61, 0x00);
    write_reg(0x62, 0x00);
    write_reg(0x64, 0x00);
    write_reg(0x65, 0x00);
    write_reg(0x66, 0xA0);

    write_reg(0xFF, 0x01);
    write_reg(0x22, 0x32);
    write_reg(0x47, 0x14);
    write_reg(0x49, 0xFF);
    write_reg(0x4A, 0x00);

    write_reg(0xFF, 0x00);
    write_reg(0x7A, 0x0A);
    write_reg(0x7B, 0x00);
    write_reg(0x78, 0x21);

    write_reg(0xFF, 0x01);
    write_reg(0x23, 0x34);
    write_reg(0x42, 0x00);
    write_reg(0x44, 0xFF);
    write_reg(0x45, 0x26);
    write_reg(0x46, 0x05);
    write_reg(0x40, 0x40);
    write_reg(0x0E, 0x06);
    write_reg(0x20, 0x1A);
    write_reg(0x43, 0x40);

    write_reg(0xFF, 0x00);
    write_reg(0x34, 0x03);
    write_reg(0x35, 0x44);

    write_reg(0xFF, 0x01);
    write_reg(0x31, 0x04);
    write_reg(0x4B, 0x09);
    write_reg(0x4C, 0x05);
    write_reg(0x4D, 0x04);

    write_reg(0xFF, 0x00);
    write_reg(0x44, 0x00);
    write_reg(0x45, 0x20);
    write_reg(0x47, 0x08);
    write_reg(0x48, 0x28);
    write_reg(0x67, 0x00);
    write_reg(0x70, 0x04);
    write_reg(0x71, 0x01);
    write_reg(0x72, 0xFE);
    write_reg(0x76, 0x00);
    write_reg(0x77, 0x00);

    write_reg(0xFF, 0x01);
    write_reg(0x0D, 0x01);

    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x01);
    write_reg(0x01, 0xF8);

    write_reg(0xFF, 0x01);
    write_reg(0x8E, 0x01);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);

    /* -- VL53L0X_load_tuning_settings() end */

    /* "Set interrupt config to new sample ready" */
    write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_reg(GPIO_HV_MUX_ACTIVE_HIGH, read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    s_measurement_timing_budget_us = vl53l0x_get_measurement_timing_budget();

    /* "Disable MSRC and TCC by default" */
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* "Recalculate timing budget" */
    vl53l0x_set_measurement_timing_budget(s_measurement_timing_budget_us);

    /* VL53L0X_StaticInit() end */

    /* VL53L0X_PerformRefCalibration() begin */

    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!perform_single_ref_calibration(0x40))
        return false;

    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!perform_single_ref_calibration(0x00))
        return false;

    /* "restore the previous Sequence Config" */
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* VL53L0X_PerformRefCalibration() end */

    return true;
}

void vl53l0x_start_continuous(uint32_t period_ms)
{
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    write_reg(0x91, s_stop_variable);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);

    if (period_ms != 0) {
        uint16_t osc_calibrate_val = read_reg16(OSC_CALIBRATE_VAL);
        if (osc_calibrate_val != 0)
            period_ms *= osc_calibrate_val;

        write_reg32(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        write_reg(SYSRANGE_START, 0x04); /* timed */
    } else {
        write_reg(SYSRANGE_START, 0x02); /* back-to-back */
    }
}

void vl53l0x_stop_continuous(void)
{
    write_reg(SYSRANGE_START, 0x01); /* single-shot */

    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    write_reg(0x91, 0x00);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
}

uint16_t vl53l0x_read_range_continuous_mm(void)
{
    start_timeout();
    while ((read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (check_timeout_expired()) {
            s_did_timeout = true;
            return VL53L0X_OUT_OF_RANGE;
        }
    }

    /* Linearity Corrective Gain assumed 1000 (default); no fractional ranging */
    uint16_t range = read_reg16(RESULT_RANGE_STATUS + 10);

    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

uint16_t vl53l0x_read_range_single_mm(void)
{
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    write_reg(0x91, s_stop_variable);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);

    write_reg(SYSRANGE_START, 0x01);

    /* wait until start bit has been cleared */
    start_timeout();
    while (read_reg(SYSRANGE_START) & 0x01) {
        if (check_timeout_expired()) {
            s_did_timeout = true;
            return VL53L0X_OUT_OF_RANGE;
        }
    }

    return vl53l0x_read_range_continuous_mm();
}

/* ======================================================================== */
/* Internal helpers                                                         */
/* ======================================================================== */

static uint8_t get_vcsel_pulse_period(enum vcsel_period_type type)
{
    if (type == VCSEL_PERIOD_PRE_RANGE)
        return decode_vcsel_period(read_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    else if (type == VCSEL_PERIOD_FINAL_RANGE)
        return decode_vcsel_period(read_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    return 255;
}

static bool get_spad_info(uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;

    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);

    write_reg(0xFF, 0x06);
    write_reg(0x83, read_reg(0x83) | 0x04);
    write_reg(0xFF, 0x07);
    write_reg(0x81, 0x01);

    write_reg(0x80, 0x01);

    write_reg(0x94, 0x6b);
    write_reg(0x83, 0x00);
    start_timeout();
    while (read_reg(0x83) == 0x00) {
        if (check_timeout_expired())
            return false;
    }
    write_reg(0x83, 0x01);
    tmp = read_reg(0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    write_reg(0x81, 0x00);
    write_reg(0xFF, 0x06);
    write_reg(0x83, read_reg(0x83) & ~0x04);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x01);

    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);

    return true;
}

static void get_sequence_step_enables(struct seq_step_enables *enables)
{
    uint8_t sequence_config = read_reg(SYSTEM_SEQUENCE_CONFIG);

    enables->tcc         = (sequence_config >> 4) & 0x1;
    enables->dss         = (sequence_config >> 3) & 0x1;
    enables->msrc        = (sequence_config >> 2) & 0x1;
    enables->pre_range   = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

static void get_sequence_step_timeouts(const struct seq_step_enables *enables,
                                       struct seq_step_timeouts *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);

    timeouts->msrc_dss_tcc_mclks = read_reg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
        timeout_mclks_to_us(timeouts->msrc_dss_tcc_mclks,
                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks =
        decode_timeout(read_reg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
        timeout_mclks_to_us(timeouts->pre_range_mclks,
                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = get_vcsel_pulse_period(VCSEL_PERIOD_FINAL_RANGE);

    timeouts->final_range_mclks =
        decode_timeout(read_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;

    timeouts->final_range_us =
        timeout_mclks_to_us(timeouts->final_range_mclks,
                            timeouts->final_range_vcsel_period_pclks);
}

static uint16_t decode_timeout(uint16_t reg_val)
{
    /* format: "(LSByte * 2^MSByte) + 1" */
    return (uint16_t)((reg_val & 0x00FF) <<
           (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    /* format: "(LSByte * 2^MSByte) + 1" */
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calc_macro_period(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calc_macro_period(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static bool perform_single_ref_calibration(uint8_t vhv_init_byte)
{
    write_reg(SYSRANGE_START, 0x01 | vhv_init_byte);

    start_timeout();
    while ((read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (check_timeout_expired())
            return false;
    }

    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_reg(SYSRANGE_START, 0x00);

    return true;
}
