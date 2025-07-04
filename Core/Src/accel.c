/*
 * accel.c
 *
 *  Created on: Jun 10, 2025
 *      Author: andrii-vukolov
 */
#include "accel.h"

/**
 * @example parameterSet(&acc1, &acc1.pmode, 2);
 * */
static accel_error_t
parameter_set(accel_t *paccel, parameter_t *par, uint8_t value)
{
    accel_error_t err  = ACCEL_OK;
    uint8_t       buf  = 0;
    uint8_t       mask = 0;

    par->value = value;

    for (uint8_t i = 0; i < par->len; i++)
        mask |= (1 << i);
    mask <<= par->bias;
    paccel->data_read(par->address, 1, &buf);
    buf   = buf & (~mask);
    value = value << par->bias;
    buf |= value;

    par->is_read = 1; // new value is in paccel-obj

    paccel->data_write(par->address, 1, &buf);
    return err;
}

static uint8_t parameter_get(accel_t *paccel, parameter_t *par)
{

    uint8_t value = 0;
    uint8_t mask  = 0;

    if (par->is_read == 0) //check if the parameter is been read
                           //and the value is in paccel-obj
    {
        paccel->data_read(par->address, 1, &value);
        for (uint8_t i = 0; i < par->len; i++)
            mask |= (1 << i);
        mask <<= par->bias;
        value        = (value & mask) >> par->bias;
        par->value   = value;
        par->is_read = 1;
    } else {
        value = par->value;
    }

    return value;
}

static float accel_data_calculate(accel_t *haccel, int16_t regdata)
{
    float   dt         = 0;
    float   sens[2][4] = { { 0.976, 1.952, 3.904, 7.808 },
                           { 0.244, 0.488, 0.976, 1.952 } };
    uint8_t fs_index   = parameter_get(haccel, &haccel->fs);
    uint8_t lp_index   = ((parameter_get(haccel, &haccel->lpmode) > 0) ? 1 : 0);
    dt                 = regdata * sens[fs_index][lp_index];
    return dt;
}

/**/
accel_error_t accel_power_mode_set(accel_t    *haccel,
                                   pmode_t     mode,
                                   lpmode_t    lpmode,
                                   odr_t       odr,
                                   low_noise_t ln)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->pmode, mode);
    err |= parameter_set(haccel, &haccel->lpmode, lpmode);
    err |= parameter_set(haccel, &haccel->odr, odr);
    err |= parameter_set(haccel, &haccel->low_noise, ln);

    return err;
}

/**/
accel_error_t
accel_filter_set(accel_t *haccel, uint8_t bw, uint8_t fds, full_scale_t fs)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &(haccel->bw_filt), bw);
    err = parameter_set(haccel, &(haccel->fds), fds);
    err = parameter_set(haccel, &(haccel->fs), fs);
    return err;
}

accel_error_t accel_int1_set(accel_t *haccel, interrupt_t int1)
{
    accel_error_t err = ACCEL_OK;

    if ((int1.ibit.single_tap_sleep_chg == 1) || (int1.ibit.ff_drdy_t == 1) ||
        (int1.ibit.wu_boot == 1) || (int1.ibit.tap_ovr == 1) ||
        (int1.ibit.sixd_sleep_state == 1)) {
        err = parameter_set(haccel, &(haccel->interrupts_en), ACCEL_ENABLE);
    }

    err = parameter_set(haccel, &(haccel->int1_drdy), int1.ibit.drdy);
    err = parameter_set(haccel, &(haccel->int1_fth), int1.ibit.fifio_thr);
    err = parameter_set(haccel, &(haccel->int1_diff5), int1.ibit.diff5);
    err = parameter_set(haccel, &(haccel->int1_tap), int1.ibit.tap_ovr);
    err = parameter_set(haccel, &(haccel->int1_ff), int1.ibit.ff_drdy_t);
    err = parameter_set(haccel, &(haccel->int1_wu), int1.ibit.wu_boot);
    err = parameter_set(
            haccel, &(haccel->int1_single_tap), int1.ibit.single_tap_sleep_chg);
    err = parameter_set(haccel, &(haccel->int1_6d), int1.ibit.sixd_sleep_state);

    return err;
}

accel_error_t accel_int2_set(accel_t *haccel, interrupt_t int2)
{
    accel_error_t err = ACCEL_OK;

    if ((int2.ibit.sixd_sleep_state == 1) ||
        (int2.ibit.single_tap_sleep_chg == 1)) {
        err = parameter_set(haccel, &(haccel->interrupts_en), ACCEL_ENABLE);
    }

    err = parameter_set(haccel, &(haccel->int2_drdy), int2.ibit.drdy);
    err = parameter_set(haccel, &(haccel->int2_fth), int2.ibit.fifio_thr);
    err = parameter_set(haccel, &(haccel->int2_diff5), int2.ibit.diff5);
    err = parameter_set(haccel, &(haccel->int2_ovr), int2.ibit.tap_ovr);
    err = parameter_set(haccel, &(haccel->int2_drdy_t), int2.ibit.ff_drdy_t);
    err = parameter_set(haccel, &(haccel->int2_boot), int2.ibit.wu_boot);
    err = parameter_set(
            haccel, &(haccel->int2_sleep_chg), int2.ibit.single_tap_sleep_chg);
    err = parameter_set(
            haccel, &(haccel->int2_sleep_state), int2.ibit.sixd_sleep_state);

    return err;
}

accel_error_t accel_int_disable(accel_t *haccel)
{
    accel_error_t err      = ACCEL_OK;
    uint8_t       data_buf = 0;

    err = haccel->data_write(ADD_CTRL4_INT1_PAD_CTRL, 1, &data_buf);
    err = haccel->data_write(ADD_CTRL5_INT2_PAD_CTRL, 1, &data_buf);
    err = haccel->data_read(ADD_ALL_INT_SRC, 1, &data_buf);

    err = parameter_set(haccel, &(haccel->interrupts_en), ACCEL_DISABLE);
    return err;
}

accel_error_t accel_int_mode_set(accel_t *haccel, drdy_pin_mode_t mode)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->lir, mode);

    return err;
}

accel_error_t
accel_free_fall_set(accel_t *haccel, ffthreshold_t thrashold, uint8_t duration)
{
    accel_error_t err   = ACCEL_OK;
    uint8_t       dur04 = (duration & 0x1F);
    uint8_t       dur5  = (duration & 0x20) >> 5;

    err = parameter_set(haccel, &haccel->ff_ths, thrashold);
    err |= parameter_set(haccel, &haccel->ff_dur_04, dur04);
    err |= parameter_set(haccel, &haccel->ff_dur_5, dur5);
    return err;
}

accel_error_t
accel_wake_up_set(accel_t *haccel, uint8_t thrashold, uint8_t duration)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->wake_dur, duration);
    err |= parameter_set(haccel, &haccel->wk_ths, thrashold);

    return err;
}

accel_error_t accel_offset_set(accel_t *haccel,
                               uint8_t  x_offset,
                               uint8_t  y_offset,
                               uint8_t  z_offset)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->x_ofs_usr, x_offset);
    err |= parameter_set(haccel, &haccel->y_ofs_usr, y_offset);
    err |= parameter_set(haccel, &haccel->z_ofs_usr, z_offset);

    return err;
}

accel_error_t accel_orientation_param_set(accel_t *haccel, sixd_thr_t threshold)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->ths_6d, threshold);
    return err;
}

accel_error_t accel_4d_set(accel_t *haccel, uint8_t en_dis)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->en_4d, en_dis);
    return err;
}

accel_error_t accel_double_tap_set(accel_t   *haccel,
                                   tap_dir_t *dir,
                                   tap_thr_t *tap_threshold,
                                   uint8_t    tap_window,
                                   uint8_t    priority,
                                   uint8_t    quiet,
                                   uint8_t    latency)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->tap_x_en, dir->crd.x_dir);
    err = parameter_set(haccel, &haccel->tap_y_en, dir->crd.y_dir);
    err = parameter_set(haccel, &haccel->tap_z_en, dir->crd.z_dir);
    err = parameter_set(haccel, &haccel->tap_thsx, tap_threshold->x_thr);
    err = parameter_set(haccel, &haccel->tap_thsy, tap_threshold->y_thr);
    err = parameter_set(haccel, &haccel->tap_thsz, tap_threshold->z_thr);
    err = parameter_set(haccel, &haccel->tap_prior, priority);
    err = parameter_set(haccel, &haccel->shock, tap_window);
    err = parameter_set(haccel, &haccel->quiet, quiet);
    err = parameter_set(haccel, &haccel->latency, latency);
    err = parameter_set(haccel, &haccel->single_double_tap, 1);

    return err;
}

accel_error_t accel_single_tap_set(accel_t   *haccel,
                                   tap_dir_t *dir,
                                   tap_thr_t *tap_threshold,
                                   uint8_t    tap_window,
                                   uint8_t    priority,
                                   uint8_t    quiet)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel,
                        &haccel->tap_x_en,
                        dir->crd.x_dir); //enable tap detection on X
    err = parameter_set(haccel,
                        &haccel->tap_y_en,
                        dir->crd.y_dir); //enable tap detection on Y
    err = parameter_set(haccel,
                        &haccel->tap_z_en,
                        dir->crd.z_dir); //enable tap detection on Z
    err = parameter_set(
            haccel, &haccel->tap_thsx, tap_threshold->x_thr); //threshold on X
    err = parameter_set(
            haccel, &haccel->tap_thsy, tap_threshold->y_thr); //threshold on Y
    err = parameter_set(
            haccel, &haccel->tap_thsz, tap_threshold->z_thr); //threshold on Z

    err = parameter_set(haccel, &haccel->tap_prior, priority);
    err = parameter_set(haccel, &haccel->shock, tap_window); //shock time window
    err = parameter_set(haccel, &haccel->quiet, quiet); // quiet time window
    err = parameter_set(haccel, &haccel->single_double_tap, 0);

    return err;
}

accel_error_t accel_activity_recognition_set(accel_t *haccel,
                                             uint8_t  sleep_on_off,
                                             uint8_t  wu_duration,
                                             uint8_t  inact_duration,
                                             uint8_t  threshold)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->sleep_on, sleep_on_off);
    err = parameter_set(haccel, &haccel->wake_dur, wu_duration);
    err = parameter_set(haccel, &haccel->sleep_dur, inact_duration);
    err = parameter_set(haccel, &haccel->wk_ths, threshold);
    err = parameter_set(haccel, &haccel->sleep_on, 1);

    return err;
}

accel_error_t
accel_fifo_set(accel_t *haccel, fifo_mode_t fifo_mode, uint8_t fifo_thr)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->fmode, fifo_mode);
    err = parameter_set(haccel, &haccel->fth, fifo_thr);

    return err;
}

accel_error_t accel_autoincrement_set(accel_t *haccel, uint8_t ainc)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->if_add_inc, ainc);

    return err;
}

accel_error_t accel_interface_set(accel_t *haccel, intarface_t ii)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->sim, (uint8_t)(ii & 0x01));
    err = parameter_set(
            haccel, &haccel->i2c_disable, (uint8_t)((ii & 0x02) >> 1));

    return err;
}

accel_error_t accel_bdu_set(accel_t *haccel, uint8_t bdu_bit)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->bdu, bdu_bit);

    return err;
}

accel_error_t accel_slp_mode_set(accel_t *haccel, uint8_t md_sel, uint8_t md_1)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->slp_mode_sel, md_sel);
    err = parameter_set(haccel, &haccel->slp_mode_1, md_1);

    return err;
}

accel_error_t accel_sixd_oriantation_detection(accel_t *haccel,
                                               uint8_t *orient,
                                               uint8_t *flag_orient)
{
    accel_error_t err       = ACCEL_OK;
    uint8_t       data_byte = 0;

    err          = haccel->data_read(ADD_SIXD_SRC, 1, &data_byte);
    *flag_orient = (data_byte & 0x40) >> 6;
    if (sizeof(orient) > 1) {
        for (uint8_t i = 0; i < 6; i++)
            orient[i] = (data_byte & (1 << i)) >> i;
    } else {
        *orient = data_byte;
    }

    return err;
}

accel_error_t motoin_detection(accel_t *haccel, uint8_t on_off)
{
    accel_error_t err = ACCEL_OK;

    err = parameter_set(haccel, &haccel->stationary, on_off);
    err = parameter_set(haccel, &haccel->sleep_on, on_off);

    return err;
}

fifo_state_t accel_fifo_state_get(accel_t *haccel, uint8_t *fifo_qty)
{
    fifo_state_t ret;
    uint8_t      buf;

    haccel->data_read(ADD_FIFO_SAMPLES, 1, &buf);
    if ((buf >= 0x80) && (buf < 0xC0))
        ret = FIFO_WATERMARK;
    if ((buf >= 0xC0) && (buf < 0xE0))
        ret = FIFO_FULL;
    if (buf >= 0xE0)
        ret = FIFO_OVRFULL;
    if (buf == 0)
        ret = FIFO_EMPTY;
    *fifo_qty = buf & 0x1F;

    return ret;
}

accel_error_t accel_data_get(accel_t *haccel, float *adata)
{
    accel_error_t err        = ACCEL_OK;
    uint8_t       reg_buf[6] = { 0 };
    int16_t       val[3]     = { 0 };
    resol_t       resolution = RES_14;
    uint8_t       buf_byte   = 0;
    uint8_t       i          = 0;
    //    fifo_state_t  is_data    = FIFO_EMPTY;
    //    uint8_t       data_qty   = 0;

    //    err = haccel->data_read(ADD_OUT_X_L, 6, reg_buf);

    reg_buf[0] = parameter_get(haccel, &haccel->out_x_l);
    reg_buf[1] = parameter_get(haccel, &haccel->out_x_h);
    reg_buf[2] = parameter_get(haccel, &haccel->out_y_l);
    reg_buf[3] = parameter_get(haccel, &haccel->out_y_h);
    reg_buf[4] = parameter_get(haccel, &haccel->out_z_l);
    reg_buf[5] = parameter_get(haccel, &haccel->out_z_h);

    haccel->out_x_l.is_read = 0;
    haccel->out_x_h.is_read = 0;
    haccel->out_y_l.is_read = 0;
    haccel->out_y_h.is_read = 0;
    haccel->out_z_l.is_read = 0;
    haccel->out_z_h.is_read = 0;

    /*resolution detection*/
    buf_byte = parameter_get(haccel, &haccel->pmode);
    if (buf_byte == 1)
        resolution = RES_14;
    else {
        buf_byte = parameter_get(haccel, &haccel->lpmode);
        if (buf_byte == 0)
            resolution = RES_12;
        else
            resolution = RES_14;
    }
    /*int16_t data forming*/
    for (i = 0; i < 3; i++) {
        val[i] = (int16_t)(reg_buf[i * 2 + 1] << 8) | reg_buf[2 * i];
        if (resolution == RES_14)
            val[i] >>= 2;
        else
            val[i] >>= 4;
    }
    /*float data calculation*/
    for (i = 0; i < 3; i++) {
        adata[i] = accel_data_calculate(haccel, val[i]);
    }

    return err;
}

uint8_t accel_data_ready_get(accel_t *haccel)
{
    uint8_t drdy = 0;

    drdy                 = parameter_get(haccel, &haccel->drdy);
    haccel->drdy.is_read = 0;
    //err  = haccel->data_read(ADD_CTRL7, 1, &data_buf);

    return drdy;
}

accel_error_t accel_event_status_get(accel_t *haccel, uint8_t *int_sources)
{
    accel_error_t err = ACCEL_OK;

    err = haccel->data_read(ADD_STATUS_DUP, 5, int_sources);

    return err;
}

uint8_t accel_id_get(accel_t *haccel)
{
    uint8_t ret = 0;
    ret         = parameter_get(haccel, &haccel->id);
    return ret;
}

accel_error_t accel_sixd_orientation_get(accel_t *haccel, uint8_t *orient_byte)
{
    accel_error_t err = ACCEL_OK;

    err = haccel->data_read(ADD_SIXD_SRC, 1, orient_byte);

    haccel->xl.value          = (*orient_byte & 0x01) >> 0;
    haccel->xh.value          = (*orient_byte & 0x02) >> 1;
    haccel->yl.value          = (*orient_byte & 0x04) >> 2;
    haccel->yh.value          = (*orient_byte & 0x08) >> 3;
    haccel->zl.value          = (*orient_byte & 0x10) >> 4;
    haccel->zh.value          = (*orient_byte & 0x20) >> 5;
    haccel->ia_6d_src.value   = (*orient_byte & 0x40) >> 6;
    haccel->xl.is_read        = 1;
    haccel->xh.is_read        = 1;
    haccel->yl.is_read        = 1;
    haccel->yh.is_read        = 1;
    haccel->zl.is_read        = 1;
    haccel->zh.is_read        = 1;
    haccel->ia_6d_src.is_read = 1;

    return err;
}

accel_error_t accel_tap_get(accel_t *haccel, uint8_t *tap_src_byte)
{
    accel_error_t err = ACCEL_OK;

    err                    = haccel->data_read(ADD_TAP_SRC, 1, tap_src_byte);
    haccel->z_tap.value    = (*tap_src_byte & 0x01) >> 0;
    haccel->y_tap.value    = (*tap_src_byte & 0x02) >> 1;
    haccel->x_tap.value    = (*tap_src_byte & 0x04) >> 2;
    haccel->tap_sign.value = (*tap_src_byte & 0x08) >> 3;
    haccel->double_tap_src.value = (*tap_src_byte & 0x10) >> 4;
    haccel->single_tap_src.value = (*tap_src_byte & 0x20) >> 5;
    haccel->tap_ia.value         = (*tap_src_byte & 0x40) >> 6;

    return err;
}

accel_error_t accel_init(accel_t *haccel)
{
    accel_error_t err = ACCEL_OK;
    interrupt_t   aint1, aint2;
    tap_dir_t     a1_tap_single_dir, a1_tap_double_dir;
    tap_thr_t     a1_tap_single_thr, a1_tap_double_thr;
    //==========================================================ACCEL SETTINGS

    haccel->id.address   = ADD_WHO_AM_I;
    haccel->id.bias      = 0;
    haccel->id.len       = 8;
    haccel->id.read_only = 1;
    haccel->id.is_read   = 0;
    haccel->id.value     = 0;

    haccel->pmode.address   = ADD_CTRL1;
    haccel->pmode.bias      = 2;
    haccel->pmode.len       = 2;
    haccel->pmode.read_only = 0;
    haccel->pmode.is_read   = 0;
    haccel->pmode.value     = 0;

    haccel->lpmode.address   = ADD_CTRL1;
    haccel->lpmode.bias      = 0;
    haccel->lpmode.len       = 2;
    haccel->lpmode.read_only = 0;
    haccel->lpmode.is_read   = 0;
    haccel->lpmode.value     = 0;

    haccel->odr.address   = ADD_CTRL1;
    haccel->odr.bias      = 4;
    haccel->odr.len       = 3;
    haccel->odr.read_only = 0;
    haccel->odr.is_read   = 0;
    haccel->odr.value     = 0;

    haccel->sim.address   = ADD_CTRL2;
    haccel->sim.bias      = 0;
    haccel->sim.len       = 1;
    haccel->sim.read_only = 0;
    haccel->sim.is_read   = 0;
    haccel->sim.value     = 0;

    haccel->i2c_disable.address   = ADD_CTRL2;
    haccel->i2c_disable.bias      = 1;
    haccel->i2c_disable.len       = 1;
    haccel->i2c_disable.read_only = 0;
    haccel->i2c_disable.is_read   = 0;
    haccel->i2c_disable.value     = 1;

    haccel->if_add_inc.address   = ADD_CTRL2;
    haccel->if_add_inc.bias      = 2;
    haccel->if_add_inc.len       = 1;
    haccel->if_add_inc.read_only = 0;
    haccel->if_add_inc.is_read   = 0;
    haccel->if_add_inc.value     = 0;

    haccel->bdu.address   = ADD_CTRL2;
    haccel->bdu.bias      = 3;
    haccel->bdu.len       = 1;
    haccel->bdu.read_only = 0;
    haccel->bdu.is_read   = 0;
    haccel->bdu.value     = 0;

    haccel->cs_pu_disc.address   = ADD_CTRL2;
    haccel->cs_pu_disc.bias      = 4;
    haccel->cs_pu_disc.len       = 1;
    haccel->cs_pu_disc.read_only = 0;
    haccel->cs_pu_disc.is_read   = 0;
    haccel->cs_pu_disc.value     = 0;

    haccel->soft_reset.address   = ADD_CTRL2;
    haccel->soft_reset.bias      = 6;
    haccel->soft_reset.len       = 1;
    haccel->soft_reset.read_only = 0;
    haccel->soft_reset.is_read   = 0;
    haccel->soft_reset.value     = 0;

    haccel->boot.address   = ADD_CTRL2;
    haccel->boot.bias      = 7;
    haccel->boot.len       = 1;
    haccel->boot.read_only = 0;
    haccel->boot.is_read   = 0;
    haccel->boot.value     = 0;

    haccel->slp_mode_1.address   = ADD_CTRL3;
    haccel->slp_mode_1.bias      = 0;
    haccel->slp_mode_1.len       = 1;
    haccel->slp_mode_1.read_only = 0;
    haccel->slp_mode_1.is_read   = 0;
    haccel->slp_mode_1.value     = 0;

    haccel->slp_mode_sel.address   = ADD_CTRL3;
    haccel->slp_mode_sel.bias      = 1;
    haccel->slp_mode_sel.len       = 1;
    haccel->slp_mode_sel.read_only = 0;
    haccel->slp_mode_sel.is_read   = 0;
    haccel->slp_mode_sel.value     = 0;

    haccel->hl_active.address   = ADD_CTRL3;
    haccel->hl_active.bias      = 3;
    haccel->hl_active.len       = 1;
    haccel->hl_active.read_only = 0;
    haccel->hl_active.is_read   = 0;
    haccel->hl_active.value     = 0;

    haccel->lir.address   = ADD_CTRL3;
    haccel->lir.bias      = 4;
    haccel->lir.len       = 1;
    haccel->lir.read_only = 0;
    haccel->lir.is_read   = 0;
    haccel->lir.value     = 0;

    haccel->pp_od.address   = ADD_CTRL3;
    haccel->pp_od.bias      = 5;
    haccel->pp_od.len       = 1;
    haccel->pp_od.read_only = 0;
    haccel->pp_od.is_read   = 0;
    haccel->pp_od.value     = 0;

    haccel->st1.address   = ADD_CTRL3;
    haccel->st1.bias      = 6;
    haccel->st1.len       = 1;
    haccel->st1.read_only = 0;
    haccel->st1.is_read   = 0;
    haccel->st1.value     = 0;

    haccel->st2.address   = ADD_CTRL3;
    haccel->st2.bias      = 7;
    haccel->st2.len       = 1;
    haccel->st2.read_only = 0;
    haccel->st2.is_read   = 0;
    haccel->st2.value     = 0;

    haccel->int1_drdy.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_drdy.bias      = 0;
    haccel->int1_drdy.len       = 1;
    haccel->int1_drdy.read_only = 0;
    haccel->int1_drdy.is_read   = 0;
    haccel->int1_drdy.value     = 0;

    haccel->int1_fth.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_fth.bias      = 1;
    haccel->int1_fth.len       = 1;
    haccel->int1_fth.read_only = 0;
    haccel->int1_fth.is_read   = 0;
    haccel->int1_fth.value     = 0;

    haccel->int1_diff5.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_diff5.bias      = 2;
    haccel->int1_diff5.len       = 1;
    haccel->int1_diff5.read_only = 0;
    haccel->int1_diff5.is_read   = 0;
    haccel->int1_diff5.value     = 0;

    haccel->int1_tap.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_tap.bias      = 3;
    haccel->int1_tap.len       = 1;
    haccel->int1_tap.read_only = 0;
    haccel->int1_tap.is_read   = 0;
    haccel->int1_tap.value     = 0;

    haccel->int1_ff.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_ff.bias      = 4;
    haccel->int1_ff.len       = 1;
    haccel->int1_ff.read_only = 0;
    haccel->int1_ff.is_read   = 0;
    haccel->int1_ff.value     = 0;

    haccel->int1_wu.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_wu.bias      = 5;
    haccel->int1_wu.len       = 1;
    haccel->int1_wu.read_only = 0;
    haccel->int1_wu.is_read   = 0;
    haccel->int1_wu.value     = 0;

    haccel->int1_single_tap.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_single_tap.bias      = 6;
    haccel->int1_single_tap.len       = 1;
    haccel->int1_single_tap.read_only = 0;
    haccel->int1_single_tap.is_read   = 0;
    haccel->int1_single_tap.value     = 0;

    haccel->int1_6d.address   = ADD_CTRL4_INT1_PAD_CTRL;
    haccel->int1_6d.bias      = 7;
    haccel->int1_6d.len       = 1;
    haccel->int1_6d.read_only = 0;
    haccel->int1_6d.is_read   = 0;
    haccel->int1_6d.value     = 0;

    haccel->int2_drdy.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_drdy.bias      = 0;
    haccel->int2_drdy.len       = 1;
    haccel->int2_drdy.read_only = 0;
    haccel->int2_drdy.is_read   = 0;
    haccel->int2_drdy.value     = 0;

    haccel->int2_fth.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_fth.bias      = 1;
    haccel->int2_fth.len       = 1;
    haccel->int2_fth.read_only = 0;
    haccel->int2_fth.is_read   = 0;
    haccel->int2_fth.value     = 0;

    haccel->int2_diff5.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_diff5.bias      = 2;
    haccel->int2_diff5.len       = 1;
    haccel->int2_diff5.read_only = 0;
    haccel->int2_diff5.is_read   = 0;
    haccel->int2_diff5.value     = 0;

    haccel->int2_ovr.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_ovr.bias      = 3;
    haccel->int2_ovr.len       = 1;
    haccel->int2_ovr.read_only = 0;
    haccel->int2_ovr.is_read   = 0;
    haccel->int2_ovr.value     = 0;

    haccel->int2_drdy_t.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_drdy_t.bias      = 4;
    haccel->int2_drdy_t.len       = 1;
    haccel->int2_drdy_t.read_only = 0;
    haccel->int2_drdy_t.is_read   = 0;
    haccel->int2_drdy_t.value     = 0;

    haccel->int2_boot.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_boot.bias      = 5;
    haccel->int2_boot.len       = 1;
    haccel->int2_boot.read_only = 0;
    haccel->int2_boot.is_read   = 0;
    haccel->int2_boot.value     = 0;

    haccel->int2_sleep_chg.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_sleep_chg.bias      = 6;
    haccel->int2_sleep_chg.len       = 1;
    haccel->int2_sleep_chg.read_only = 0;
    haccel->int2_sleep_chg.is_read   = 0;
    haccel->int2_sleep_chg.value     = 0;

    haccel->int2_sleep_state.address   = ADD_CTRL5_INT2_PAD_CTRL;
    haccel->int2_sleep_state.bias      = 7;
    haccel->int2_sleep_state.len       = 1;
    haccel->int2_sleep_state.read_only = 0;
    haccel->int2_sleep_state.is_read   = 0;
    haccel->int2_sleep_state.value     = 0;

    haccel->low_noise.address   = ADD_CTRL6;
    haccel->low_noise.bias      = 2;
    haccel->low_noise.len       = 1;
    haccel->low_noise.read_only = 0;
    haccel->low_noise.is_read   = 0;
    haccel->low_noise.value     = 0;

    haccel->fds.address   = ADD_CTRL6;
    haccel->fds.bias      = 3;
    haccel->fds.len       = 1;
    haccel->fds.read_only = 0;
    haccel->fds.is_read   = 0;
    haccel->fds.value     = 0;

    haccel->fs.address   = ADD_CTRL6;
    haccel->fs.bias      = 4;
    haccel->fs.len       = 2;
    haccel->fs.read_only = 0;
    haccel->fs.is_read   = 0;
    haccel->fs.value     = 0;

    haccel->bw_filt.address   = ADD_CTRL6;
    haccel->bw_filt.bias      = 6;
    haccel->bw_filt.len       = 2;
    haccel->bw_filt.read_only = 0;
    haccel->bw_filt.is_read   = 0;
    haccel->bw_filt.value     = 0;

    haccel->out_temp.address   = ADD_OUT_T;
    haccel->out_temp.bias      = 0;
    haccel->out_temp.len       = 8;
    haccel->out_temp.read_only = 1;
    haccel->out_temp.is_read   = 0;
    haccel->out_temp.value     = 0;

    haccel->drdy.address   = ADD_STATUS;
    haccel->drdy.bias      = 0;
    haccel->drdy.len       = 1;
    haccel->drdy.read_only = 1;
    haccel->drdy.is_read   = 0;
    haccel->drdy.value     = 0;

    haccel->ff_ia_st.address   = ADD_STATUS;
    haccel->ff_ia_st.bias      = 1;
    haccel->ff_ia_st.len       = 1;
    haccel->ff_ia_st.read_only = 1;
    haccel->ff_ia_st.is_read   = 0;
    haccel->ff_ia_st.value     = 0;

    haccel->ia_6d.address   = ADD_STATUS;
    haccel->ia_6d.bias      = 2;
    haccel->ia_6d.len       = 1;
    haccel->ia_6d.read_only = 1;
    haccel->ia_6d.is_read   = 0;
    haccel->ia_6d.value     = 0;

    haccel->single_tap.address   = ADD_STATUS;
    haccel->single_tap.bias      = 3;
    haccel->single_tap.len       = 1;
    haccel->single_tap.read_only = 1;
    haccel->single_tap.is_read   = 0;
    haccel->single_tap.value     = 0;

    haccel->double_tap.address   = ADD_STATUS;
    haccel->double_tap.bias      = 4;
    haccel->double_tap.len       = 1;
    haccel->double_tap.read_only = 1;
    haccel->double_tap.is_read   = 0;
    haccel->double_tap.value     = 0;

    haccel->sleep_state.address   = ADD_STATUS;
    haccel->sleep_state.bias      = 5;
    haccel->sleep_state.len       = 1;
    haccel->sleep_state.read_only = 1;
    haccel->sleep_state.is_read   = 0;
    haccel->sleep_state.value     = 0;

    haccel->wu_ia_st.address   = ADD_STATUS;
    haccel->wu_ia_st.bias      = 6;
    haccel->wu_ia_st.len       = 1;
    haccel->wu_ia_st.read_only = 1;
    haccel->wu_ia_st.is_read   = 0;
    haccel->wu_ia_st.value     = 0;

    haccel->fifo_ths.address   = ADD_STATUS;
    haccel->fifo_ths.bias      = 7;
    haccel->fifo_ths.len       = 1;
    haccel->fifo_ths.read_only = 1;
    haccel->fifo_ths.is_read   = 0;
    haccel->fifo_ths.value     = 0;

    haccel->out_x_l.address   = ADD_OUT_X_L;
    haccel->out_x_l.bias      = 2;
    haccel->out_x_l.len       = 6;
    haccel->out_x_l.read_only = 1;
    haccel->out_x_l.is_read   = 0;
    haccel->out_x_l.value     = 0;

    haccel->out_x_h.address   = ADD_OUT_X_H;
    haccel->out_x_h.bias      = 0;
    haccel->out_x_h.len       = 8;
    haccel->out_x_h.read_only = 1;
    haccel->out_x_h.is_read   = 0;
    haccel->out_x_h.value     = 0;

    haccel->out_y_l.address   = ADD_OUT_Y_L;
    haccel->out_y_l.bias      = 2;
    haccel->out_y_l.len       = 6;
    haccel->out_y_l.read_only = 1;
    haccel->out_y_l.is_read   = 0;
    haccel->out_y_l.value     = 0;

    haccel->out_y_h.address   = ADD_OUT_Y_H;
    haccel->out_y_h.bias      = 0;
    haccel->out_y_h.len       = 8;
    haccel->out_y_h.read_only = 1;
    haccel->out_y_h.is_read   = 0;
    haccel->out_y_h.value     = 0;

    haccel->out_z_l.address   = ADD_OUT_Z_L;
    haccel->out_z_l.bias      = 2;
    haccel->out_z_l.len       = 6;
    haccel->out_z_l.read_only = 1;
    haccel->out_z_l.is_read   = 0;
    haccel->out_z_l.value     = 0;

    haccel->out_z_h.address   = ADD_OUT_Z_H;
    haccel->out_z_h.bias      = 0;
    haccel->out_z_h.len       = 8;
    haccel->out_z_h.read_only = 1;
    haccel->out_z_h.is_read   = 0;
    haccel->out_z_h.value     = 0;

    haccel->fth.address   = ADD_FIFO_CTRL;
    haccel->fth.bias      = 0;
    haccel->fth.len       = 5;
    haccel->fth.read_only = 0;
    haccel->fth.is_read   = 0;
    haccel->fth.value     = 0;

    haccel->fmode.address   = ADD_FIFO_CTRL;
    haccel->fmode.bias      = 5;
    haccel->fmode.len       = 3;
    haccel->fmode.read_only = 0;
    haccel->fmode.is_read   = 0;
    haccel->fmode.value     = 0;

    haccel->fmode.address   = ADD_FIFO_CTRL;
    haccel->fmode.bias      = 5;
    haccel->fmode.len       = 3;
    haccel->fmode.read_only = 0;
    haccel->fmode.is_read   = 0;
    haccel->fmode.value     = 0;

    haccel->diff.address   = ADD_FIFO_SAMPLES;
    haccel->diff.bias      = 0;
    haccel->diff.len       = 6;
    haccel->diff.read_only = 1;
    haccel->diff.is_read   = 0;
    haccel->diff.value     = 0;

    haccel->fifo_ovr.address   = ADD_FIFO_SAMPLES;
    haccel->fifo_ovr.bias      = 6;
    haccel->fifo_ovr.len       = 1;
    haccel->fifo_ovr.read_only = 1;
    haccel->fifo_ovr.is_read   = 0;
    haccel->fifo_ovr.value     = 0;

    haccel->fifo_fth.address   = ADD_FIFO_SAMPLES;
    haccel->fifo_fth.bias      = 7;
    haccel->fifo_fth.len       = 1;
    haccel->fifo_fth.read_only = 1;
    haccel->fifo_fth.is_read   = 0;
    haccel->fifo_fth.value     = 0;

    haccel->tap_thsx.address   = ADD_TAP_THS_X;
    haccel->tap_thsx.bias      = 0;
    haccel->tap_thsx.len       = 5;
    haccel->tap_thsx.read_only = 0;
    haccel->tap_thsx.is_read   = 0;
    haccel->tap_thsx.value     = 0;

    haccel->ths_6d.address   = ADD_TAP_THS_X;
    haccel->ths_6d.bias      = 5;
    haccel->ths_6d.len       = 2;
    haccel->ths_6d.read_only = 0;
    haccel->ths_6d.is_read   = 0;
    haccel->ths_6d.value     = 0;

    haccel->en_4d.address   = ADD_TAP_THS_X;
    haccel->en_4d.bias      = 7;
    haccel->en_4d.len       = 1;
    haccel->en_4d.read_only = 0;
    haccel->en_4d.is_read   = 0;
    haccel->en_4d.value     = 0;

    haccel->tap_thsy.address   = ADD_TAP_THS_Y;
    haccel->tap_thsy.bias      = 0;
    haccel->tap_thsy.len       = 5;
    haccel->tap_thsy.read_only = 0;
    haccel->tap_thsy.is_read   = 0;
    haccel->tap_thsy.value     = 0;

    haccel->tap_prior.address   = ADD_TAP_THS_Y;
    haccel->tap_prior.bias      = 5;
    haccel->tap_prior.len       = 3;
    haccel->tap_prior.read_only = 0;
    haccel->tap_prior.is_read   = 0;
    haccel->tap_prior.value     = 0;

    haccel->tap_thsz.address   = ADD_TAP_THS_Z;
    haccel->tap_thsz.bias      = 0;
    haccel->tap_thsz.len       = 5;
    haccel->tap_thsz.read_only = 0;
    haccel->tap_thsz.is_read   = 0;
    haccel->tap_thsz.value     = 0;

    haccel->tap_z_en.address   = ADD_TAP_THS_Z;
    haccel->tap_z_en.bias      = 5;
    haccel->tap_z_en.len       = 1;
    haccel->tap_z_en.read_only = 0;
    haccel->tap_z_en.is_read   = 0;
    haccel->tap_z_en.value     = 0;

    haccel->tap_y_en.address   = ADD_TAP_THS_Z;
    haccel->tap_y_en.bias      = 6;
    haccel->tap_y_en.len       = 1;
    haccel->tap_y_en.read_only = 0;
    haccel->tap_y_en.is_read   = 0;
    haccel->tap_y_en.value     = 0;

    haccel->tap_x_en.address   = ADD_TAP_THS_Z;
    haccel->tap_x_en.bias      = 7;
    haccel->tap_x_en.len       = 1;
    haccel->tap_x_en.read_only = 0;
    haccel->tap_x_en.is_read   = 0;
    haccel->tap_x_en.value     = 0;

    haccel->shock.address   = ADD_INT_DUR;
    haccel->shock.bias      = 0;
    haccel->shock.len       = 2;
    haccel->shock.read_only = 0;
    haccel->shock.is_read   = 0;
    haccel->shock.value     = 0;

    haccel->quiet.address   = ADD_INT_DUR;
    haccel->quiet.bias      = 2;
    haccel->quiet.len       = 2;
    haccel->quiet.read_only = 0;
    haccel->quiet.is_read   = 0;
    haccel->quiet.value     = 0;

    haccel->latency.address   = ADD_INT_DUR;
    haccel->latency.bias      = 4;
    haccel->latency.len       = 4;
    haccel->latency.read_only = 0;
    haccel->latency.is_read   = 0;
    haccel->latency.value     = 0;

    haccel->wk_ths.address   = ADD_WAKE_UP_THS;
    haccel->wk_ths.bias      = 0;
    haccel->wk_ths.len       = 6;
    haccel->wk_ths.read_only = 0;
    haccel->wk_ths.is_read   = 0;
    haccel->wk_ths.value     = 0;

    haccel->sleep_on.address   = ADD_WAKE_UP_THS;
    haccel->sleep_on.bias      = 6;
    haccel->sleep_on.len       = 1;
    haccel->sleep_on.read_only = 0;
    haccel->sleep_on.is_read   = 0;
    haccel->sleep_on.value     = 0;

    haccel->single_double_tap.address   = ADD_WAKE_UP_THS;
    haccel->single_double_tap.bias      = 7;
    haccel->single_double_tap.len       = 1;
    haccel->single_double_tap.read_only = 0;
    haccel->single_double_tap.is_read   = 0;
    haccel->single_double_tap.value     = 0;

    haccel->sleep_dur.address   = ADD_WAKE_UP_DUR;
    haccel->sleep_dur.bias      = 0;
    haccel->sleep_dur.len       = 4;
    haccel->sleep_dur.read_only = 0;
    haccel->sleep_dur.is_read   = 0;
    haccel->sleep_dur.value     = 0;

    haccel->stationary.address   = ADD_WAKE_UP_DUR;
    haccel->stationary.bias      = 4;
    haccel->stationary.len       = 1;
    haccel->stationary.read_only = 0;
    haccel->stationary.is_read   = 0;
    haccel->stationary.value     = 0;

    haccel->wake_dur.address   = ADD_WAKE_UP_DUR;
    haccel->wake_dur.bias      = 5;
    haccel->wake_dur.len       = 2;
    haccel->wake_dur.read_only = 0;
    haccel->wake_dur.is_read   = 0;
    haccel->wake_dur.value     = 0;

    haccel->ff_dur_5.address   = ADD_WAKE_UP_DUR;
    haccel->ff_dur_5.bias      = 7;
    haccel->ff_dur_5.len       = 1;
    haccel->ff_dur_5.read_only = 0;
    haccel->ff_dur_5.is_read   = 0;
    haccel->ff_dur_5.value     = 0;

    haccel->ff_dur_04.address   = ADD_FREE_FALL;
    haccel->ff_dur_04.bias      = 3;
    haccel->ff_dur_04.len       = 5;
    haccel->ff_dur_04.read_only = 0;
    haccel->ff_dur_04.is_read   = 0;
    haccel->ff_dur_04.value     = 0;

    haccel->ff_ths.address   = ADD_FREE_FALL;
    haccel->ff_ths.bias      = 0;
    haccel->ff_ths.len       = 3;
    haccel->ff_ths.read_only = 0;
    haccel->ff_ths.is_read   = 0;
    haccel->ff_ths.value     = 0;

    haccel->drdy_dup.address   = ADD_STATUS_DUP;
    haccel->drdy_dup.bias      = 0;
    haccel->drdy_dup.len       = 1;
    haccel->drdy_dup.read_only = 1;
    haccel->drdy_dup.is_read   = 0;
    haccel->drdy_dup.value     = 0;

    haccel->ff_ia_dup.address   = ADD_STATUS_DUP;
    haccel->ff_ia_dup.bias      = 1;
    haccel->ff_ia_dup.len       = 1;
    haccel->ff_ia_dup.read_only = 1;
    haccel->ff_ia_dup.is_read   = 0;
    haccel->ff_ia_dup.value     = 0;

    haccel->ia_6d_dup.address   = ADD_STATUS_DUP;
    haccel->ia_6d_dup.bias      = 2;
    haccel->ia_6d_dup.len       = 1;
    haccel->ia_6d_dup.read_only = 1;
    haccel->ia_6d_dup.is_read   = 0;
    haccel->ia_6d_dup.value     = 0;

    haccel->single_tap_dup.address   = ADD_STATUS_DUP;
    haccel->single_tap_dup.bias      = 3;
    haccel->single_tap_dup.len       = 1;
    haccel->single_tap_dup.read_only = 1;
    haccel->single_tap_dup.is_read   = 0;
    haccel->single_tap_dup.value     = 0;

    haccel->double_tap_dup.address   = ADD_STATUS_DUP;
    haccel->double_tap_dup.bias      = 4;
    haccel->double_tap_dup.len       = 1;
    haccel->double_tap_dup.read_only = 1;
    haccel->double_tap_dup.is_read   = 0;
    haccel->double_tap_dup.value     = 0;

    haccel->sleep_state_ia_dup.address   = ADD_STATUS_DUP;
    haccel->sleep_state_ia_dup.bias      = 5;
    haccel->sleep_state_ia_dup.len       = 1;
    haccel->sleep_state_ia_dup.read_only = 1;
    haccel->sleep_state_ia_dup.is_read   = 0;
    haccel->sleep_state_ia_dup.value     = 0;

    haccel->drdy_t_dup.address   = ADD_STATUS_DUP;
    haccel->drdy_t_dup.bias      = 6;
    haccel->drdy_t_dup.len       = 1;
    haccel->drdy_t_dup.read_only = 1;
    haccel->drdy_t_dup.is_read   = 0;
    haccel->drdy_t_dup.value     = 0;

    haccel->ovr_dup.address   = ADD_STATUS_DUP;
    haccel->ovr_dup.bias      = 7;
    haccel->ovr_dup.len       = 1;
    haccel->ovr_dup.read_only = 1;
    haccel->ovr_dup.is_read   = 0;
    haccel->ovr_dup.value     = 0;

    haccel->z_wu.address   = ADD_WAKE_UP_SRC;
    haccel->z_wu.bias      = 0;
    haccel->z_wu.len       = 1;
    haccel->z_wu.read_only = 1;
    haccel->z_wu.is_read   = 0;
    haccel->z_wu.value     = 0;

    haccel->y_wu.address   = ADD_WAKE_UP_SRC;
    haccel->y_wu.bias      = 1;
    haccel->y_wu.len       = 1;
    haccel->y_wu.read_only = 1;
    haccel->y_wu.is_read   = 0;
    haccel->y_wu.value     = 0;

    haccel->x_wu.address   = ADD_WAKE_UP_SRC;
    haccel->x_wu.bias      = 2;
    haccel->x_wu.len       = 1;
    haccel->x_wu.read_only = 1;
    haccel->x_wu.is_read   = 0;
    haccel->x_wu.value     = 0;

    haccel->wu_ia.address   = ADD_WAKE_UP_SRC;
    haccel->wu_ia.bias      = 3;
    haccel->wu_ia.len       = 1;
    haccel->wu_ia.read_only = 1;
    haccel->wu_ia.is_read   = 0;
    haccel->wu_ia.value     = 0;

    haccel->sleep_state_ia.address   = ADD_WAKE_UP_SRC;
    haccel->sleep_state_ia.bias      = 4;
    haccel->sleep_state_ia.len       = 1;
    haccel->sleep_state_ia.read_only = 1;
    haccel->sleep_state_ia.is_read   = 0;
    haccel->sleep_state_ia.value     = 0;

    haccel->ff_ia.address   = ADD_WAKE_UP_SRC;
    haccel->ff_ia.bias      = 5;
    haccel->ff_ia.len       = 1;
    haccel->ff_ia.read_only = 1;
    haccel->ff_ia.is_read   = 0;
    haccel->ff_ia.value     = 0;

    haccel->z_tap.address   = ADD_TAP_SRC;
    haccel->z_tap.bias      = 0;
    haccel->z_tap.len       = 1;
    haccel->z_tap.read_only = 1;
    haccel->z_tap.is_read   = 0;
    haccel->z_tap.value     = 0;

    haccel->y_tap.address   = ADD_TAP_SRC;
    haccel->y_tap.bias      = 1;
    haccel->y_tap.len       = 1;
    haccel->y_tap.read_only = 1;
    haccel->y_tap.is_read   = 0;
    haccel->y_tap.value     = 0;

    haccel->x_tap.address   = ADD_TAP_SRC;
    haccel->x_tap.bias      = 2;
    haccel->x_tap.len       = 1;
    haccel->x_tap.read_only = 1;
    haccel->x_tap.is_read   = 0;
    haccel->x_tap.value     = 0;

    haccel->tap_sign.address   = ADD_TAP_SRC;
    haccel->tap_sign.bias      = 3;
    haccel->tap_sign.len       = 1;
    haccel->tap_sign.read_only = 1;
    haccel->tap_sign.is_read   = 0;
    haccel->tap_sign.value     = 0;

    haccel->double_tap_src.address   = ADD_TAP_SRC;
    haccel->double_tap_src.bias      = 4;
    haccel->double_tap_src.len       = 1;
    haccel->double_tap_src.read_only = 1;
    haccel->double_tap_src.is_read   = 0;
    haccel->double_tap_src.value     = 0;

    haccel->single_tap_src.address   = ADD_TAP_SRC;
    haccel->single_tap_src.bias      = 5;
    haccel->single_tap_src.len       = 1;
    haccel->single_tap_src.read_only = 1;
    haccel->single_tap_src.is_read   = 0;
    haccel->single_tap_src.value     = 0;

    haccel->tap_ia.address   = ADD_TAP_SRC;
    haccel->tap_ia.bias      = 6;
    haccel->tap_ia.len       = 1;
    haccel->tap_ia.read_only = 1;
    haccel->tap_ia.is_read   = 0;
    haccel->tap_ia.value     = 0;

    haccel->xl.address   = ADD_SIXD_SRC;
    haccel->xl.bias      = 0;
    haccel->xl.len       = 1;
    haccel->xl.read_only = 1;
    haccel->xl.is_read   = 0;
    haccel->xl.value     = 0;

    haccel->xh.address   = ADD_SIXD_SRC;
    haccel->xh.bias      = 1;
    haccel->xh.len       = 1;
    haccel->xh.read_only = 1;
    haccel->xh.is_read   = 0;
    haccel->xh.value     = 0;

    haccel->yl.address   = ADD_SIXD_SRC;
    haccel->yl.bias      = 2;
    haccel->yl.len       = 1;
    haccel->yl.read_only = 1;
    haccel->yl.is_read   = 0;
    haccel->yl.value     = 0;

    haccel->yh.address   = ADD_SIXD_SRC;
    haccel->yh.bias      = 3;
    haccel->yh.len       = 1;
    haccel->yh.read_only = 1;
    haccel->yh.is_read   = 0;
    haccel->yh.value     = 0;

    haccel->zl.address   = ADD_SIXD_SRC;
    haccel->zl.bias      = 4;
    haccel->zl.len       = 1;
    haccel->zl.read_only = 1;
    haccel->zl.is_read   = 0;
    haccel->zl.value     = 0;

    haccel->zh.address   = ADD_SIXD_SRC;
    haccel->zh.bias      = 5;
    haccel->zh.len       = 1;
    haccel->zh.read_only = 1;
    haccel->zh.is_read   = 0;
    haccel->zh.value     = 0;

    haccel->ia_6d_src.address   = ADD_SIXD_SRC;
    haccel->ia_6d_src.bias      = 6;
    haccel->ia_6d_src.len       = 1;
    haccel->ia_6d_src.read_only = 1;
    haccel->ia_6d_src.is_read   = 0;
    haccel->ia_6d_src.value     = 0;

    haccel->x_ofs_usr.address   = ADD_X_OFS_USR;
    haccel->x_ofs_usr.bias      = 0;
    haccel->x_ofs_usr.len       = 8;
    haccel->x_ofs_usr.read_only = 0;
    haccel->x_ofs_usr.is_read   = 0;
    haccel->x_ofs_usr.value     = 0;

    haccel->y_ofs_usr.address   = ADD_Y_OFS_USR;
    haccel->y_ofs_usr.bias      = 0;
    haccel->y_ofs_usr.len       = 8;
    haccel->y_ofs_usr.read_only = 0;
    haccel->y_ofs_usr.is_read   = 0;
    haccel->y_ofs_usr.value     = 0;

    haccel->z_ofs_usr.address   = ADD_Z_OFS_USR;
    haccel->z_ofs_usr.bias      = 0;
    haccel->z_ofs_usr.len       = 8;
    haccel->z_ofs_usr.read_only = 0;
    haccel->z_ofs_usr.is_read   = 0;
    haccel->z_ofs_usr.value     = 0;

    haccel->lpass_on6d.address   = ADD_CTRL7;
    haccel->lpass_on6d.bias      = 0;
    haccel->lpass_on6d.len       = 1;
    haccel->lpass_on6d.read_only = 0;
    haccel->lpass_on6d.is_read   = 0;
    haccel->lpass_on6d.value     = 0;

    haccel->hp_ref_mode.address   = ADD_CTRL7;
    haccel->hp_ref_mode.bias      = 1;
    haccel->hp_ref_mode.len       = 1;
    haccel->hp_ref_mode.read_only = 0;
    haccel->hp_ref_mode.is_read   = 0;
    haccel->hp_ref_mode.value     = 0;

    haccel->usr_off_w.address   = ADD_CTRL7;
    haccel->usr_off_w.bias      = 2;
    haccel->usr_off_w.len       = 1;
    haccel->usr_off_w.read_only = 0;
    haccel->usr_off_w.is_read   = 0;
    haccel->usr_off_w.value     = 0;

    haccel->usr_off_on_wu.address   = ADD_CTRL7;
    haccel->usr_off_on_wu.bias      = 3;
    haccel->usr_off_on_wu.len       = 1;
    haccel->usr_off_on_wu.read_only = 0;
    haccel->usr_off_on_wu.is_read   = 0;
    haccel->usr_off_on_wu.value     = 0;

    haccel->usr_off_on_out.address   = ADD_CTRL7;
    haccel->usr_off_on_out.bias      = 4;
    haccel->usr_off_on_out.len       = 1;
    haccel->usr_off_on_out.read_only = 0;
    haccel->usr_off_on_out.is_read   = 0;
    haccel->usr_off_on_out.value     = 0;

    haccel->interrupts_en.address   = ADD_CTRL7;
    haccel->interrupts_en.bias      = 5;
    haccel->interrupts_en.len       = 1;
    haccel->interrupts_en.read_only = 0;
    haccel->interrupts_en.is_read   = 0;
    haccel->interrupts_en.value     = 0;

    haccel->int2_on_int1.address   = ADD_CTRL7;
    haccel->int2_on_int1.bias      = 6;
    haccel->int2_on_int1.len       = 1;
    haccel->int2_on_int1.read_only = 0;
    haccel->int2_on_int1.is_read   = 0;
    haccel->int2_on_int1.value     = 0;

    haccel->drdy_pulsed.address   = ADD_CTRL7;
    haccel->drdy_pulsed.bias      = 7;
    haccel->drdy_pulsed.len       = 1;
    haccel->drdy_pulsed.read_only = 0;
    haccel->drdy_pulsed.is_read   = 0;
    haccel->drdy_pulsed.value     = 0;

    aint1.ibyte             = 0;
    aint2.ibyte             = 0;
    a1_tap_single_dir.cbyte = 7; // all directions 0b00000111
    a1_tap_double_dir.cbyte = 7; // all directions

    a1_tap_single_thr.x_thr = 4; // 4/32 = 1/8g
    a1_tap_single_thr.y_thr = 4;
    a1_tap_single_thr.z_thr = 4;

    a1_tap_double_thr.x_thr = 4; // 4/32 = 1/8g
    a1_tap_double_thr.y_thr = 4;
    a1_tap_double_thr.z_thr = 4;

    err = accel_filter_set(haccel, 0, 0, fs_4); //Filter mode set
    err |= accel_int1_set(haccel, aint1);       //Interrupt 1 set
    err |= accel_int2_set(haccel, aint2);       //Interrupt 2 set
    //    accel_free_fall_set(&a1, FF_250, 1); //Free fall detection set
    //    accel_wake_up_set(&a1, 16, 2);       //Wake up mode set
    err |= accel_int_mode_set(haccel, PULSED); //Interrupt mode set
                                               //    accel_single_tap_set(&a1,
    //                         &a1_tap_single_dir,
    //                         &a1_tap_single_thr,
    //                         0,
    //                         0,
    //                         4);                   //Single tap set
    //    accel_orientation_param_set(&a1, STHR_60); //Orientation detection set
    err |= accel_int_disable(haccel);
    err |= accel_interface_set(haccel, SPI_4);
    err |= accel_bdu_set(haccel, 1);
    //=========================================================FIFO settings
    err |= accel_autoincrement_set(haccel, ACCEL_DISABLE);
    //accel_fifo_set(&a1, FIFO_OFF, 0);
    err |= accel_fifo_set(haccel, FIFO_CONTIN, 32);
    //accel_slp_mode_set(&a1, 0, 0);
    //=========================================================Turn on accel
    err |= accel_power_mode_set(haccel,
                                LOW_POWER_MODE,
                                LP_MODE_2,
                                IPM_50,
                                LN_OFF); //Power mode set

    accel_id_get(haccel);

    //==========================================================END OF ACCEL SETTINGS
    return err;
}
