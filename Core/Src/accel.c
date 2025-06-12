/*
 * accel.c
 *
 *  Created on: Jun 10, 2025
 *      Author: andrii-vukolov
 */
#include "accel.h"
#include <stdint.h>

/*
 * example: parameterSet(&acc1, &acc1.PMode, 2);
 * */
static accelError_t
parameterSet(accel_t *paccel, parameter_t *par, uint8_t value)
{
    accelError_t err  = accelOk;
    uint8_t      buf  = 0;
    uint8_t      mask = 0;

    for (uint8_t i = 0; i < par->len; i++)
        mask |= (1 << i);
    mask <<= par->bias;
    paccel->dataRead(par->address, 1, &buf);
    buf   = buf & (~mask);
    value = value << par->bias;
    buf |= value;

    par->value  = value;
    par->isRead = 1; // new value is in paccel-obj

    paccel->dataWrite(par->address, 1, &buf);
    return err;
}

static uint8_t parameterGet(accel_t *paccel, parameter_t *par)
{

    uint8_t value = 0;
    uint8_t mask  = 0;

    if (par->isRead == 0) //check if the parameter is been read
                          //and the value is in paccel-obj
    {
        paccel->dataRead(par->address, 1, &value);
        for (uint8_t i = 0; i < par->len; i++)
            mask |= (1 << i);
        mask <<= par->bias;
        value       = (value & mask) >> par->bias;
        par->value  = value;
        par->isRead = 1;
    } else {
        value = par->value;
    }

    return value;
}

//accelError_t (*parSet)(accel_t *paccel, parameter_t *par, uint8_t value);
//uint8_t (*parGet)(accel_t *paccel, parameter_t *par);

static float accelDataCalculate(accel_t *haccel, int16_t regdata)
{
    float   dt         = 0;
    float   sens[2][4] = { { 0.976, 1.952, 3.904, 7.808 },
                           { 0.244, 0.488, 0.976, 1.952 } };
    uint8_t fs_index   = parameterGet(haccel, &haccel->FS);
    uint8_t lp_index   = ((parameterGet(haccel, &haccel->LPMode) > 0) ? 1 : 0);
    dt                 = regdata * sens[lp_index][fs_index];
    return dt;
}

/**/
accelError_t accelPowerModeSet(accel_t   *haccel,
                               pmode_t    mode,
                               lpmode_t   lpmode,
                               odr_t      odr,
                               lowNoise_t ln)
{
    accelError_t err = accelOk;

    err = parameterSet(haccel, &haccel->PMode, mode);
    err |= parameterSet(haccel, &haccel->LPMode, lpmode);
    err |= parameterSet(haccel, &haccel->ODR, odr);
    err |= parameterSet(haccel, &haccel->LOW_NOISE, ln);

    return err;
}

/**/
accelError_t accelFilterSet(accel_t *haccel, uint8_t bw, uint8_t fds)
{
    accelError_t err = accelOk;
    err = parameterSet(haccel, &(haccel->BW_FILT), haccel->BW_FILT.value);
    err |= parameterSet(haccel, &(haccel->FDS), haccel->FDS.value);
    return err;
}

accelError_t accelInt1Set(accel_t *haccel,
                          uint8_t  int1_drdy,
                          uint8_t  int1_FifioThr,
                          uint8_t  int1_diff5,
                          uint8_t  int1_tap,
                          uint8_t  int1_ff,
                          uint8_t  int1_wu,
                          uint8_t  int1_single_tap,
                          uint8_t  int1_6D)
{
    accelError_t err     = accelOk;
    uint8_t      dataBuf = 0;
    dataBuf = 0 | (int1_6D << 7) | (int1_single_tap << 6) | (int1_wu << 5) |
              (int1_ff << 4) | (int1_tap << 3) | (int1_diff5 << 2) |
              (int1_FifioThr << 1) | (int1_drdy);
    err = haccel->dataWrite(ADD_CTRL4_INT1_PAD_CTRL, 1, &dataBuf);
    return err;
}

accelError_t accelInt2Set(accel_t *haccel,
                          uint8_t  int2_drdy,
                          uint8_t  int2_FifoThr,
                          uint8_t  int2_diff5,
                          uint8_t  int2_ovr,
                          uint8_t  int2_drdy_t,
                          uint8_t  int2_boot,
                          uint8_t  int2_sleep_chg,
                          uint8_t  int2_sleep_state)
{
    accelError_t err     = accelOk;
    uint8_t      dataBuf = 0;
    dataBuf              = 0 | (int2_sleep_state << 7) | (int2_sleep_chg << 6) |
              (int2_boot << 5) | (int2_drdy_t << 4) | (int2_ovr << 3) |
              (int2_diff5 << 2) | (int2_FifoThr << 1) | (int2_drdy);
    err = haccel->dataWrite(ADD_CTRL4_INT1_PAD_CTRL, 1, &dataBuf);
    return err;
}

accelError_t accelDataGet(accel_t *haccel, float *adata)
{
    accelError_t err        = accelOk;
    uint8_t      regBuf[6]  = { 0 };
    int16_t      val[3]     = { 0 };
    resol_t      resolution = res14;
    uint8_t      bufByte    = 0;
    uint8_t      i          = 0;

    /*resolution detection*/
    bufByte = parameterGet(haccel, &haccel->PMode);
    if (bufByte == 1)
        resolution = res14;
    else {
        bufByte = parameterGet(haccel, &haccel->LPMode);
        if (bufByte == 0)
            resolution = res12;
    }

    err = haccel->dataRead(ADD_OUT_X_L, 6, regBuf);
#ifdef LITTLE_ENDIANNESS
    for (i = 0; i < 3; i++) {
        val[i] = (regBuf[i * 2 + 1] << 8) | regBuf[2 * i];
    }
#else
    for (i = 0; i < 3; i++) {
        val[i] = (regBuf[i * 2] << 8) | regBuf[2 * i + 1];
    }
#endif

    for (i = 0; i < 3; i++) {

        adata[i] = accelDataCalculate(haccel, val[i]);
        if (resolution == res14)
            adata[i] = adata[i] / 4;
    }
    return err;
}

accelError_t accelIntSourceGet(accel_t *haccel, uint8_t *intSources)
{
    accelError_t err = accelOk;

    err = haccel->dataRead(ADD_STATUS_DUP, 5, intSources);

    return err;
}

accelError_t
accelFreeFallSet(accel_t *haccel, ffthreshold_t thrashold, uint8_t duration)
{
    accelError_t err = accelOk;
    err              = parameterSet(haccel, &haccel->FF_THS, thrashold);
    err |= parameterSet(haccel, &haccel->FF_DUR, duration);
    return err;
}

accelError_t accelInit(accel_t *haccel)
{
    accelError_t err = accelOk;

    return err;
}
