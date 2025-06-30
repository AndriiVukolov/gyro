/*
 * gyro_I3G4250D.c
 * @brief Driver for hyroscope motion sensor I3G4250D
 *  Created on: May 28, 2025
 *      Author: Andrii V.
 *
 */

#include "gyro_I3G4250D.h"
#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"

static uint16_t
LPCutoffCalculation(gyro_t *hGyro) //table 21 (result need to divide by 10)
{
    uint16_t lpcf          = 0;
    uint8_t  coff          = 0 | (hGyro->DataRate << 2) | hGyro->BandWidth;
    uint16_t coffTable[16] = { 125, 250, 250, 250,  125, 250, 500, 700,
                               200, 250, 500, 1100, 300, 350, 500, 1100 };
    lpcf                   = coffTable[coff];

    return lpcf;
}

static drate_t OdrCalculation(gyro_t *hGyro)
{

    uint16_t odr = (uint16_t)(pow(2, hGyro->DataRate) * 100);
    return (drate_t)odr;
}

static gyroPowMode_t PModeCalculation(gyro_t *hGyro)
{
    gyroPowMode_t pm;
    if (hGyro->PDownModeEn == 0)
        pm = DOWN;
    else if ((hGyro->Zen + hGyro->Yen + hGyro->Xen) == 0)
        pm = SLEEP;
    else
        pm = NORMAL;
    return pm;
}

static uint16_t
HPCutoffCalculation(gyro_t *hGyro) //table 26 (need to divide by 100)
{
    uint16_t hpf           = 0;
    uint16_t rowOdr[4][10] = {
        { 800, 400, 200, 100, 50, 20, 10, 5, 2, 1 },
        { 1500, 800, 400, 200, 100, 50, 20, 10, 5, 2 },
        { 3000, 1500, 800, 400, 200, 100, 50, 20, 10, 5 },
        { 5600, 3000, 1500, 800, 400, 200, 100, 50, 20, 10 }
    };
    hpf = rowOdr[hGyro->DataRate][hGyro->HPCutOffCode];
    return hpf;
}

static float AngleCalculate(gyro_t *hGyro, int16_t angle)
{
    float res   = 0;
    float scale = 0;

    switch (hGyro->FullScale) {
    case FS245:
        scale = 8.75;
        break;
    case FS500:
        scale = 17.5;
        break;
    case FS2000:
        scale = 70.0;
        break;
    case FS2000_2:
        scale = 70.0;
        break;
    }
    res = ((float)angle * scale) / 1000;
    return res;
}

gyroError_t gyroReadID(gyro_t *hGyro)
{
    uint8_t idByte = 0;
    uint8_t err    = hGyro->data_read(ADD_WHO_AM_I, 1, &idByte);

    /*WHO_AM_I (0Fh)*/
    if (err != gyroOk)
        err = gyroCommError;
    else
        hGyro->Id = idByte;

    return err;
}

gyroError_t gyroReadAll(gyro_t *hGyro, uint8_t *regs)
{
    gyroError_t err = gyroOk;

    err = hGyro->data_read(ADD_CTRL_REG1, 8, &regs[0]);
    err = hGyro->data_read(
            ADD_OUT_X_L,
            6,
            &regs[8]); //cyclic increment of address inside 6 value registers
    err = hGyro->data_read(ADD_FIFO_CTRL_REG, 11, &regs[14]);

    return err;
}

gyroError_t gyroInit(gyro_t *hGyro)
{
    gyroError_t err;

    //==========================================================GYRO SETTINGS

    //=====================REG1==============
    hGyro->DataRate    = ODR100;
    hGyro->BandWidth   = 3;
    hGyro->PDownModeEn = 1;
    hGyro->Zen         = 1;
    hGyro->Yen         = 1;
    hGyro->Xen         = 1;
    //=====================REG2==============
    hGyro->HPFilterMode =
            0; //Normal mode (reset by reading the REFERENCE/DATACAPTURE (25h) register)
    hGyro->HPCutOffCode = 0;
    //=====================REG3==============
    hGyro->Int1En       = DISABLE;
    hGyro->Int1Boot     = DISABLE;
    hGyro->IntActConfig = 0;
    hGyro->Int2En       = DISABLE;
    hGyro->FIFOOvrnEn   = DISABLE;
    hGyro->FIFOEmptyEn  = DISABLE;
    //=====================REG4==============
    hGyro->Ble        = 0;
    hGyro->FullScale  = FS500;
    hGyro->SelfTestEn = DISABLE;
    hGyro->SPIModeSel = 0;
    //=====================REG5==============
    hGyro->Boot        = 0;
    hGyro->FifoEn      = 0;
    hGyro->HPFilterEn  = 0;
    hGyro->Int1SelConf = 0;
    hGyro->OutSelConf  = 0;
    //=====================INT1_CFG=========
    hGyro->Andor = 0;
    hGyro->Lir   = 0;
    hGyro->ZHIE  = 0;
    hGyro->ZLIE  = 0;
    hGyro->YHIE  = 0;
    hGyro->YLIE  = 0;
    hGyro->XHIE  = 0;
    hGyro->XLIE  = 0;
    //=====================INT1_DURATION====
    hGyro->Int1Wait     = 0;
    hGyro->Int1Duration = 1;
    //=====================INT1_THRESHOLDS==
    hGyro->Int1XHighTr = 0;
    hGyro->Int1XLowTr  = 0;
    hGyro->Int1YHighTr = 0;
    hGyro->Int1YLowTr  = 0;
    hGyro->Int1ZHighTr = 0;
    hGyro->Int1ZLowTr  = 0;
    //===================FIFO MODE SELECTION
    hGyro->FifoModeSel = 0;
    //=====================HighPath Filter Mode
    hGyro->HPFilterMode = HPNormal;
    hGyro->HPCutOffCode = HPCF0;

    //==========================================================END OF GYRO SETTINGS

    err = gyroSetOutMode(hGyro);
    err = gyroSetIntMode(hGyro);
    err = gyroSetFullScale(hGyro);
    err = gyroSetFifoMode(hGyro);
    err = gyroTurnOn(hGyro);
    err = gyroSetHPFilter(hGyro);

    return err;
}

gyroError_t gyroDeinit(gyro_t *hGyro)
{
    return gyroOk;
};

gyroError_t gyroTurnOff(gyro_t *hGyro) //CTRL_REG1 (20h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    hGyro->PowerMode = DOWN;
    err              = hGyro->data_read(ADD_CTRL_REG1, 1, &dataByte);
    if (err != gyroOk)
        return err;
    dataByte &= 0xF0;
    err = hGyro->data_write(ADD_CTRL_REG1, 1, &dataByte);
    return err;
}
gyroError_t gyroSleep(gyro_t *hGyro) //CTRL_REG1 (20h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    hGyro->PowerMode = SLEEP;
    err              = hGyro->data_read(ADD_CTRL_REG1, 1, &dataByte);
    if (err != gyroOk)
        return err;
    dataByte &= 0xF0;
    dataByte |= 0x08;
    err = hGyro->data_write(ADD_CTRL_REG1, 1, &dataByte);
    return err;
}
gyroError_t gyroTurnOn(gyro_t *hGyro) //CTRL_REG1 (20h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    hGyro->PowerMode = NORMAL;
    dataByte         = (hGyro->DataRate << 6) | (hGyro->BandWidth << 4) |
               (hGyro->PDownModeEn << 3) | (hGyro->Zen << 2) |
               (hGyro->Yen << 1) | hGyro->Xen;
    err = hGyro->data_write(ADD_CTRL_REG1, 1, &dataByte);
    return err;
}

gyroError_t gyroReadVal(gyro_t *hGyro,
                        float  *gVal) //OUT_X_L (28h) - OUT_Z_H (2Dh)
{
    uint8_t     buff[6] = { 0 };
    int16_t     val[3]  = { 0 };
    uint8_t     i       = 0;
    gyroError_t err     = gyroOk;
    uint8_t     n       = 0;

    err = gyroReadFifoStatus(hGyro);

    if (hGyro->FifoEn == 1)
        n = hGyro->FifoStored;
    else
        n = 1;

    if ((n == 31) && (hGyro->FifoOvrnStatus == 1))
        n = 32;

    while ((n--) > 0) {
        err = hGyro->data_read(ADD_OUT_X_L, 6, buff);
    }
    // check in the control register 4 the data alignment (Big Endian or Little Endian)
    //(0: data LSB @ lower address; 1: data MSB @ lower address)
    if (hGyro->Ble == 0) {
        for (i = 0; i < 3; i++) {
            val[i]  = (int16_t)(((uint16_t)buff[2 * i + 1] << 8) + buff[2 * i]);
            gVal[i] = AngleCalculate(hGyro, val[i]);
        }
    } else {
        for (i = 0; i < 3; i++) {
            val[i]  = (int16_t)(((uint16_t)buff[2 * i] << 8) + buff[2 * i + 1]);
            gVal[i] = AngleCalculate(hGyro, val[i]);
        }
    }
    hGyro->OutX = val[0];
    hGyro->OutY = val[1];
    hGyro->OutZ = val[2];

    return err;
}

gyroError_t gyroReadStatus(gyro_t *hGyro) //STATUS_REG (27h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    err = hGyro->data_read(ADD_STATUS_REG, 1, &dataByte);
    if (err != gyroOk)
        return err;
    hGyro->zyxor = (dataByte & 0x80) >> 7;
    hGyro->zor   = (dataByte & 0x40) >> 6;
    hGyro->yor   = (dataByte & 0x20) >> 5;
    hGyro->xor   = (dataByte & 0x10) >> 4;
    hGyro->zyxda = (dataByte & 0x08) >> 3;
    hGyro->zda   = (dataByte & 0x04) >> 2;
    hGyro->yda   = (dataByte & 0x02) >> 1;
    hGyro->xda   = (dataByte & 0x01) >> 0;
    return err;
}

gyroError_t gyroReadFifoStatus(gyro_t *hGyro) //FIFO_SRC_REG (2Fh)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    err = hGyro->data_read(ADD_FIFO_SRC_REG, 1, &dataByte);
    if (err != gyroOk)
        return err;
    hGyro->FifoWtmStatus  = (dataByte & 0x80) >> 7;
    hGyro->FifoOvrnStatus = (dataByte & 0x40) >> 6;
    hGyro->FifoEmpty      = (dataByte & 0x20) >> 5;
    hGyro->FifoStored     = (dataByte & 0x1F);
    err                   = hGyro->data_read(ADD_CTRL_REG1, 1, &dataByte);
    if (err != gyroOk)
        return err;
    hGyro->Boot        = (dataByte & 0x80) >> 7;
    hGyro->FifoEn      = (dataByte & 0x40) >> 6;
    hGyro->HPFilterEn  = (dataByte & 0x10) >> 4;
    hGyro->Int1SelConf = (dataByte & 0x0C) >> 2;
    hGyro->OutSelConf  = (dataByte & 0x03) >> 0;

    return err;
}

gyroError_t gyroReadIntStatus(gyro_t *hGyro) //INT1_SRC (31h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    err = hGyro->data_read(ADD_INT1_SRC, 1, &dataByte);
    if (err != gyroOk)
        return err;
    hGyro->InterruptFlags->IntFlag   = (dataByte & 0x40) >> 6;
    hGyro->InterruptFlags->ZhighFlag = (dataByte & 0x20) >> 5;
    hGyro->InterruptFlags->ZLowFlag  = (dataByte & 0x10) >> 4;
    hGyro->InterruptFlags->YhighFlag = (dataByte & 0x08) >> 3;
    hGyro->InterruptFlags->YlowFlag  = (dataByte & 0x04) >> 2;
    hGyro->InterruptFlags->XhighFlag = (dataByte & 0x02) >> 1;
    hGyro->InterruptFlags->XlowFlag  = (dataByte & 0x01) >> 0;
    return err;
}

gyroError_t gyroSetOutDataRate(gyro_t *hGyro,
                               drate_t dataRate) //CTRL_REG1 (20h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    hGyro->Odr      = dataRate;
    hGyro->DataRate = (uint8_t)sqrt((double)dataRate / 100);
    err             = hGyro->data_read(ADD_CTRL_REG1, 1, &dataByte);
    if (err != gyroOk)
        return err;
    dataByte = (dataByte & 0x3F) | (hGyro->DataRate << 6);
    err      = hGyro->data_write(ADD_CTRL_REG1, 1, &dataByte);
    return err;
}

gyroError_t gyroSetHPFilter(gyro_t *hGyro) //CTRL_REG2 (21h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    dataByte = 0 | (hGyro->HPFilterMode << 4) | hGyro->HPCutOffCode;
    err      = hGyro->data_write(ADD_CTRL_REG2, 1, &dataByte);
    return err;
}

gyroError_t gyroSetFifoMode(gyro_t *hGyro) //FIFO_CTRL_REG (2Eh)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    dataByte &= (hGyro->FifoModeSel << 5) | hGyro->FifoThreshold;
    err = hGyro->data_write(ADD_FIFO_CTRL_REG, 1, &dataByte);
    return err;
}

gyroError_t gyroSetIntMode(
        gyro_t *hGyro) // CTRL_REG3 (22h), INT1_CFG (30h), INT1_DURATION (38h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    //int sources
    dataByte = 0 | (hGyro->Andor << 7) | (hGyro->Lir << 6) |
               (hGyro->ZHIE << 5) | (hGyro->ZLIE << 4) | (hGyro->YHIE << 3) |
               (hGyro->YLIE << 2) | (hGyro->XHIE << 1) | hGyro->XLIE;
    err = hGyro->data_write(ADD_INT1_CFG, 1, &dataByte);
    if (err != gyroOk)
        return err;
    //FIFO int sources
    dataByte = 0 | (hGyro->Int1En << 7) | (hGyro->Int1Boot << 6) |
               (hGyro->IntActConfig << 5) | (hGyro->IOutType << 4) |
               (hGyro->Int2En << 3) | (hGyro->FIFOWtmEn << 2) |
               (hGyro->FIFOOvrnEn << 1) | hGyro->FIFOEmptyEn;
    err = hGyro->data_write(ADD_CTRL_REG3, 1, &dataByte);
    if (err != gyroOk)
        return err;
    //int duration
    dataByte = 0 | (hGyro->Int1Wait << 7) | hGyro->Int1Duration;
    err      = hGyro->data_write(ADD_INT1_DURATION, 1, &dataByte);
    //int thresholds
    err = hGyro->data_write(ADD_INT1_THS_XH, 1, &hGyro->Int1XHighTr);
    err = hGyro->data_write(ADD_INT1_THS_XL, 1, &hGyro->Int1XLowTr);
    err = hGyro->data_write(ADD_INT1_THS_YH, 1, &hGyro->Int1YHighTr);
    err = hGyro->data_write(ADD_INT1_THS_YL, 1, &hGyro->Int1YLowTr);
    err = hGyro->data_write(ADD_INT1_THS_ZH, 1, &hGyro->Int1ZHighTr);
    err = hGyro->data_write(ADD_INT1_THS_ZL, 1, &hGyro->Int1ZLowTr);
    return err;
}

gyroError_t gyroSetOutMode(gyro_t *hGyro) // CTRL_REG5 (24h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    dataByte = 0 | (hGyro->Boot << 7) | (hGyro->FifoEn << 6) |
               (hGyro->HPFilterEn << 4) | (hGyro->Int1SelConf << 2) |
               hGyro->OutSelConf;
    err = hGyro->data_write(ADD_CTRL_REG5, 1, &dataByte);
    return err;
}

gyroError_t gyroReadReg5(gyro_t *hGyro)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    err                = hGyro->data_read(ADD_CTRL_REG5, 1, &dataByte);
    hGyro->Boot        = (dataByte & 0x80) >> 7;
    hGyro->FifoEn      = (dataByte & 0x40) >> 6;
    hGyro->HPFilterEn  = (dataByte & 0x10) >> 4;
    hGyro->Int1SelConf = (dataByte & 0x0C) >> 2;
    hGyro->OutSelConf  = (dataByte & 0x03) >> 0;

    return err;
}

gyroError_t gyroReadReference(gyro_t *hGyro)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    err = hGyro->data_read(ADD_REFERENCE, 1, &dataByte);

    return err;
}

gyroError_t gyroSetBle(gyro_t *hGyro, uint8_t ble) //CTRL_REG4 (23h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;
    err                  = hGyro->data_read(ADD_CTRL_REG4, 1, &dataByte);
    if (err != gyroOk)
        return err;
    hGyro->Ble = ble;
    ble        = (ble << 6) | 0xBF;
    dataByte &= ble;
    err = hGyro->data_write(ADD_CTRL_REG4, 1, &dataByte);
    return err;
}

gyroError_t gyroSetFullScale(gyro_t *hGyro) //CTRL_REG4 (23h)
{
    uint8_t     dataByte = 0;
    gyroError_t err      = gyroOk;

    dataByte = 0 | (hGyro->Ble << 6) | (hGyro->FullScale << 4) |
               (hGyro->SelfTestEn << 1) | hGyro->SPIModeSel;
    err = hGyro->data_write(ADD_CTRL_REG4, 1, &dataByte);
    return err;
}

gyroError_t gyroSelfTest(gyro_t *hGyro, uint8_t conf) //CTRL_REG4 (23h)
{
    uint8_t     dataByte = 0;
    uint8_t     buf[32];
    gyroError_t err = gyroOk;

    err = hGyro->data_read(ADD_CTRL_REG4, 1, &dataByte);
    if (err != gyroOk)
        return err;
    dataByte |= conf << 1;
    err = hGyro->data_write(ADD_CTRL_REG4, 1, &dataByte);
    if (err != gyroOk)
        return err;
    gyroReadAll(hGyro, buf);
    if (hGyro->SelfTestEn != 0) {
        dataByte &= 0x79; //reset ST1-ST0 bits
        err = hGyro->data_write(ADD_CTRL_REG5, 1, &dataByte);
    }
    return err;
}

gyroError_t gyroReboot(gyro_t *hGyro)
{
    uint8_t     dataByte = 0;
    uint8_t     buf[32];
    gyroError_t err = gyroOk;

    err = hGyro->data_read(ADD_CTRL_REG5, 1, &dataByte);
    if (err != gyroOk)
        return err;
    dataByte |= 0x80;
    err = hGyro->data_write(ADD_CTRL_REG5, 1, &dataByte);
    if (err != gyroOk)
        return err;
    gyroReadAll(hGyro, buf);
    if (hGyro->Boot) {
        hGyro->Boot = 0;
        dataByte    = (hGyro->Boot << 7) | (hGyro->FifoEn << 6) |
                   (hGyro->HPFilterEn << 4) | (hGyro->Int1SelConf << 2) |
                   hGyro->OutSelConf;
        err = hGyro->data_write(ADD_CTRL_REG5, 1, &dataByte);
    }
    return err;
}
