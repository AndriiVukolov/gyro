/*
 * accel.h
 *
 *  Created on: Jun 10, 2025
 *      Author: andrii-vukolov
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include <stdint.h>

#define LITTLE_ENDIANNESS
//#define BIG_ENDIANNESS

#define ADD_OUT_T_L             0x0D
#define ADD_OUT_T_H             0x0E
#define ADD_WHO_AM_I            0x0F
#define ADD_CTRL1               0x20
#define ADD_CTRL2               0x21
#define ADD_CTRL3               0x22
#define ADD_CTRL4_INT1_PAD_CTRL 0x23
#define ADD_CTRL5_INT2_PAD_CTRL 0x24
#define ADD_CTRL6               0x25
#define ADD_OUT_T               0x26
#define ADD_STATUS              0x27
#define ADD_OUT_X_L             0x28
#define ADD_OUT_X_H             0x29
#define ADD_OUT_Y_L             0x2A
#define ADD_OUT_Y_H             0x2B
#define ADD_OUT_Z_L             0x2C
#define ADD_OUT_Z_H             0x2D
#define ADD_FIFO_CTRL           0x2E
#define ADD_FIFO_SAMPLES        0x2F
#define ADD_TAP_THS_X           0x30
#define ADD_TAP_THS_Y           0x31
#define ADD_TAP_THS_Z           0x32
#define ADD_INT_DUR             0x33
#define ADD_WAKE_UP_THS         0x34
#define ADD_WAKE_UP_DUR         0x35
#define ADD_FREE_FALL           0x36
#define ADD_STATUS_DUP          0x37
#define ADD_WAKE_UP_SRC         0x38
#define ADD_TAP_SRC             0x39
#define ADD_SIXD_SRC            0x3A
#define ADD_ALL_INT_SRC         0x3B
#define ADD_X_OFS_USR           0x3C
#define ADD_Y_OFS_USR           0x3D
#define ADD_Z_OFS_USR           0x3E
#define ADD_CTRL7               0x3F

/*@brief Types of accel-object errors
 *
 * */
typedef enum {
    accelOk,
    accelInitFalse,
    accelCommunicationFalse,
    accelPowerFalse
} accelError_t;

typedef struct {
    uint8_t value;
    uint8_t address;
    uint8_t len;  //bit qty
    uint8_t bias; //bit number
    uint8_t read_only;
    uint8_t isRead; //  =1 if the parameter is been read or been set and value is in accel-obj
} parameter_t;

//================================================Power Mode Section
/*@brief Power modes of accel-object
 * */
typedef enum { LowPowerMode, HighPerfomanceMode, SingleDataConversion } pmode_t;

/*@brief Low power modes of accel-object
 * */
typedef enum { LPMode1, LPMode2, LPMode3, LPMode4 } lpmode_t;

/*@brief Output data rates of accel-object
 * */
typedef enum {
    pd,
    hp1_6,
    ipm12_5,
    ipm25,
    ipm50,
    ipm100,
    ipm200,
    hp400,
    hp800,
    hp1600
} odr_t;

typedef enum {
    res12, //12 bit
    res14, //14 bit
} resol_t;

typedef enum { ln_off, ln_on } lowNoise_t;

//================================================Filter section
/*
• Only LPF1 (green path): BW_FILT[1:0] = 00 and FDS = 0. Low-pass filter 1 bandwidth.
• LPF1 + LPF2 (purple path): BW_FILT[1:0] to a value different from 00 and FDS = 0. Bandwidth: low-pass path.
• LPF1 + HP (blue path): FDS = 1. Bandwidth: high-pass path.*/
typedef enum { bw0, bw1, bw2, bw3 } bw_filt_t;

//================================================Status
typedef enum {
    latced, //DRDY_PULSED bit = 0 in the CTRL7[7]
    pulsed  //DRDY_PULSED bit = 1 in the CTRL7[7]
} drdyPinMode_t;

typedef enum {
    ff156mg,
    ff219mg,
    ff250mg,
    ff344mg,
    ff406mg,
    ff469mg,
    ff500mg
} ffthreshold_t;

//=========================================================================
//================================================Common settings structure

typedef struct {
    //================================================WHO_AM_I
    parameter_t ID;
    //================================================CTRL1
    parameter_t PMode;
    parameter_t LPMode;
    parameter_t ODR;
    //================================================CTRL2
    parameter_t SIM;
    parameter_t I2C_DISABLE;
    parameter_t IF_ADD_INC;
    parameter_t BDU; //BlockDataUpdate
    parameter_t CS_PU_DISC;
    parameter_t SOFT_RESET;
    parameter_t BOOT;
    //================================================CTRL3
    parameter_t SLP_MODE_1;
    parameter_t SLP_MODE_SEL;
    parameter_t H_LACTIVE;
    parameter_t LIR;
    parameter_t PP_OD;
    parameter_t ST1;
    parameter_t ST2;
    //=================================================CTRL4_INT1_PAD_CTRL
    parameter_t INT1_DRDY;
    parameter_t INT1_FTH;
    parameter_t INT1_DIFF5;
    parameter_t INT1_TAP;
    parameter_t INT1_FF;
    parameter_t INT1_WU;
    parameter_t INT1_SINGLE_TAP;
    parameter_t INT1_6D;
    //=================================================CTRL5_INT2_PAD_CTRL
    parameter_t INT2_DRDY;
    parameter_t INT2_FTH;
    parameter_t INT2_DIFF5;
    parameter_t INT2_OVR;
    parameter_t INT2_DRDY_T;
    parameter_t INT2_BOOT;
    parameter_t INT2_SLEEP_CHG;
    parameter_t INT2_SLEEP_STATE;
    //==================================================CTRL6
    parameter_t LOW_NOISE;
    parameter_t FDS;
    parameter_t FS; //full scale
    parameter_t BW_FILT;
    //==================================================OUT_T
    parameter_t OUT_TEMP;
    //==================================================STATUS
    parameter_t DRDY;
    parameter_t FF_IA_ST;
    parameter_t IA_6D;
    parameter_t SINGLE_TAP;
    parameter_t DOUBLE_TAP;
    parameter_t SLEEP_STATE;
    parameter_t WU_IA_ST;
    parameter_t FIFO_THS;
    //================================================OUT_X_L
    parameter_t OUT_X_L;
    //================================================OUT_X_H
    parameter_t OUT_X_H;
    //================================================OUT_Y_L
    parameter_t OUT_Y_L;
    //================================================OUT_Y_H
    parameter_t OUT_Y_H;
    //================================================OUT_Z_L
    parameter_t OUT_Z_L;
    //================================================OUT_Z_H
    parameter_t OUT_Z_H;
    //================================================FIFO_CTRL
    parameter_t FTH;
    parameter_t FMode;
    //================================================FIFO_SAMPLES
    parameter_t Diff;
    parameter_t FIFO_OVR;
    parameter_t FIFO_FTH;
    //================================================TAP_THS_X
    parameter_t TAP_THSX;
    parameter_t THS_6D;
    parameter_t EN_4D;
    //================================================TAP_THS_Y
    parameter_t TAP_THSY;
    parameter_t TAP_PRIOR;
    //================================================TAP_THS_Z
    parameter_t TAP_THSZ;
    parameter_t TAP_Z_EN;
    parameter_t TAP_Y_EN;
    parameter_t TAP_X_EN;
    //================================================INT_DUR
    parameter_t SHOCK;
    parameter_t QUIET;
    parameter_t LATENCY;
    //================================================WAKE_UP_THS
    parameter_t WK_THS;
    parameter_t SLEEP_ON;
    parameter_t SINGLE_DOUBLE_TAP;
    //================================================WAKE_UP_DUR
    parameter_t SLEEP_DUR;
    parameter_t STATIONARY;
    parameter_t WAKE_DUR;
    //===============================================FREE_FALL
    parameter_t FF_DUR;
    parameter_t FF_THS;
    //================================================STATUS_DUP
    parameter_t DRDY_DUP;
    parameter_t FF_IA_DUP;
    parameter_t IA_6D_DUP;
    parameter_t SINGLE_TAP_DUP;
    parameter_t DOUBLE_TAP_DUP;
    parameter_t SLEEP_STATE_IA_DUP;
    parameter_t DRDY_T;
    parameter_t OVR;
    //================================================WAKE_UP_SRC
    parameter_t Z_WU;
    parameter_t Y_WU;
    parameter_t X_WU;
    parameter_t WU_IA;
    parameter_t SLEEP_STATE_IA;
    parameter_t FF_IA;
    //===============================================TAP_SRC
    parameter_t Z_TAP;
    parameter_t Y_TAP;
    parameter_t X_TAP;
    parameter_t TAP_SIGN;
    parameter_t DOUBLE_TAP_SRC;
    parameter_t SINGLE_TAP_SRC;
    parameter_t TAP_IA;
    //=================================================SIXD_SRC
    parameter_t XL;
    parameter_t XH;
    parameter_t YL;
    parameter_t YH;
    parameter_t ZL;
    parameter_t ZH;
    parameter_t IA_6D_SRC;
    //==================================================ALL_INT_SRC
    parameter_t FF_IA_SRC;
    parameter_t WU_IA_SRC;
    parameter_t SINGLE_TAP_INT_SRC;
    parameter_t DOUBLE_TAP_INT_SRC;
    parameter_t IA_6D_INT_SRC;
    parameter_t SLEEP_CHANGE_IA_INT_SRC;
    //==================================================X_OFS_USR
    parameter_t X_OFS_USR;
    //==================================================Y_OFS_USR
    parameter_t Y_OFS_USR;
    //==================================================Z_OFS_USR
    parameter_t Z_OFS_USR;
    //==================================================CTRL7
    parameter_t LPASS_ON6D;
    parameter_t HP_REF_MODE;
    parameter_t USR_OFF_W;
    parameter_t USR_OFF_ON_WU;
    parameter_t USR_OFF_ON_OUT;
    parameter_t INTERRUPTS_ENABLE;
    parameter_t INT2_ON_INT1;
    parameter_t DRDY_PULSED; //DRDY pin mode

    accelError_t (*dataWrite)(uint8_t startAdd, uint8_t len, uint8_t *source);
    accelError_t (*dataRead)(uint8_t startAdd, uint8_t len, uint8_t *dest);

} accel_t;

/**@brief Set the power mode of accel-object
 * haccel - accel_t object
 * mode - power mode
 * lpmode - low power mode
 * odr - output data rate
 * ln - low noise bit
 * returns accelOk = 0 or error code
 */
accelError_t accelPowerModeSet(accel_t   *haccel,
                               pmode_t    mode,
                               lpmode_t   lpmode,
                               odr_t      odr,
                               lowNoise_t ln); //power mode set

/*@brief Set the filter mode of accel obj
 * bw - bandwidth filter mode
 * fds - FDS-bit
 * returns accelOk = 0 or error code
 * */
accelError_t accelFilterSet(accel_t *haccel, uint8_t bw, uint8_t fds);

/*@brief Set interrupt 1 configuration bits *
 * • INT1_6D: 6D recognition is routed to the INT1 pin.
 * • INT1_SINGLE_TAP: single-tap event recognition is routed to the INT1 pin.
 * • INT1_WU: wake-up event recognition is routed to the INT1 pin.
 * • INT1_FF: free-fall event recognition is routed to the INT1 pin.
 * • INT1_TAP: double-tap event recognition is routed to the INT1 pin.
 * • INT1_DIFF5: FIFO full recognition is routed to the INT1 pin.
 * • INT1_FTH: FIFO threshold event is routed to the INT1 pin.
 * • INT1_DRDY: accelerometer data-ready is routed to the INT1 pin.
 * returns accelOk = 0 or error code
 * */
accelError_t accelInt1Set(accel_t *haccel,
                          uint8_t  int1_drdy,
                          uint8_t  int1_FifioThr,
                          uint8_t  int1_diff5,
                          uint8_t  int1_tap,
                          uint8_t  int1_ff,
                          uint8_t  int1_wu,
                          uint8_t  int1_single_tap,
                          uint8_t  int1_6D);

/*@brief Set interrupt 2 configuration bits *
 * • INT2_SLEEP_STATE: enable routing SLEEP_STATE to the INT2 pin.
 * • INT2_SLEEP_CHG: sleep change status routed to the INT2 pin.
 * • INT2 _BOOT: boot state routed to the INT2 pin.
 * • INT2_DRDY_T: temperature data-ready is routed to the INT2 pin.
 * • INT2 _OVR: FIFO overrun interrupt is routed to the INT2 pin.
 * • INT2_DIFF5: FIFO full recognition is routed to the INT2 pin.
 * • INT2_FTH: FIFO threshold event is routed to the INT2 pin.
 * • INT2_DRDY: accelerometer data-ready to the INT2 pin.
 * returns accelOk = 0 or error code
 * */
accelError_t accelInt2Set(accel_t *haccel,
                          uint8_t  int2_drdy,
                          uint8_t  int2_FifioThr,
                          uint8_t  int2_diff5,
                          uint8_t  int2_ovr,
                          uint8_t  int2_drdy_t,
                          uint8_t  int2_boot,
                          uint8_t  int2_sleep_chg,
                          uint8_t  int2_sleep_state);

/*@brief set the threshold for free-fall detection
 * threshold - threshold value
 * duration - duration value
 * returns accelOk = 0 or error code
 * */
accelError_t
accelFreeFallSet(accel_t *haccel, ffthreshold_t thrashoold, uint8_t duration);

/*@brief Get accelerometer data
 * adata - array of 3-axis data
 * returns accelOk = 0 or error code
 *  */
accelError_t accelDataGet(accel_t *haccel, float *adata);

/**@brief Get interrupt sources
 * • STATUS (27h) or STATUS_DUP (37h)
 * • WAKE_UP_SRC (38h)
 * • TAP_SRC (39h)
 * • SIXD_SRC (3Ah)
 * • ALL_INT_SRC (3Bh)
 * returns accelOk = 0 or error code
 */
accelError_t accelIntSourceGet(accel_t *haccel, uint8_t *intSources);

#endif /* INC_ACCEL_H_ */
