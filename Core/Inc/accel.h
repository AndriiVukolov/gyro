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

enum { ACCEL_DISABLE = 0, ACCEL_ENABLE = 1 };

/**@brief Types of accel-object errors */
typedef enum {
    ACCEL_OK,
    ACCEL_INIT_FALSE,
    ACCEL_COMM_FALSE,
    ACCEL_POWER_FALSE
} accel_error_t;

typedef enum { SPI_4, SPI_3, I2C } intarface_t;

/**@brief Every parameter - is a structure parameter_t*/
typedef struct {
    uint8_t value;
    uint8_t address;
    uint8_t len;  //bit qty
    uint8_t bias; //bit number
    uint8_t read_only;
    uint8_t is_read; //  =1 if the parameter is been read or been set and value is in accel-obj
} parameter_t;

//================================================Power Mode Section
/**@brief Power modes of accel-object */
typedef enum {
    LOW_POWER_MODE,
    HIGH_PERFOMANCE_MODE,
    SINGLE_DATA_CONVERSION
} pmode_t;

/**@brief Low power modes of accel-object */
typedef enum { LP_MODE_1, LP_MODE_2, LP_MODE_3, LP_MODE_4 } lpmode_t;

/**@brief Output data rates of accel-object */
typedef enum {
    POW_DOWN,
    HP_1_6,
    IPM_12_5,
    IPM_25,
    IPM_50,
    IPM_100,
    OPM_200,
    HP_400,
    HP_800,
    HP_1600
} odr_t;

/**@brief Full scale (g)*/
typedef enum { fs_2, fs_4, fs_8, fs_16 } full_scale_t;

/**@brief Output data resolution */
typedef enum {
    RES_12, //12 bit
    RES_14, //14 bit
} resol_t;

/**@brief Low noise mode */
typedef enum { LN_OFF, LN_ON } low_noise_t;

//================================================Filter section
/**@brief Bandwidth filter types
• Only LPF1 (green path): BW_FILT[1:0] = 00 and FDS = 0. Low-pass filter 1 bandwidth.
• LPF1 + LPF2 (purple path): BW_FILT[1:0] to a value different from 00 and FDS = 0. Bandwidth: low-pass path.
• LPF1 + HP (blue path): FDS = 1. Bandwidth: high-pass path.*/
typedef enum { BW_0, BW_1, BW_2, BW_3 } bw_filt_t;

//================================================Status

/**@brief Data-ready pin mode*/
typedef enum {
    LATCHED, //DRDY_PULSED bit = 0 in the CTRL7[7]
    PULSED   //DRDY_PULSED bit = 1 in the CTRL7[7]
} drdy_pin_mode_t;

/**@brief Free-fall threshold values*/
typedef enum {
    FF_156, //156 mg
    FF_219, //219 mg
    FF_250, //250 mg
    FF_344, //344 mg
    FF_406, //406 mg
    FF_469, //469 mg
    FF_500  //500 mg
} ffthreshold_t;

typedef enum {
    FIFO_OFF       = 0,
    FIFO_MODE      = 1,
    CONTIN_TO_FIFO = 3,
    BP_TO_CONTIN   = 4,
    FIFO_CONTIN    = 6
} fifo_mode_t;

typedef enum {
    FIFO_FULL,
    FIFO_OVRFULL,
    FIFO_EMPTY,
    FIFO_WATERMARK
} fifo_state_t;

//================================================Interrupts
/**@brief bit field for interrupt settings */
typedef union {
    struct {
        uint8_t drdy : 1; //accelerometer data-ready is routed to the INT1 or INT2 pin.
        uint8_t fifio_thr : 1; //FIFO threshold event is routed to the INT1 or INT2 pin.
        uint8_t diff5 : 1; //FIFO full recognition is routed to the INT1 or INT2 pin.
        uint8_t tap_ovr : 1; //double-tap event recognition is routed to the INT1 pin. //FIFO overrun interrupt is routed to the INT2 pin.
        uint8_t ff_drdy_t : 1; //free-fall event recognition is routed to the INT1 pin. //temperature data-ready is routed to the INT2 pin.
        uint8_t wu_boot : 1; //wake-up event recognition is routed to the INT1 pin. //boot state routed to the INT2 pin.
        uint8_t single_tap_sleep_chg : 1; //SINGLE_TAP: single-tap event recognition is routed to the INT1 pin. // sleep change status routed to the INT2 pin.
        uint8_t sixd_sleep_state : 1; //6D recognition is routed to the INT1 pin. // enable routing SLEEP_STATE to the INT2 pin.
    } ibit;
    uint8_t ibyte : 8;
} interrupt_t;

//================================================6D-Thresholds
typedef enum {
    STHR_80, // 80 deg
    STHR_70, // 70 deg
    STHR_60, // 60 deg
    STHR_50  //50 deg
} sixd_thr_t;
//================================================TAP 111

typedef union {
    struct {
        uint8_t x_dir : 1;
        uint8_t y_dir : 1;
        uint8_t z_dir : 1;
    } crd;
    uint8_t cbyte : 3;
} tap_dir_t;

typedef struct {
    uint8_t x_thr;
    uint8_t y_thr;
    uint8_t z_thr;
} tap_thr_t;
//=========================================================================
//================================================Common settings structure

/**@brief Structure with parameters of accel-object*/
typedef struct {
    //================================================WHO_AM_I
    parameter_t id;
    //================================================CTRL1
    parameter_t pmode;  //power mode
    parameter_t lpmode; //low power mode
    parameter_t odr;    //output data rate
    //================================================CTRL2
    parameter_t sim; //SPI mode (4-wire/3-wire)
    parameter_t i2c_disable;
    parameter_t if_add_inc;
    parameter_t bdu; //Block Data Update
    parameter_t cs_pu_disc;
    parameter_t soft_reset;
    parameter_t boot;
    //================================================CTRL3
    parameter_t slp_mode_1;
    parameter_t slp_mode_sel;
    parameter_t hl_active;
    parameter_t lir;
    parameter_t pp_od;
    parameter_t st1;
    parameter_t st2;
    //=================================================CTRL4_INT1_PAD_CTRL
    parameter_t int1_drdy;
    parameter_t int1_fth;
    parameter_t int1_diff5;
    parameter_t int1_tap;
    parameter_t int1_ff;
    parameter_t int1_wu;
    parameter_t int1_single_tap;
    parameter_t int1_6d;
    //=================================================CTRL5_INT2_PAD_CTRL
    parameter_t int2_drdy;
    parameter_t int2_fth;
    parameter_t int2_diff5;
    parameter_t int2_ovr;
    parameter_t int2_drdy_t;
    parameter_t int2_boot;
    parameter_t int2_sleep_chg;
    parameter_t int2_sleep_state;
    //==================================================CTRL6
    parameter_t low_noise;
    parameter_t fds;
    parameter_t fs; //full scale
    parameter_t bw_filt;
    //==================================================OUT_T
    parameter_t out_temp;
    //==================================================STATUS
    parameter_t drdy;
    parameter_t ff_ia_st;
    parameter_t ia_6d;
    parameter_t single_tap;
    parameter_t double_tap;
    parameter_t sleep_state;
    parameter_t wu_ia_st;
    parameter_t fifo_ths;
    //================================================OUT_X_L
    parameter_t out_x_l;
    //================================================OUT_X_H
    parameter_t out_x_h;
    //================================================OUT_Y_L
    parameter_t out_y_l;
    //================================================OUT_Y_H
    parameter_t out_y_h;
    //================================================OUT_Z_L
    parameter_t out_z_l;
    //================================================OUT_Z_H
    parameter_t out_z_h;
    //================================================FIFO_CTRL
    parameter_t fth;
    parameter_t fmode;
    //================================================FIFO_SAMPLES
    parameter_t diff;
    parameter_t fifo_ovr;
    parameter_t fifo_fth;
    //================================================TAP_THS_X
    parameter_t tap_thsx;
    parameter_t ths_6d;
    parameter_t en_4d;
    //================================================TAP_THS_Y
    parameter_t tap_thsy;
    parameter_t tap_prior;
    //================================================TAP_THS_Z
    parameter_t tap_thsz;
    parameter_t tap_z_en;
    parameter_t tap_y_en;
    parameter_t tap_x_en;
    //================================================INT_DUR
    parameter_t shock;
    parameter_t quiet;
    parameter_t latency;
    //================================================WAKE_UP_THS
    parameter_t wk_ths;
    parameter_t sleep_on;
    parameter_t single_double_tap;
    //================================================WAKE_UP_DUR
    parameter_t sleep_dur;
    parameter_t stationary;
    parameter_t wake_dur;
    parameter_t ff_dur_5;
    //===============================================FREE_FALL
    parameter_t ff_dur_04;
    parameter_t ff_ths;
    //================================================STATUS_DUP
    parameter_t drdy_dup;
    parameter_t ff_ia_dup;
    parameter_t ia_6d_dup;
    parameter_t single_tap_dup;
    parameter_t double_tap_dup;
    parameter_t sleep_state_ia_dup;
    parameter_t drdy_t_dup;
    parameter_t ovr_dup;
    //================================================WAKE_UP_SRC
    parameter_t z_wu;
    parameter_t y_wu;
    parameter_t x_wu;
    parameter_t wu_ia;
    parameter_t sleep_state_ia;
    parameter_t ff_ia;
    //===============================================TAP_SRC
    parameter_t z_tap;
    parameter_t y_tap;
    parameter_t x_tap;
    parameter_t tap_sign;
    parameter_t double_tap_src;
    parameter_t single_tap_src;
    parameter_t tap_ia;
    //=================================================SIXD_SRC
    parameter_t xl;
    parameter_t xh;
    parameter_t yl;
    parameter_t yh;
    parameter_t zl;
    parameter_t zh;
    parameter_t ia_6d_src;
    //==================================================ALL_INT_SRC
    parameter_t ff_ia_src;
    parameter_t wu_ia_src;
    parameter_t single_tap_int_src;
    parameter_t double_tap_int_src;
    parameter_t ia_6d_int_src;
    parameter_t sleep_chng_ia_src;
    //==================================================X_OFS_USR
    parameter_t x_ofs_usr;
    //==================================================Y_OFS_USR
    parameter_t y_ofs_usr;
    //==================================================Z_OFS_USR
    parameter_t z_ofs_usr;
    //==================================================CTRL7
    parameter_t lpass_on6d;
    parameter_t hp_ref_mode;
    parameter_t usr_off_w;
    parameter_t usr_off_on_wu;
    parameter_t usr_off_on_out;
    parameter_t interrupts_en;
    parameter_t int2_on_int1;
    parameter_t drdy_pulsed; //DRDY pin mode

    accel_error_t (*data_write)(uint8_t  start_add,
                                uint8_t  len,
                                uint8_t *source);
    accel_error_t (*data_read)(uint8_t start_add, uint8_t len, uint8_t *dest);

} accel_t;
//==============================================================SET functions
/**
 * @brief Initializes accelerometer
 * @param *haccel - pointer to accel-object
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_init(accel_t *haccel);

/**
 * @brief Sets power mode of accel-object
 * @param *haccel - pointer on accel-object
 * @param mode - power mode of accel-obj
 * @param lpmode - low power mode
 * @param odr - output data rate
 * @param ln - low noise bit
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 */
accel_error_t accel_power_mode_set(accel_t    *haccel,
                                   pmode_t     mode,
                                   lpmode_t    lpmode,
                                   odr_t       odr,
                                   low_noise_t ln); //power mode set

/**
 * @brief Sets the filter mode of accel-obj
 * @param *haccel - pointer on accel-object
 * @param bw - bandwidth filter mode (BW_FILT[1:0])
 * @param fds - FDS-bit
 * @param fs - full-scale value
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 */
accel_error_t
accel_filter_set(accel_t *haccel, uint8_t bw, uint8_t fds, full_scale_t fs);

/**
 * @brief Sets interrupt 1 configuration bits
 * @param *haccel - pointer on accel-object
 * @param int1 - byte consist of setting bits
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_int1_set(accel_t *haccel, interrupt_t int1);

/**
 * @brief Sets interrupt 2 configuration bits
 * @param *haccel - pointer on accel-object
 * @param int2 - byte consist of setting bits
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_int2_set(accel_t *haccel, interrupt_t int2);

/**
 * @brief Disables all interrupts
 * @param *haccel - pointer on accel-object
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_int_disable(accel_t *haccel);

/**
 * @brief Sets the threshold for free-fall detection
 * @param *haccel - pointer on accel-object
 * @param threshold - threshold value
 * @param duration - Duration time is measured in N/ODR, where N is the content of the FF_DUR[5:0] field of the FREE_FALL /WAKE_UP_DUR registers and ODR is the accelerometer data rate.
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_free_fall_set(accel_t      *haccel,
                                  ffthreshold_t thrashoold,
                                  uint8_t       duration);

/**
 * @brief Sets threshold and duration for wake-up event detection
 * @param *haccel - pointer on accel-object
 * @param threshold - threshold value of 1 LSB of these 6 bits depends on the selected accelerometer full scale: 1 LSB = FS/64
 * @param duration - duration value  corresponds to 1*ODR time
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t
accel_wake_up_set(accel_t *haccel, uint8_t thrashold, uint8_t duration);

/**
 * @brief Sets user offsets for every zxis
 * @param *haccel - pointer on accel-object
 * @param x_offset - x offset
 * @param y_offset - y offset
 * @param z_offset - z offset
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_offset_set(accel_t *haccel,
                               uint8_t  x_offset,
                               uint8_t  y_offset,
                               uint8_t  z_offset);

/**
 * @brief Sets threshold for 6D-orientation change
 * @param *haccel - pointer on accel-object
 * @param threshold - value 0 - 3
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_orientation_param_set(accel_t   *haccel,
                                          sixd_thr_t threshold);

/**
 * @brief Sets an interrupt mode (pulse or latched)
 * @param *haccel - pointer on accel-object
 * @param mode - value pulse or latched
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_int_mode_set(accel_t *haccel, drdy_pin_mode_t mode);

/**
 * @brief Enables or disables 4d-direction detection
 * @param *haccel - pointer on accel-object
 * @param mode: 1-enable / 0 - disable
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_4d_set(accel_t *haccel, uint8_t en_dis);

/**@brief Sets mode of recognition single/double tap events
 * @param *haccel - pointer on accel-object
 * @param *dir - pointer to structure with enable/disable bits for tap recognition on direction X,Y,Z
 * @param *tap_threshold - pointer to structure with thresholds on directions X,Y,Z:   1 LSB = FS/32
 * @param tap_window - time for recognition tap event: the default value of these bits is 00 and corresponds to 4/ODR time. If the SHOCK[1:0] bits are set to a different value, 1 LSB corresponds to 8/ODR time
 * @param priority - TAP priority Z-Y-X
 * @param quiet - period of quiet time window defines the time after the first tap recognition in which there must not be any overthreshold. the default value of these bits is 00 and corresponds to 2/ODR time, where ODR is the accelerometer output data rate. If the QUIET[1:0] bits are set to a different value, 1 LSB corresponds to 4/ODR time.
 * @param latency - maximum time between two consecutive detected taps
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_double_tap_set(accel_t   *haccel,
                                   tap_dir_t *dir,
                                   tap_thr_t *tap_threshold,
                                   uint8_t    tap_window,
                                   uint8_t    priority,
                                   uint8_t    quiet,
                                   uint8_t    latency);

accel_error_t accel_single_tap_set(accel_t   *haccel,
                                   tap_dir_t *dir,
                                   tap_thr_t *tap_threshold,
                                   uint8_t    tap_window,
                                   uint8_t    priority,
                                   uint8_t    quiet);

/**@brief Sets parameters of activity/inactivity transfer
 * @param *haccel - pointer on accel-object
 * @param sleep_on_off - activate/deactivate sleep detection
 * @param  wu_duration - Period of activity
 * @param inact_duration - Period of inactivity
 * @param  threshold - threshold
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 *
 * */
accel_error_t accel_activity_recognition_set(accel_t *haccel,
                                             uint8_t  sleep_on_off,
                                             uint8_t  wu_duration,
                                             uint8_t  inact_duration,
                                             uint8_t  threshold);

/**@brief Detects orientation in space
 * @param *haccel - pointer on accel-object
 * @param *flag_orient - sixd_ia - is set high when the device switches from one orientation to another
 * @param *orient - pointer  to array with orientation:
 *                - ZH (YH, XH) is set high when the face perpendicular to the Z (Y,X) axis is almost flat and the acceleration measured on the Z (Y,X) axis is positive and in the absolute value bigger than the threshold
 *                - ZL (YL, XL) is set high when the face perpendicular to the Z (Y,X) axis is almost flat and the acceleration measured on the Z (Y,X) axis is negative and in the absolute value bigger than the threshold
 *
 * */
accel_error_t accel_sixd_oriantation_detection(accel_t *haccel,
                                               uint8_t *orient,
                                               uint8_t *flag_orient);

/**
 * @brief Sets mode for fifo buffer
 * @param *haccel - pointer on accel-object
 * @param fifo_mode - one of fifo_mode_t parameters
 * @param fifo_thr - water level fifo buffer
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t
accel_fifo_set(accel_t *haccel, fifo_mode_t fifo_mode, uint8_t fifo_thr);

/**
 * @brief Enable or disable the autoincrement of register address
 * @param *haccel - pointer on accel-object
 * @param ainc - bit value 0-disable / 1 - enable
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_autoincrement_set(accel_t *haccel, uint8_t ainc);

/**
 * @brief Sets interface of accel-object
 * @param *haccel - pointer on accel-object
 * @param ii - interface (SPI4/SPI3/I2C)
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_interface_set(accel_t *haccel, intarface_t ii);

/**
 * @brief Enable/disable block data update
 * @param *haccel - pointer on accel-object
 * @param bdu-bit - 0-disable/1-enable
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_bdu_set(accel_t *haccel, uint8_t bdu_bit);

/**
 * @brief Sets sleep mode
 * @param *haccel - pointer on accel-object
 * @param md_sel - sleep mode
 * @param md_1 - 0-disable/1-enable
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_slp_mode_set(accel_t *haccel, uint8_t md_sel, uint8_t md_1);

//========================================================================================GET functions
/**
 * @brief Gets accelerometer data
 * @param *haccel - pointer on accel-object
 * @param *adata - array of 3-axis data
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_data_get(accel_t *haccel, float *adata);

/**
 * @brief Returns 1 if new data is ready for reading
 * */
uint8_t accel_data_ready_get(accel_t *haccel);

/**
 * @brief Gets event sources and place them to byte-array
 * @param *haccel - pointer on accel-object
 * @param *int_sources - byte array with interrupt sources:
 * • STATUS_DUP (37h)
 * • WAKE_UP_SRC (38h)
 * • TAP_SRC (39h)
 * • SIXD_SRC (3Ah)
 * • ALL_INT_SRC (3Bh)
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 */
accel_error_t accel_event_status_get(accel_t *haccel, uint8_t *int_sources);

/** *
 * @brief Gets state of current orientation
 * @param *haccel - pointer on accel-object
 * @param *orient_byte - byte equal to SIXD_SRC register:
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 */
accel_error_t accel_sixd_orientation_get(accel_t *haccel, uint8_t *orient_byte);

/**
 * @brief Gets source of tab event and write it to tap_src_byte
 * @param *haccel - pointer on accel-object
 * @param *tap_src_byte - pointer to byte with tap source bits
 * @retval ACCEL_OK = 0
 * @retval ACCEL_INIT_FALSE = 1
 * @retval ACCEL_COMM_FALSE = 2
 * @retval ACCEL_POWER_FALSE = 3
 * */
accel_error_t accel_tap_get(accel_t *haccel, uint8_t *tap_src_byte);

/**
 * @brief Gets fifo state
 * @param *haccel - pointer on accel-object
 * @param *fifo_qty - pointer to value-variable
 * @retval fifo_state_t value
 * */
fifo_state_t accel_fifo_state_get(accel_t *haccel, uint8_t *fifo_qty);

/**
 * @brief Gets ID of accel-obj
 * @param *haccel - pointer on accel-object
 * @retval ID
 * */
uint8_t accel_id_get(accel_t *haccel);

#endif /* INC_ACCEL_H_ */
