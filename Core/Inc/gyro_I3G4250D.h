/*
 * @file gyro_I3G4250D.h
 * @brief MEMS motion sensor driver: 3-axis digital output gyroscope SPI/I2C
 *
 *  @Created on: May 26, 2025
 *  @Author: Andrii V.
 *  @copyright (C) 2025 Alnicko Development OU. All rights reserved.
 */

#ifndef INC_GYRO_I3G4250D_H_
#define INC_GYRO_I3G4250D_H_

#include <stdint.h>

/*Statuses of several settings*/
enum is_enable { DISABLE = 0, ENABLE = 1 };

/*Errors*/
typedef enum {
  gyroOk = 0,
  gyroCommError = 1,
  gyroUnknownStatus = 2,
  gyroNotAnswer = 3,
  gyroInitError = 4
} gyroError_t;

typedef enum {
  gyroModeBypass = 0,
  gyroModeFifo = 1,
  gyroModeStream = 2
} gyroMode_t;

typedef enum d_rate {
  ODR100 = 100,
  ODR200 = 200,
  ODR400 = 400,
  ODR800 = 800
} drate_t;

typedef enum hp_cf {
  HPCF0,
  HPCF1,
  HPCF2,
  HPCF3,
  HPCF4,
  HPCF5,
  HPCF6,
  HPCF7,
  HPCF8,
  HPCF9
} gyroHpcf_t;

typedef enum hp_mode {
  HPNormal = 0, // Normal mode (reset by reading the REFERENCE/DATACAPTURE (25h)
                // register)
  HPRef = 1,    // Reference signal for filtering
  HPNorma2 = 2, // Normal mode (reset by reading the REFERENCE/DATACAPTURE (25h)
                // register)
  HPAutoReset = 3 // Autoreset on interrupt event
} gyroHPMode_t;

typedef enum { NORMAL, SLEEP, DOWN } gyroPowMode_t;

typedef enum { FS245, FS500, FS2000 } fscale_t;

#define ADD_WHO_AM_I 0x0F
#define ADD_CTRL_REG1 0x20
#define ADD_CTRL_REG2 0x21
#define ADD_CTRL_REG3 0x22
#define ADD_CTRL_REG4 0x23
#define ADD_CTRL_REG5 0x24
#define ADD_REFERENCE 0x25
#define ADD_OUT_TEMP 0x26
#define ADD_STATUS_REG 0x27
#define ADD_OUT_X_L 0x28
#define ADD_OUT_X_H 0x29
#define ADD_OUT_Y_L 0x2A
#define ADD_OUT_Y_H 0x2B
#define ADD_OUT_Z_L 0x2C
#define ADD_OUT_Z_H 0x2D
#define ADD_FIFO_CTRL_REG 0x2E
#define ADD_FIFO_SRC_REG 0x2F
#define ADD_INT1_CFG 0x30
#define ADD_INT1_SRC 0x31
#define ADD_INT1_THS_XH 0x32
#define ADD_INT1_THS_XL 0x33
#define ADD_INT1_THS_YH 0x34
#define ADD_INT1_THS_YL 0x35
#define ADD_INT1_THS_ZH 0x36
#define ADD_INT1_THS_ZL 0x37
#define ADD_INT1_DURATION 0x38

typedef struct int_flg {
  /*INT1_SRC (31h)*/
  uint8_t
      IntFlag; // Interrupt active. Default value: 0 (0: no interrupt has been
               // generated; 1: one or more interrupts have been generated)
  uint8_t ZhighFlag; // Z high. Default value: 0 (0: no interrupt, 1: Z high
                     // event has occurred)
  uint8_t ZLowFlag; // Z low. Default value: 0 (0: no interrupt; 1: Z low event
                    // has occurred)
  uint8_t YhighFlag; // Y high. Default value: 0 (0: no interrupt, 1: Y high
                     // event has occurred)
  uint8_t YlowFlag; // Y low. Default value: 0 (0: no interrupt, 1: Y low event
                    // has occurred)
  uint8_t XhighFlag; // X high. Default value: 0 (0: no interrupt, 1: X high
                     // event has occurred)
  uint8_t XlowFlag; // X low. Default value: 0 (0: no interrupt, 1: X low event
                    // has occurred)
} gyroIntFlag_t;

typedef struct gyro_param {
  /*Function prototypes for read and write registers*/
  gyroError_t (*funcReadRegs)(uint16_t startAdd, uint16_t len, uint8_t *store);
  gyroError_t (*funcWriteRegs)(uint16_t startAdd, uint16_t len, uint8_t *store);
  /*-Calculating values-*/
  drate_t Odr; // Digital output data rate (105/208/420/840)
  gyroPowMode_t
      PowerMode;     // power mode selection (power-down / normal /sleep mode)
  uint16_t LPCutOff; // table 21
  uint16_t HPCutOff; // table 26
  /*WHO_AM_I (0Fh)*/
  uint8_t Id;
  /*CTRL_REG1 (20h)*/
  uint8_t DataRate;    // Output data rate selection. Refer to Table 21.
  uint8_t BandWidth;   // Bandwidth selection. Refer to Table 21.
  uint8_t PDownModeEn; // Enables power-down mode. Default value: 0
  uint8_t Zen;         // Enables Z-axis. Default value: 1
  uint8_t Yen;         // Enables Y-axis. Default value: 1
  uint8_t Xen;         // Enables X-axis. Default value: 1
  /*CTRL_REG2 (21h)*/
  uint8_t HPFilterMode; // High-pass filter mode selection. Default value: 00
  uint8_t HPCutOffCode; // High-pass filter cutoff frequency selection
  /*CTRL_REG3 (22h)*/
  uint8_t Int1En;       // Enables interrupt on the INT1 pin. Default value 0.
  uint8_t Int1Boot;     // Boot status available on INT1. Default value 0.
  uint8_t IntActConfig; // Interrupt active configuration on INT1. Default value
                        // 0. (0: high; 1: low)
  uint8_t IOutType; // Push-pull / open drain. Default value: 0. (0: push-pull;
                    // 1: open drain)
  uint8_t Int2En; // Data ready on DRDY/INT2. Default value 0. (0: disable; 1:
                  // enable)
  uint8_t FIFOWtmEn; // FIFO watermark interrupt on DRDY/INT2. Default value: 0.
                     // (0: disable; 1: enable)
  uint8_t FIFOOvrnEn; // FIFO overrun interrupt on DRDY/INT2 Default value: 0.
                      // (0: disable; 1: enable)
  uint8_t FIFOEmptyEn; // FIFO empty interrupt on DRDY/INT2. Default value: 0.
                       // (0: disable; 1: enable)
  /*CTRL_REG4 (23h)*/
  uint8_t Ble; // Big/little endian data selection. Default value 0. (0: data
               // LSB @ lower address; 1: data MSB @ lower address)
  fscale_t FullScale; // Full-scale selection. Default value: 00 (00: ±245 dps;
                      // 01: ±500 dps; 10: ±2000 dps; 11: ±2000 dps)
  uint8_t SelfTestEn; // Enables self-test. Default value: 00 (00: self-test
                      // disabled; other: see Table 31
  uint8_t SPIModeSel; // SPI serial interface mode selection. Default value: 0
                      // (0: 4-wire interface; 1: 3-wire interface)
  /*CTRL_REG5 (24h)*/
  uint8_t Boot; // Reboot memory content. Default value: 0 (0: normal mode; 1:
                // reboot memory content)
  uint8_t FifoEn; // Enable FIFO. Default value: 0 (0: FIFO disabled; 1: FIFO
                  // enabled)
  uint8_t HPFilterEn; // Enable high-pass filter (see Figure 18). Default value:
                      // 0 (0: HPF disabled; 1: HPF enabled
  uint8_t Int1SelConf; // INT1 selection configuration (see Figure 18, Table
                       // 35). Default value: 0
  uint8_t OutSelConf; // Out selection configuration (see Figure 18, Table 34).
                      // Default value: 0
  /*REFERENCE/DATACAPTURE (25h)*/
  uint16_t RefData;
  /*OUT_TEMP (26h)*/
  uint16_t OutTemp;
  /*STATUS_REG (27h)*/
  uint8_t zyxor; // X-, Y-, Z-axis data overrun. Default value: 0 (0: no overrun
                 // has occurred; 1: new data has overwritten the previous data
                 // before it was read)
  uint8_t
      zor; // Z-axis data overrun. Default value: 0 (0: no overrun has occurred;
           // 1: new data for the Z-axis has overwritten the previous data)
  uint8_t
      yor; // Y-axis data overrun. Default value: 0 (0: no overrun has occurred;
           // 1: a new data for the Y-axis has overwritten the previous data)
  uint8_t xor
      ; // X-axis data overrun. Default value: 0 (0: no overrun has occurred; 1:
        // a new data for the X-axis has overwritten the previous data)
  uint8_t
      zyxda; // X, Y, Z-axis new data available. Default value: 0 (0: a new set
             // of data is not yet available; 1: a new set of data is available)
  uint8_t zda; // Z-axis new data available. Default value: 0 (0: new data for
               // the Z-axis is not yet available; 1: new data for the Z-axis is
               // available)
  uint8_t yda; // Y-axis new data available. Default value: 0 (0: new data for
               // the Y-axis is not yet available; 1: new data for the Y-axis is
               // available)
  uint8_t xda; // X-axis new data available. Default value: 0 (0: new data for
               // the X-axis is not yet available; 1: new data for the X-axis is
               // available)
  /*OUT_X_L (28h), OUT_X_H (29h)*/
  uint16_t OutX; // X-axis angular rate data. The value is expressed as two's
                 // complement
  /*OUT_Y_L (2Ah), OUT_Y_H (2Bh)*/
  uint16_t OutY; // Y-axis angular rate data. The value is expressed as two's
                 // complement.
  /*OUT_Z_L (2Ch), OUT_Z_H (2Dh)*/
  uint16_t OutZ; // Z-axis angular rate data. The value is expressed as two's
                 // complement.
  /*FIFO_CTRL_REG (2Eh)*/
  uint8_t FifoModeSel;   // FIFO mode selection. Default value: 000
  uint8_t FifoThreshold; // FIFO threshold. Watermark level setting.
  /*FIFO_SRC_REG (2Fh)*/
  uint8_t FifoWtmStatus; // Watermark status. (0: FIFO filling is lower than WTM
                         // level; 1: FIFO filling is equal to or higher than
                         // WTM level)
  uint8_t FifoOvrnStatus; // Overrun bit status. (0: FIFO is not completely
                          // filled; 1: FIFO is completely filled)
  uint8_t FifoEmpty;  // FIFO empty bit. (0: FIFO not empty; 1: FIFO empty)
  uint8_t FifoStored; // FIFO stored data level
  /*INT1_CFG (30h)*/
  uint8_t Andor; // AND/OR combination of interrupt events. Default value: 0 (0:
                 // OR combination of interrupt events 1: AND combination of
                 // interrupt events
  uint8_t Lir; // Latch interrupt request. Default value: 0 (0: interrupt
               // request not latched; 1: interrupt request latched) Cleared by
               // reading INT1_SRC (31h).
  uint8_t ZHIE; // Enables interrupt generation on Z high event. Default value:
                // 0 (0: disable interrupt request; 1: enable interrupt request
                // on measured rate value higher than preset threshold)
  uint8_t ZLIE; // Enables interrupt generation on Z low event. Default value: 0
                // (0: disable interrupt request; 1: enable interrupt request on
                // measured rate value lower than preset threshold)
  uint8_t YHIE; // Enables interrupt generation on Y high event. Default value:
                // 0 (0: disable interrupt request; 1: enable interrupt request
                // on measured rate value higher than preset threshold)
  uint8_t YLIE; // Enables interrupt generation on Y low event. Default value: 0
                // (0: disable interrupt request; 1: enable interrupt request on
                // measured rate value lower than preset threshold)
  uint8_t XHIE; // Enables interrupt generation on X high event. Default value:
                // 0 (0: disable interrupt request; 1: enable interrupt request
                // on measured rate value higher than preset threshold)
  uint8_t XLIE; // Enables interrupt generation on X low event. Default value: 0
                // (0: disable interrupt request; 1: enable interrupt request on
                // measured rate value lower than preset threshold)
  /*INT1_SRC (31h)*/
  gyroIntFlag_t *InterruptFlags;
  /*INT1_THS_XH (32h)*/
  uint8_t Int1XHighTr; // Interrupt threshold X HIGH. Default value: 0000 0000
  /*INT1_THS_XL (33h)*/
  uint8_t Int1XLowTr; // Interrupt threshold X LOW. Default value: 0000 0000
  /*INT1_THS_YH (34h)*/
  uint8_t Int1YHighTr; // Interrupt threshold Y HIGH. Default value: 0000 0000
  /*INT1_THS_YL (35h)*/
  uint8_t Int1YLowTr; // Interrupt threshold Y LOW. Default value: 0000 0000
  /*INT1_THS_ZH (36h)*/
  uint8_t Int1ZHighTr; // Interrupt threshold Z HIGH. Default value: 0000 0000
  /*INT1_THS_ZL (37h)*/
  uint8_t Int1ZLowTr; // Interrupt threshold Z LOW. Default value: 0000 0000
  /*INT1_DURATION (38h)*/
  uint8_t Int1Wait; // Enables WAIT bit. Default value: 0 (0: disable; 1:
                    // enable)
  uint8_t Int1Duration; // Duration value. Default value: 000 0000
} gyro_t;

/*@brief Initialize Gyro-object - motion sensor type I3G4250D
 * Creates defined connection type, tries to read WHO_AM_I register,
 * stores the value in appropriate fields of hGyro.
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroInit(gyro_t *hGyro);

/*@brief Deinitialize Gyro-object - motion sensor
 * Releases connection for other devices
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroDeinit(gyro_t *hGyro);

/*@brief Turns off hGyro-object
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroPowerDown(gyro_t *hGyro);

/*@brief Turns hGyro-object to sleep mode
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroSleep(gyro_t *hGyro);

/*@brief Turns on hGyro-object
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroTurnOn(gyro_t *hGyro);

/*@brief Reads current yaw, pitch, and roll data.
 * hGyro - pointer to structure defined Gyro-object
 * gVal  - pointer to array type uint16_t[3] = X, Y, Z
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroReadVal(gyro_t *hGyro, float *gVal);

/*@brief Reads current state of status register of the Gyro-object
 * and stores it in appropriate fields of hGyro
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroReadStatus(gyro_t *hGyro);

/*@brief Reads value of FIFO_SRC_REG register and stores data
 * in appropriate fields of hGyro
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroReadFifoStatus(gyro_t *hGyro);

/*@brief Defines the source of interrupt and sets appropriate flag
 * intFlag - pointer to structure of interruption flags
 * Returns gyroOk (= 0) if parameter written correctly or error code
 */
gyroError_t gyroReadIntStatus(gyro_t *hGyro);

/*@brief Sets the Gyro-object data collecting mode
 * hGyro - pointer to structure defined Gyro-object
 * mode  - bypass
 *       - FIFO
 *       - stream
 * wtm   - FIFO Watermark level.
 * Returns gyroOk (= 0) if parameter written correctly
 * or error code
 * */
gyroError_t gyroSetFifoMode(gyro_t *hGyro, gyroMode_t mode, uint16_t wtm);

/*@brief Sets interrupts on pins INT1 and INT2
 *  hGyro   - pointer to structure defined Gyro-object
 *  Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroSetIntMode(gyro_t *hGyro);

/*@brief Sets mode of filtering output data and selects which data
 * use for interrupt source - filtered or not *
 * hGyro   - pointer to structure defined Gyro-object
 * next parameters need to be set:
 * - Boot;
 * - FifoEn;
 * - HPFilterEn;
 * - Int1SelConf;
 * - OutSelConf;
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroSetOutMode(gyro_t *hGyro);

/*@brief Sets digital output data rate
 * hGyro - pointer to structure defined Gyro-object
 * dataRate - {100, 200, 400, 800} Hz
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroSetOutDataRate(gyro_t *hGyro, drate_t dataRate);

/*@brief Sets cutoff frequency for HighPath filter
 * hGyro - pointer to structure defined Gyro-object
 * mode  - High-pass filter mode (0, 2 - Normal mode, 1 - Reference signal for
 * filtering, 3 - Autoreset on interrupt event) freq  - cutoff frequency (0..9)
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroSetHPFilter(gyro_t *hGyro, gyroHPMode_t mode, gyroHpcf_t freq);

/*@brief Sets big/little endian data order
 *hGyro - pointer to structure defined Gyro-object
 *ble   - 0: data LSB @ lower address; 1: data MSB @ lower address
 *Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroSetBle(gyro_t *hGyro, uint8_t ble);

/*@brief sets full scale measurement
 * hGyro - pointer to structure defined Gyro-object
 * type  - (00: ±245 dps; 01: ±500 dps; 10: ±2000 dps; 11: ±2000 dps)
 * Returns gyroOk (= 0) if parameter written correctly or error code
 *  */
gyroError_t gyroSetFullScale(gyro_t *hGyro, fscale_t type);

/*@brief starts self test
 * hGyro - pointer to structure defined Gyro-object
 * conf  - self-test mode configuration 0 - Normal mode, 1 - Self-test 0 (+), 3
 * - Self-test 1 (-) Returns gyroOk (= 0) if parameter written correctly or
 * error code
 * */
gyroError_t gyroSelfTest(gyro_t *hGyro, uint8_t conf);

/*@brief Retoots hGyro object
 * hGyro - pointer to structure defined Gyro-object
 * Returns gyroOk (= 0) if parameter written correctly or error code
 * */
gyroError_t gyroReboot(gyro_t *hGyro);

#endif /* INC_GYRO_I3G4250D_H_ */
