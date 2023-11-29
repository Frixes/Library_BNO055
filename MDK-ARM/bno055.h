
#ifndef BNO055_I2C_LIBRARY
#define BNO055_I2C_LIBRARY

#include "stm32f1xx_hal.h"



#define BNO055_I2C_ADDRESS 0x50

#define BNO055_CHIP_ID 	0x00
#define BNO055_ACC_ID		0x01
#define BNO055_MAG_ID 0x02
#define BNO055_GYR_ID 0x03
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_DATA_X_LSB 0x1A
#define BNO055_EUL_DATA_X_MSB 0x1B
#define BNO055_EUL_DATA_Y_LSB 0x1C
#define BNO055_EUL_DATA_Y_MSB 0x1D
#define BNO055_EUL_DATA_Z_LSB 0x1E
#define BNO055_EUL_DATA_Z_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D
#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33


#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B



#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
typedef struct {

	float w;
	float x;
	float y;
	float z;
	float roll, pitch, yaw;

	
} BNO055_vector;

typedef struct {
	float rollCalibration, pitchCalibration, yawCalibration;
} BNO055_calibration;

typedef enum {  // BNO-55 operation modes
  BNO055_OPERATION_MODE_CONFIG = 0x00,
  // Sensor Mode
  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYRONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_ACCGYRO,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_AMG,  // 0x07
                              // Fusion Mode
  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF  // 0x0C
} BNO055_operationMode;


void BNO055_setup(I2C_HandleTypeDef* i2c);

BNO055_vector BNO055_getVector(uint8_t vec);

uint8_t BNO055_getCalibrationStatus();
uint8_t BNO055_getSelfTestStatus();
uint8_t BNO055_getSystemStatus();
uint8_t BNO055_getSystemErrorStatus();

BNO055_calibration BNO055_getVectorEulerCalibration();


BNO055_vector BNO055_getVectorAccelerometer();
BNO055_vector BNO055_getVectorMagnetometer();
BNO055_vector BNO055_getVectorGyroscope();
BNO055_vector BNO055_getVectorEuler();
BNO055_vector BNO055_getVectorLinearAccel();
BNO055_vector BNO055_getVectorQuaternion();
BNO055_vector BNO055_getVectorGravity();


void BNO055_ReadRegister( uint8_t reg, uint8_t *data);
void BNO055_ReadRegisters( uint8_t reg, uint8_t *data, uint8_t lenght);
void BNO055_WriteRegister( uint8_t reg, uint8_t data);

#endif