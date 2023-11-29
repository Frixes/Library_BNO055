

#include "bno055.h"

#define ACCEL_SCALE 100
#define MAG_SCALE 16
#define EULER_SCALE 16
#define GYRO_SCALE 16
#define QUA_SCALE 16384


static I2C_HandleTypeDef* i2cHandler;

void BNO055_setup(I2C_HandleTypeDef* i2c) {

	i2cHandler = i2c;
	
	BNO055_WriteRegister(BNO055_SYS_TRIGGER, 0x0);
	
	BNO055_WriteRegister(BNO055_UNIT_SEL, 0x00);
	BNO055_WriteRegister(BNO055_OPR_MODE, BNO055_OPERATION_MODE_CONFIG); 
	HAL_Delay(100);
}

uint8_t BNO055_getCalibrationStatus() {
	uint8_t data;
	BNO055_ReadRegister(BNO055_CALIB_STAT, &data);
	
	return data;
}

uint8_t BNO055_getSelfTestStatus() {
	uint8_t data;
	BNO055_ReadRegister(BNO055_ST_RESULT, &data);
	
	return data;
}

uint8_t BNO055_getSystemStatus() {
	uint8_t data;
	BNO055_ReadRegister(BNO055_SYS_STATUS, &data);
	
	return data;
}

uint8_t BNO055_getSystemErrorStatus() {
	uint8_t data;
	BNO055_ReadRegister(BNO055_SYS_ERR, &data);
	
	return data;
}

BNO055_vector BNO055_getVector(uint8_t vec) {

	 uint8_t regData[8];
	
	if(vec == BNO055_QUA_DATA_W_LSB) {
	
		BNO055_ReadRegisters(vec, regData, 8); 
		
	} else {
	
		BNO055_ReadRegisters(vec, regData, 6);
		
	}
	
	double scale = 0;
	
	if(vec == BNO055_ACC_DATA_X_LSB) {
		scale = ACCEL_SCALE;
	} else if(vec == BNO055_MAG_DATA_X_LSB) {
		scale = MAG_SCALE;
	} else if(vec == BNO055_GYR_DATA_X_LSB) {
		scale = GYRO_SCALE;
	} else if(vec == BNO055_EUL_DATA_X_LSB) {
		scale = EULER_SCALE;
	} else if(vec == BNO055_LIA_DATA_X_LSB) {
		scale = ACCEL_SCALE;
	} else if(vec == BNO055_QUA_DATA_W_LSB) {
		scale = QUA_SCALE;
	} else if(vec == BNO055_GRV_DATA_X_LSB) {
		scale = ACCEL_SCALE;
	}
	
	BNO055_vector xyz = {.w = 0};
	
	if(vec == BNO055_QUA_DATA_W_LSB) {
	
		xyz.w = (int16_t) ((regData[1] << 8) | regData[0]) / scale;
		xyz.x = (int16_t) ((regData[3] << 8) | regData[2]) / scale;
		xyz.y = (int16_t) ((regData[5] << 8) | regData[4]) / scale;
		xyz.z = (int16_t) ((regData[7] << 8) | regData[6]) / scale;
		
	} else if(vec == BNO055_EUL_DATA_X_LSB){
	
		xyz.yaw = (int16_t) ((regData[1] << 8) | regData[0]) / scale;
		xyz.pitch = (int16_t) ((regData[3] << 8) | regData[2]) / scale;
		xyz.roll = (int16_t) ((regData[5] << 8) | regData[4]) / scale;
		
	} else {
	
		xyz.x = (int16_t) ((regData[1] << 8) | regData[0]) / scale;
		xyz.y = (int16_t) ((regData[3] << 8) | regData[2]) / scale;
		xyz.z = (int16_t) ((regData[5] << 8) | regData[4]) / scale;
		
	}

	return xyz;
	
}

BNO055_calibration BNO055_getVectorEulerCalibration() {
		BNO055_calibration cal;
		BNO055_vector vec;
	float rollCal, pitchCal, yawCal;
		
	for(int i = 0; i < 2000; i++){
		vec = BNO055_getVectorEuler();
		rollCal += vec.roll;
		pitchCal += vec.pitch;
		yawCal += vec.yaw;

	}
	
	cal.rollCalibration = rollCal / 2000;
	cal.pitchCalibration = pitchCal / 2000;
	cal.yawCalibration = yawCal / 2000;
	
	return cal;
}


BNO055_vector BNO055_getVectorAccelerometer() {
//	BNO055_WriteRegister(BNO055_OPR_MODE, BNO055_OPERATION_MODE_ACCONLY); 
	return BNO055_getVector(BNO055_ACC_DATA_X_LSB);
	
}

BNO055_vector BNO055_getVectorMagnetometer() {
//	BNO055_WriteRegister(BNO055_OPR_MODE, BNO055_OPERATION_MODE_MAGONLY);
	return BNO055_getVector(BNO055_MAG_DATA_X_LSB);
	
}

BNO055_vector BNO055_getVectorGyroscope() {
//	BNO055_WriteRegister(BNO055_OPR_MODE, BNO055_OPERATION_MODE_GYRONLY);
	return BNO055_getVector(BNO055_GYR_DATA_X_LSB);
	
}

BNO055_vector BNO055_getVectorEuler() {

	return BNO055_getVector(BNO055_EUL_DATA_X_LSB);
	
}

BNO055_vector BNO055_getVectorLinearAccel() {

	return BNO055_getVector(BNO055_LIA_DATA_X_LSB);

}

BNO055_vector BNO055_getVectorQuaternion() {

	return BNO055_getVector(BNO055_QUA_DATA_W_LSB);

}

BNO055_vector BNO055_getVectorGravity() {

	return BNO055_getVector(BNO055_GRV_DATA_X_LSB);

}


void BNO055_ReadRegister( uint8_t reg, uint8_t* data) {
	
	HAL_I2C_Mem_Read( i2cHandler, BNO055_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read_DMA( i2cHandler, BNO055_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1);

}

void BNO055_ReadRegisters( uint8_t reg, uint8_t* data, uint8_t lenght) {

	HAL_I2C_Mem_Read( i2cHandler, BNO055_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read_DMA( i2cHandler, BNO055_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, lenght);
	
	
}

void BNO055_WriteRegister( uint8_t reg, uint8_t data) {

	HAL_I2C_Mem_Write( i2cHandler, BNO055_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Write_DMA( i2cHandler, BNO055_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1);
	
}

