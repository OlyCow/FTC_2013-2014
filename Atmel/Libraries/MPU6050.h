// Basic acquiring and processing of the data fetched from the MPU-6050.
// A lot of stuff is taken from Jeff Rowberg's library. <jeff@rowberg.net>
// For more documentation (and I mean *more*), see his website or repo.
#ifndef MPU6050_H
#define MPU6050_H

#include <avr/io.h>
#include "i2cmaster.h"
//#include "I2C.h"

#define MPU6050_ADDRESS_AD0_LOW		0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH	0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS		MPU6050_ADDRESS_AD0_LOW

#define MPU6050_ADDRESS				MPU6050_ADDRESS_AD0_LOW // NOTE: This definition is application specific.

// Copied from Jeff Rowberg's library.
// These are only the ones contained in the official registry map.
// TODO: Look at the DMP documentation (requires registration on the
// official Invensense page; still may not have documentation).
#define MPU6050_RA_SELF_TEST_X		0x0D
#define MPU6050_RA_SELF_TEST_Y		0x0E
#define MPU6050_RA_SELF_TEST_Z		0x0F
#define MPU6050_RA_SELF_TEST_A		0x10
#define MPU6050_RA_SMPLRT_DIV		0x19
#define MPU6050_RA_CONFIG			0x1A
#define MPU6050_RA_GYRO_CONFIG		0x1B
#define MPU6050_RA_ACCEL_CONFIG		0x1C
#define MPU6050_RA_FIFO_EN			0x23
#define MPU6050_RA_I2C_MST_CTRL		0x24
#define MPU6050_RA_I2C_SLV0_ADDR	0x25
#define MPU6050_RA_I2C_SLV0_REG		0x26
#define MPU6050_RA_I2C_SLV0_CTRL	0x27
#define MPU6050_RA_I2C_SLV1_ADDR	0x28
#define MPU6050_RA_I2C_SLV1_REG		0x29
#define MPU6050_RA_I2C_SLV1_CTRL	0x2A
#define MPU6050_RA_I2C_SLV2_ADDR	0x2B
#define MPU6050_RA_I2C_SLV2_REG		0x2C
#define MPU6050_RA_I2C_SLV2_CTRL	0x2D
#define MPU6050_RA_I2C_SLV3_ADDR	0x2E
#define MPU6050_RA_I2C_SLV3_REG		0x2F
#define MPU6050_RA_I2C_SLV3_CTRL	0x30
#define MPU6050_RA_I2C_SLV4_ADDR	0x31
#define MPU6050_RA_I2C_SLV4_REG		0x32
#define MPU6050_RA_I2C_SLV4_DO		0x33
#define MPU6050_RA_I2C_SLV4_CTRL	0x34
#define MPU6050_RA_I2C_SLV4_DI		0x35
#define MPU6050_RA_I2C_MST_STATUS	0x36
#define MPU6050_RA_INT_PIN_CFG		0x37
#define MPU6050_RA_INT_ENABLE		0x38
#define MPU6050_RA_INT_STATUS		0x3A
#define MPU6050_RA_ACCEL_XOUT_H		0x3B
#define MPU6050_RA_ACCEL_XOUT_L		0x3C
#define MPU6050_RA_ACCEL_YOUT_H		0x3D
#define MPU6050_RA_ACCEL_YOUT_L		0x3E
#define MPU6050_RA_ACCEL_ZOUT_H		0x3F
#define MPU6050_RA_ACCEL_ZOUT_L		0x40
#define MPU6050_RA_TEMP_OUT_H		0x41
#define MPU6050_RA_TEMP_OUT_L		0x42
#define MPU6050_RA_GYRO_XOUT_H		0x43
#define MPU6050_RA_GYRO_XOUT_L		0x44
#define MPU6050_RA_GYRO_YOUT_H		0x45
#define MPU6050_RA_GYRO_YOUT_L		0x46
#define MPU6050_RA_GYRO_ZOUT_H		0x47
#define MPU6050_RA_GYRO_ZOUT_L		0x48
#define MPU6050_RA_EXT_SENS_DATA_00	0x49
#define MPU6050_RA_EXT_SENS_DATA_01	0x4A
#define MPU6050_RA_EXT_SENS_DATA_02	0x4B
#define MPU6050_RA_EXT_SENS_DATA_03	0x4C
#define MPU6050_RA_EXT_SENS_DATA_04	0x4D
#define MPU6050_RA_EXT_SENS_DATA_05	0x4E
#define MPU6050_RA_EXT_SENS_DATA_06	0x4F
#define MPU6050_RA_EXT_SENS_DATA_07	0x50
#define MPU6050_RA_EXT_SENS_DATA_08	0x51
#define MPU6050_RA_EXT_SENS_DATA_09	0x52
#define MPU6050_RA_EXT_SENS_DATA_10	0x53
#define MPU6050_RA_EXT_SENS_DATA_11	0x54
#define MPU6050_RA_EXT_SENS_DATA_12	0x55
#define MPU6050_RA_EXT_SENS_DATA_13	0x56
#define MPU6050_RA_EXT_SENS_DATA_14	0x57
#define MPU6050_RA_EXT_SENS_DATA_15	0x58
#define MPU6050_RA_EXT_SENS_DATA_16	0x59
#define MPU6050_RA_EXT_SENS_DATA_17	0x5A
#define MPU6050_RA_EXT_SENS_DATA_18	0x5B
#define MPU6050_RA_EXT_SENS_DATA_19	0x5C
#define MPU6050_RA_EXT_SENS_DATA_20	0x5D
#define MPU6050_RA_EXT_SENS_DATA_21	0x5E
#define MPU6050_RA_EXT_SENS_DATA_22	0x5F
#define MPU6050_RA_EXT_SENS_DATA_23	0x60
#define MPU6050_RA_I2C_SLV0_DO		0x63
#define MPU6050_RA_I2C_SLV1_DO		0x64
#define MPU6050_RA_I2C_SLV2_DO		0x65
#define MPU6050_RA_I2C_SLV3_DO		0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL	0x67
#define MPU6050_RA_SIGNAL_PATH_RESET	0x68
#define MPU6050_RA_USER_CTRL		0x6A
#define MPU6050_RA_PWR_MGMT_1		0x6B
#define MPU6050_RA_PWR_MGMT_2		0x6C
#define MPU6050_RA_FIFO_COUNTH		0x72
#define MPU6050_RA_FIFO_COUNTL		0x73
#define MPU6050_RA_FIFO_R_W			0x74
#define MPU6050_RA_WHO_AM_I			0x75

namespace MPU
{
	// TODO: Put this into its own library (of math-y stuff? idk).
	int convert_complement(uint16_t input);

	void read(uint8_t address, uint8_t RA, uint8_t &data);
	void read_burst(uint8_t address, uint8_t RA, uint8_t* data, int size);
	void write(uint8_t address, uint8_t RA, uint8_t data);
	void write_burst(uint8_t address, uint8_t RA, uint8_t data[], int size);
	
	void initialize();
	
	//// SMPLRT_DIV register
	//uint8_t getRate();
	//void setRate(uint8_t rate);

	//// CONFIG register
	//uint8_t getExternalFrameSync();
	//void setExternalFrameSync(uint8_t sync);
	//uint8_t getDLPFMode();
	//void setDLPFMode(uint8_t bandwidth);
	//
	//// GYRO_CONFIG register
	//uint8_t getFullScaleGyroRange();
	//void setFullScaleGyroRange(uint8_t range);
	//
	//// ACCEL_CONFIG register
	//bool getAccelXSelfTest();
	//void setAccelXSelfTest(bool enabled);
	//bool getAccelYSelfTest();
	//void setAccelYSelfTest(bool enabled);
	//bool getAccelZSelfTest();
	//void setAccelZSelfTest(bool enabled);
	//uint8_t getFullScaleAccelRange();
	//void setFullScaleAccelRange(uint8_t range);
	//uint8_t getDHPFMode();
	//void setDHPFMode(uint8_t mode);
	//
	//// FF_THR register
	//uint8_t getFreefallDetectionThreshold();
	//void setFreefallDetectionThreshold(uint8_t threshold);
	//
	//// FF_DUR register
	//uint8_t getFreefallDetectionDuration();
	//void setFreefallDetectionDuration(uint8_t duration);
	//
	//// MOT_THR register
	//uint8_t getMotionDetectionThreshold();
	//void setMotionDetectionThreshold(uint8_t threshold);
	//
	//// MOT_DUR register
	//uint8_t getMotionDetectionDuration();
	//void setMotionDetectionDuration(uint8_t duration);
	//
	//// ZRMOT_THR register
	//uint8_t getZeroMotionDetectionThreshold();
	//void setZeroMotionDetectionThreshold(uint8_t threshold);
	//
	//// ZRMOT_DUR register
	//uint8_t getZeroMotionDetectionDuration();
	//void setZeroMotionDetectionDuration(uint8_t duration);
	//
	//// FIFO_EN register
	//bool getTempFIFOEnabled();
	//void setTempFIFOEnabled(bool enabled);
	//bool getXGyroFIFOEnabled();
	//void setXGyroFIFOEnabled(bool enabled);
	//bool getYGyroFIFOEnabled();
	//void setYGyroFIFOEnabled(bool enabled);
	//bool getZGyroFIFOEnabled();
	//void setZGyroFIFOEnabled(bool enabled);
	//bool getAccelFIFOEnabled();
	//void setAccelFIFOEnabled(bool enabled);
	//bool getSlave2FIFOEnabled();
	//void setSlave2FIFOEnabled(bool enabled);
	//bool getSlave1FIFOEnabled();
	//void setSlave1FIFOEnabled(bool enabled);
	//bool getSlave0FIFOEnabled();
	//void setSlave0FIFOEnabled(bool enabled);
	//
	//// I2C_MST_CTRL register
	//bool getMultiMasterEnabled();
	//void setMultiMasterEnabled(bool enabled);
	//bool getWaitForExternalSensorEnabled();
	//void setWaitForExternalSensorEnabled(bool enabled);
	//bool getSlave3FIFOEnabled();
	//void setSlave3FIFOEnabled(bool enabled);
	//bool getSlaveReadWriteTransitionEnabled();
	//void setSlaveReadWriteTransitionEnabled(bool enabled);
	//uint8_t getMasterClockSpeed();
	//void setMasterClockSpeed(uint8_t speed);
	//
	//// I2C_SLV registers (Slave 0-3)
	//uint8_t getSlaveAddress(uint8_t num);
	//void setSlaveAddress(uint8_t num, uint8_t address);
	//uint8_t getSlaveRegister(uint8_t num);
	//void setSlaveRegister(uint8_t num, uint8_t reg);
	//bool getSlaveEnabled(uint8_t num);
	//void setSlaveEnabled(uint8_t num, bool enabled);
	//bool getSlaveWordByteSwap(uint8_t num);
	//void setSlaveWordByteSwap(uint8_t num, bool enabled);
	//bool getSlaveWriteMode(uint8_t num);
	//void setSlaveWriteMode(uint8_t num, bool mode);
	//bool getSlaveWordGroupOffset(uint8_t num);
	//void setSlaveWordGroupOffset(uint8_t num, bool enabled);
	//uint8_t getSlaveDataLength(uint8_t num);
	//void setSlaveDataLength(uint8_t num, uint8_t length);
	//
	//// I2C_SLV registers (Slave 4)
	//uint8_t getSlave4Address();
	//void setSlave4Address(uint8_t address);
	//uint8_t getSlave4Register();
	//void setSlave4Register(uint8_t reg);
	//void setSlave4OutputByte(uint8_t data);
	//bool getSlave4Enabled();
	//void setSlave4Enabled(bool enabled);
	//bool getSlave4InterruptEnabled();
	//void setSlave4InterruptEnabled(bool enabled);
	//bool getSlave4WriteMode();
	//void setSlave4WriteMode(bool mode);
	//uint8_t getSlave4MasterDelay();
	//void setSlave4MasterDelay(uint8_t delay);
	//uint8_t getSlate4InputByte();
	//
	//// I2C_MST_STATUS register
	//bool getPassthroughStatus();
	//bool getSlave4IsDone();
	//bool getLostArbitration();
	//bool getSlave4Nack();
	//bool getSlave3Nack();
	//bool getSlave2Nack();
	//bool getSlave1Nack();
	//bool getSlave0Nack();
	//
	//// INT_PIN_CFG register
	//bool getInterruptMode();
	//void setInterruptMode(bool mode);
	//bool getInterruptDrive();
	//void setInterruptDrive(bool drive);
	//bool getInterruptLatch();
	//void setInterruptLatch(bool latch);
	//bool getInterruptLatchClear();
	//void setInterruptLatchClear(bool clear);
	//bool getFSyncInterruptLevel();
	//void setFSyncInterruptLevel(bool level);
	//bool getFSyncInterruptEnabled();
	//void setFSyncInterruptEnabled(bool enabled);
	//bool getI2CBypassEnabled();
	//void setI2CBypassEnabled(bool enabled);
	//bool getClockOutputEnabled();
	//void setClockOutputEnabled(bool enabled);
	//
	//// INT_ENABLE register
	//uint8_t getIntEnabled();
	//void setIntEnabled(uint8_t enabled);
	//bool getIntFreefallEnabled();
	//void setIntFreefallEnabled(bool enabled);
	//bool getIntMotionEnabled();
	//void setIntMotionEnabled(bool enabled);
	//bool getIntZeroMotionEnabled();
	//void setIntZeroMotionEnabled(bool enabled);
	//bool getIntFIFOBufferOverflowEnabled();
	//void setIntFIFOBufferOverflowEnabled(bool enabled);
	//bool getIntI2CMasterEnabled();
	//void setIntI2CMasterEnabled(bool enabled);
	//bool getIntDataReadyEnabled();
	//void setIntDataReadyEnabled(bool enabled);
	//
	//// INT_STATUS register
	//uint8_t getIntStatus();
	//bool getIntFreefallStatus();
	//bool getIntMotionStatus();
	//bool getIntZeroMotionStatus();
	//bool getIntFIFOBufferOverflowStatus();
	//bool getIntI2CMasterStatus();
	//bool getIntDataReadyStatus();
	//
	//// ACCEL_OUT_ registers
	//void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
	//void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
	//void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
	//int16_t getAccelerationX();
	//int16_t getAccelerationY();
	//int16_t getAccelerationZ();
	//
	//// TEMP_OUT_ registers
	//int16_t getTemperature();
	//
	//// GYRO_OUT_ registers
	//void getRotation(int16_t* x, int16_t* y, int16_t* z);
	//int16_t getRotationX();
	//int16_t getRotationY();
	//int16_t getRotationZ();
	//
	//// EXT_SENS_DATA_ registers
	//uint8_t getExternalSensorByte(int position);
	//uint16_t getExternalSensorWord(int position);
	//uint32_t getExternalSensorDWord(int position);
	//
	//// MOT_DETECT_STATUS register
	//bool getXNegMotionDetected();
	//bool getXPosMotionDetected();
	//bool getYNegMotionDetected();
	//bool getYPosMotionDetected();
	//bool getZNegMotionDetected();
	//bool getZPosMotionDetected();
	//bool getZeroMotionDetected();
	//
	//// I2C_SLV_DO register
	//void setSlaveOutputByte(uint8_t num, uint8_t data);
	//
	//// I2C_MST_DELAY_CTRL register
	//bool getExternalShadowDelayEnabled();
	//void setExternalShadowDelayEnabled(bool enabled);
	//bool getSlaveDelayEnabled(uint8_t num);
	//void setSlaveDelayEnabled(uint8_t num, bool enabled);
	//
	//// SIGNAL_PATH_RESET register
	//void resetGyroscopePath();
	//void resetAccelerometerPath();
	//void resetTemperaturePath();
	//
	//// MOT_DETECT_CTRL register
	//uint8_t getAccelerometerPowerOnDelay();
	//void setAccelerometerPowerOnDelay(uint8_t delay);
	//uint8_t getFreefallDetectionCounterDecrement();
	//void setFreefallDetectionCounterDecrement(uint8_t decrement);
	//uint8_t getMotionDetectionCounterDecrement();
	//void setMotionDetectionCounterDecrement(uint8_t decrement);
	//
	//// USER_CTRL register
	//bool getFIFOEnabled();
	//void setFIFOEnabled(bool enabled);
	//bool getI2CMasterModeEnabled();
	//void setI2CMasterModeEnabled(bool enabled);
	//void switchSPIEnabled(bool enabled);
	//void resetFIFO();
	//void resetI2CMaster();
	//void resetSensors();
	//
	//// PWR_MGMT_1 register
	//void reset();
	//bool getSleepEnabled();
	//void setSleepEnabled(bool enabled);
	//bool getWakeCycleEnabled();
	//void setWakeCycleEnabled(bool enabled);
	//bool getTempSensorEnabled();
	//void setTempSensorEnabled(bool enabled);
	//uint8_t getClockSource();
	//void setClockSource(uint8_t source);
	//
	//// PWR_MGMT_2 register
	//uint8_t getWakeFrequency();
	//void setWakeFrequency(uint8_t frequency);
	//bool getStandbyXAccelEnabled();
	//void setStandbyXAccelEnabled(bool enabled);
	//bool getStandbyYAccelEnabled();
	//void setStandbyYAccelEnabled(bool enabled);
	//bool getStandbyZAccelEnabled();
	//void setStandbyZAccelEnabled(bool enabled);
	//bool getStandbyXGyroEnabled();
	//void setStandbyXGyroEnabled(bool enabled);
	//bool getStandbyYGyroEnabled();
	//void setStandbyYGyroEnabled(bool enabled);
	//bool getStandbyZGyroEnabled();
	//void setStandbyZGyroEnabled(bool enabled);
	//
	//// FIFO_COUNT_ registers
	//uint16_t getFIFOCount();
	//
	//// FIFO_R_W register
	//uint8_t getFIFOByte();
	//void setFIFOByte(uint8_t data);
	//void getFIFOBytes(uint8_t *data, uint8_t length);
	
	void setSleepEnabled(bool isEnabled);
	bool test();

	//// WHO_AM_I register
	//bool who_am_I(void);
}

#endif // MPU6050_H
