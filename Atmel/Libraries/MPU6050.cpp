// Basic acquiring and processing of the data fetched from the MPU-6050.
// A lot of stuff is taken from Jeff Rowberg's library. <jeff@rowberg.net>
// For more documentation (and I mean *more*), see his website or repo.
#include "MPU6050.h"

void MPU::read(uint8_t address, uint8_t request, uint8_t data[], int size)
{
	uint8_t address_W = address<<1;
	uint8_t address_R = address_W|1;
	i2c_start(address_W);
	i2c_write(request);
	i2c_rep_start(address_R);
	for (int i=0; i<size; i++) {
		if (i+1<size) {
			data[i] = i2c_readAck();
		} else {
			data[i] = i2c_readNak();
		}
	}
	i2c_stop();
}
//void MPU::read(uint8_t address, uint8_t request, uint8_t data[], int size)
//{
	//TWI::start();
	//TWI::write_SLA_W(address);
	//TWI::write_data(request);
	//TWI::hold(); // Actually, repeated start. :)
	//TWI::write_SLA_R(address);
	//TWI::read_data(data, size);
	//TWI::stop();
//}

//void MPU::write(uint8_t address, uint8_t request, uint8_t data[], int size)
void MPU::write(uint8_t address, uint8_t request, uint8_t data)
{
	uint8_t address_W = address<<1;
	uint8_t address_R = address_W|1;
	i2c_start(address_W);
	i2c_write(request);
	i2c_write(data);
	i2c_stop();
}
//void MPU::write(uint8_t address, uint8_t request, uint8_t data[], int size)
//{
	//TWI::start();
	//TWI::write_SLA_W(address);
	//TWI::write_data(request);
	//TWI::hold();
	//TWI::write_SLA_W(address);
	//TWI::write_data(data);
	//TWI::stop();
//}

void MPU::initialize(void)
{
	//setClockSource();
	//setFullScaleGyroRange();
	//setFullScaleAccelRange();
	MPU::setSleepEnabled(false);
}

// SMPLRT_DIV register

//// Get gyroscope output rate divider.
//// The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
//// Motion detection, and Free Fall detection are all based on the Sample Rate.
//// The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:
//// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
//// where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
//// 7), and 1kHz when the DLPF is enabled (see Register 26).
//uint8_t MPU::getRate() {
	//uint8_t buffer[1] = {0};
	//MPU::read(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, buffer, 1);
    //return buffer[0];
//}
//// Set gyroscope sample rate divider.
//void MPU::setRate(uint8_t rate) {
	//MPU::write(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, rate, 1);
//}

//// CONFIG register
//
//// Get external FSYNC configuration.
//// Configures the external Frame Synchronization (FSYNC) pin sampling. An
//// external signal connected to the FSYNC pin can be sampled by configuring
//// EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
//// strobes may be captured. The latched FSYNC signal will be sampled at the
//// Sampling Rate, as defined in register 25. After sampling, the latch will
//// reset to the current FSYNC signal state.
//// The sampled value will be reported in place of the least significant bit in
//// a sensor data register determined by the value of EXT_SYNC_SET according to
//// the following table.
//// EXT_SYNC_SET | FSYNC Bit Location
//// -------------+-------------------
//// 0            | Input disabled
//// 1            | TEMP_OUT_L[0]
//// 2            | GYRO_XOUT_L[0]
//// 3            | GYRO_YOUT_L[0]
//// 4            | GYRO_ZOUT_L[0]
//// 5            | ACCEL_XOUT_L[0]
//// 6            | ACCEL_YOUT_L[0]
//// 7            | ACCEL_ZOUT_L[0]
//uint8_t MPU::getExternalFrameSync() {
	//uint8_t buffer[1] = {0};
	//MPU::read(MPU6050_ADDRESS, MPU6050_RA_CONFIG, buffer, 1);
    ////I2Cdev::readBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, buffer);
    //return buffer[0];
//}
///** Set external FSYNC configuration.
 //* @see getExternalFrameSync()
 //* @see MPU6050_RA_CONFIG
 //* @param sync New FSYNC configuration value
 //*/
//void MPU6050::setExternalFrameSync(uint8_t sync) {
    //I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
//}
///** Get digital low-pass filter configuration.
 //* The DLPF_CFG parameter sets the digital low pass filter configuration. It
 //* also determines the internal sampling rate used by the device as shown in
 //* the table below.
 //*
 //* Note: The accelerometer output rate is 1kHz. This means that for a Sample
 //* Rate greater than 1kHz, the same accelerometer sample may be output to the
 //* FIFO, DMP, and sensor registers more than once.
 //*
 //* <pre>
 //*          |   ACCELEROMETER    |           GYROSCOPE
 //* DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 //* ---------+-----------+--------+-----------+--------+-------------
 //* 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 //* 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 //* 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 //* 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 //* 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 //* 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 //* 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 //* 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 //* </pre>
 //*
 //* @return DLFP configuration
 //* @see MPU6050_RA_CONFIG
 //* @see MPU6050_CFG_DLPF_CFG_BIT
 //* @see MPU6050_CFG_DLPF_CFG_LENGTH
 //*/
//uint8_t MPU6050::getDLPFMode() {
    //I2Cdev::readBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, buffer);
    //return buffer[0];
//}
////  Set digital low-pass filter configuration.
//// @param mode New DLFP configuration setting
//// @see getDLPFBandwidth()
//// @see MPU6050_DLPF_BW_256
//// @see MPU6050_RA_CONFIG
//// @see MPU6050_CFG_DLPF_CFG_BIT
//// @see MPU6050_CFG_DLPF_CFG_LENGTH
//void MPU6050::setDLPFMode(uint8_t mode) {
    //I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
//}

void MPU::setSleepEnabled(bool isEnabled)
{
	uint8_t sleep_bit = 0x00;
	if (isEnabled==true) {
		sleep_bit = 0b01000000; // This resets a bunch of other stuff :/
	}
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, sleep_bit);
	// TODO: This function is unbelievably hackish. Unbelievably.
}

//bool MPU::who_am_I(void)
//{
	//bool success = false;
	//uint8_t answer[] = {0};
	//MPU::read(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, answer, 1);
	//success = (answer[0]==MPU6050_DEFAULT_ADDRESS);
	//return success;
//}
