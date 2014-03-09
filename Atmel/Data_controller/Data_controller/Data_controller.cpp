// For collecting most of the data used in teleop (temp, IMU, etc.).
#include "Data_controller.h"

int main()
{
	setupPins();
	
	// Set up a system timer.
	// When CS10=1, and CS11 & CS12=0, clock prescaling = 1.
	// TODO: Is this the best way to set 3 different bits?
	// TOOD: Encapsulate these into a class!
	TCCR1B |= (1 << CS10); // Set CS10 in control registry.
	uint64_t SYSTEM_TIME = 0; // In microseconds.
	
	double t_prev = 0.0; // TODO: Wrap all timers into a library.
	double t_current = 0.0;
	double dt = t_current - t_prev;
	uint16_t comm_x = 0;
	uint16_t comm_y = 0;
	uint16_t comm_z = 0;
	double rot_x = 0.0;
	double rot_y = 0.0;
	double rot_z = 0.0;
	uint16_t vel_x = 0; // TODO: switch all of these over to unions.
	uint16_t vel_y = 0;
	uint16_t vel_z = 0;
	uint8_t vel_x_H = 0;
	uint8_t vel_x_L = 0;
	uint8_t vel_y_H = 0;
	uint8_t vel_y_L = 0;
	uint8_t vel_z_H = 0;
	uint8_t vel_z_L = 0;
	uint16_t vel_x_offset = 0;
	uint16_t vel_y_offset = 0;
	uint16_t vel_z_offset = 0;
	
	// Initialize SPI.
	SPCR = ((1<<SPE) |	// Enable SPI.
			(0<<MSTR) |	// 0=slave, 1=master.
			(0<<DORD) |	// 0=MSB transmitted first.
			(0<<CPOL) |	// Setting both of these to 0 ="mode 0".
			(0<<CPHA));
	
	// Initialize TWI.
	i2c_init();
	
	// Initialize gyro.
	MPU::initialize();
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00);
	
	// wait for Comm_controller and ACK
		// SS wait for LOW
		// MISO: STATUS_INIT'd
		// MOSI: STATUS_READY
	// initialize interrupts
	
    while (true) {
		// Update system timer. TODO: Encapsulate timers into a class.
		SYSTEM_TIME += TCNT1;
		TCNT1 = 0;
		
		// `dt` can be directly found (`TCNT1`), but I wanted to keep `dt`
		// independent from the timer registers so I can (TODO) encapsulate
		// those into a class when I have some spare time (ha ha).
		t_prev = t_current; // This is the "old" `t_current`.
		t_current = SYSTEM_TIME; // Now `t_current` is up-to-date.
		dt = t_current-t_prev;
		
		// Poll. Debug alert can happen at any step.
		// bumper mux (repeat x8):
			// increment mux
			// read
		// misc. (ADC) mux (repeat x16):
			// increment mux
			// read
		// gyro:
			// read from gyro
			// process gyro
		// debugging:
			// read button
			// set LEDs
    }
}

void setupPins()
{
	// Set up I/O port directions with the DDRx registers. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	//----------------SCHEMATIC----------------
	//  1-PC6: RESET			28-PC5: GYRO_SCL
	//  2-PD0: BUMP_READ		27-PC4: GYRO_SDA
	//  3-PD1: DATA_SEL_A		26-PC3: BUMP_SEL_A
	//  4-PD2: DATA_SEL_B		25-PC2: BUMP_SEL_B
	//  5-PD3: DATA_SEL_C		24-PC1: BUMP_SEL_C
	//  6-PD4: DATA_SEL_D		23-PC0: DATA_READ
	//  7-[VCC]					22-[GND]
	//  8-[GND]					21-[AREF]
	//  9-PB6: [UNUSED]			20-[AVCC]
	// 10-PB7: [UNUSED]			19-PB5: SCLK
	// 11-PD5: LED_A			18-PB4: MISO
	// 12-PD6: LED_B			17-PB3: MOSI
	// 13-PD7: LED_C			16-PB2: SS
	// 14-PB0: LED_D			15-PB1: (pushbutton)
	DDRB = ((1<<PB0) |
			(0<<PB1) |
			(0<<PB2) |
			(0<<PB3) |
			(1<<PB4) |
			(0<<PB5) |
			(0<<PB6) |	// Initialize unused pins to input (tristated).
			(0<<PB7));	// Initialize unused pins to input (tristated).
	DDRC = ((0<<PC0) |
			(1<<PC1) |
			(1<<PC2) |
			(1<<PC3) |
			(0<<PC4) |	// SDA pin initialized to input (reconfig after TWI init).
			(1<<PC5) |
			(0<<PC6));	// RESET pin initialized to input. No bit 7.
	DDRD = ((0<<PD0) |
			(1<<PD1) |
			(1<<PD2) |
			(1<<PD3) |
			(1<<PD4) |
			(1<<PD5) |
			(1<<PD6) |
			(1<<PD7));
	
	// (PORTx registers) Initialize outputs to 0 (LOW), and enable internal
	// pull-ups for the appropriate inputs. 1=pull-up resistor enabled. For
	// details, see the schematic for the DDRx registers' set-up.
	// SPI shouldn't need pull-up resistors. Nor do multiplexer read pins.
	PORTB = ((0<<PB0) |
			 (1<<PB1) |
			 (1<<PB2) |	// We DO want SS to be pulled up.
			 (0<<PB3) | // SPI doesn't need pull-up.
			 (0<<PB4) |
			 (0<<PB5) |
			 (0<<PB6) |	// Unused pins shouldn't be pulled up.
			 (0<<PB7));	// Unused pins shouldn't be pulled up.
	PORTC = ((0<<PC0) |	// Misc. mux read pin doesn't need pull-up.
			 (0<<PC1) |
			 (0<<PC2) |
			 (0<<PC3) |
			 (0<<PC4) |	// SDA pin already has a strong pull-up on it.
			 (0<<PC5) |
			 (0<<PC6));	// Pull-up unnecessary for RESET pin. No bit 7.
	PORTD = ((0<<PD0) |	// Bump mux read pin doesn't need pull-up.
			 (0<<PD1) |
			 (0<<PD2) |
			 (0<<PD3) |
			 (0<<PD4) |
			 (0<<PD5) |
			 (0<<PD6) |
			 (0<<PD7));
}