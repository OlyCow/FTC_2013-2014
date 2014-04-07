// Ï×¸øÕÔµ¤Öñ
// For collecting most of the data used in teleop (temp, IMU, etc.).
#include "Data_controller.h"



// Variables that are shared inside interrupts.
volatile uint16_t comm_x = 0;
volatile uint16_t comm_y = 0;
volatile uint16_t comm_z = 0;
volatile bool is_gyro_resetting = false;
volatile bool is_comm_ready = false;
enum LED_STATE {
	LED_OFF		= 0,
	LED_ON		= 1,
	LED_BLINK	= 2,
	LED_STATE_NUM
};
enum LED_PICK {
	LED_A	= 0,
	LED_B	= 2,
	LED_C	= 2,
	LED_D	= 3,
	LED_NUM
};
volatile LED_STATE LED_state[LED_NUM] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF};

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
	double rot_x = 0.0;
	double rot_y = 0.0;
	double rot_z = 0.0;
	uint16_t vel_x = 0; // TODO: switch all of these over to unions.
	uint16_t vel_y = 0;
	uint16_t vel_z = 0;
	uint8_t vel_x_L = 0;
	uint8_t vel_x_H = 0;
	uint8_t vel_y_L = 0;
	uint8_t vel_y_H = 0;
	uint8_t vel_z_L = 0;
	uint8_t vel_z_H = 0;
	uint16_t vel_x_offset = 0;
	uint16_t vel_y_offset = 0;
	uint16_t vel_z_offset = 0;
	const double bit_to_gyro = 250.0/32768.0;
	short LED_timer = 0;
	
	// Initialize SPI.
	SPCR = ((1<<SPE) |	// Enable SPI.
			(0<<MSTR) |	// 0=slave, 1=master.
			(0<<DORD) |	// 0=MSB transmitted first.
			(0<<CPOL) |	// Setting both of these to 0 ="mode 0".
			(0<<CPHA));
	
	// Initialize TWI.
	i2c_init();
	
	// Initialize and calibrate gyro.
	MPU::initialize();
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00);
	// TODO: Make this calibration better (take an average?).
	_delay_ms(100); // MAGIC_NUM: 
	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, vel_x_L);
	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, vel_x_H);
	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, vel_y_L);
	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, vel_y_H);
	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, vel_z_L);
	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, vel_z_H);
	vel_x_offset = (vel_x_H<<8) + vel_x_L;
	vel_y_offset = (vel_y_H<<8) + vel_y_L;
	vel_z_offset = (vel_z_H<<8) + vel_z_L;
	
	// Set up interrupts.
	PCMSK0 |= (1<<PCINT2);
	PCICR |= (1<<PCIE2);
	PCIFR |= (1<<PCIF0);
	sei();
	
	// Wait to make sure we've established communication with the Comm_controller.
	//while (is_comm_ready != true) {;}

	// TODO: Set up any interrupts that haven't been setup yet.
	
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
		
		// Gyro handling.
		// TODO: It *might* be more efficient to calculate x, y, and z one-at-a-time.
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, vel_x_L); // TODO: Condense into a single "burst read".
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, vel_x_H);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, vel_y_L);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, vel_y_H);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, vel_z_L);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, vel_z_H);
		vel_x = (vel_x_H<<8) + vel_x_L - vel_x_offset;
		vel_y = (vel_y_H<<8) + vel_y_L - vel_y_offset;
		vel_z = (vel_z_H<<8) + vel_z_L - vel_z_offset;
		rot_x += ((double)(vel_x-32768)*dt) * bit_to_gyro;
		rot_y += ((double)(vel_y-32768)*dt) * bit_to_gyro;
		rot_z += ((double)(vel_z-32768)*dt) * bit_to_gyro;
		if (is_gyro_resetting == true) {
			rot_x = 0;
			rot_x = 0;
			rot_x = 0;
			is_gyro_resetting = false;
		}
		// TODO: Split into three atomic blocks?
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			comm_x = static_cast<int>(round(rot_x)) % 360;
			comm_y = static_cast<int>(round(rot_y)) % 360;
			comm_z = static_cast<int>(round(rot_z)) % 360;
		}
		
		
		
		//TODO: DELETE THIS (TESTING)
		uint8_t test_A = 0;
		uint8_t test_B = 0;
		uint8_t test_C = 0;
		//MPU::read(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, test_A);
		//MPU::read(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, test_B);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, test_C);
		//test_C = (test_B<<8) + test_A;
		if (test_C==0) {
		//if (test_C==MPU6050_ADDRESS) {
			alertD();
		} else {
			clearD();
		}
		
		
		
		// Debugging LEDs and pushbutton.
			// read and debounce button
		++LED_timer;
		// TODO: make this "for" loop declaration much better :P
		// TODO: activate the switch statement.
		for (int led=0; led<4; led++) {
			//switch (LED_state[led]) {
				//case LED_ON :
					//LED_port[led] |= (1<<(LED_pin[led]));
					//break;
				//case LED_OFF :
					//LED_port[led] &= ~(1<<(LED_pin[led]));
					//break;
				//case LED_BLINK :
					//// TODO: Improve blinking mechanics.
					//if (LED_timer<0) {
						//LED_port[led] |= (1<<(LED_pin[led]));
					//} else {
						//LED_port[led] &= ~(1<<(LED_pin[led]));
					//}
					//break;
			//}
		}
    }
}

ISR(PCINT0_vect)
{
	if ((PINB & (0<<PINB2)) != 0) {
		return; // We only care if the SS' pin is pulled low.
	} else {
		while ((PINB & (0<<PINB2)) == 0) {
			uint8_t spi_W = STATUS_W_INIT;
			uint8_t spi_R = 0;
			
			SPDR = spi_W;
			while(!(SPSR & (1<<SPIF))) {;} // Wait until all the data is received.
			spi_R = SPDR;
			
			switch (spi_R) {
				// Initialization codes:
				case STATUS_R_INIT :
					is_comm_ready = true;
					spi_W = STATUS_W_INIT;
					break;
				case STATUS_R_ACK :
					spi_W = STATUS_W_ACK;
					break;
					
				// Data request codes:
				case STATUS_R_REQ_GYRO_X :
					spi_W = comm_x;
					break;
				case STATUS_R_REQ_GYRO_Y :
					spi_W = comm_y;
					break;
				case STATUS_R_REQ_GYRO_Z :
					spi_W = comm_z;
					break;
				case STATUS_R_END :
					spi_W = STATUS_W_ACK;
					break;
					
				// Status setting codes:
				case STATUS_R_RESET_GYRO :
					is_gyro_resetting = true;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_A_ON :
					LED_state[LED_A] = LED_ON;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_B_ON :
					LED_state[LED_B] = LED_ON;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_C_ON :
					LED_state[LED_C] = LED_ON;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_D_ON :
					LED_state[LED_D] = LED_ON;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_A_BLINK :
					LED_state[LED_A] = LED_BLINK;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_B_BLINK :
					LED_state[LED_B] = LED_BLINK;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_C_BLINK :
					LED_state[LED_C] = LED_BLINK;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_D_BLINK :
					LED_state[LED_D] = LED_BLINK;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_A_OFF :
					LED_state[LED_A] = LED_OFF;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_B_OFF :
					LED_state[LED_B] = LED_OFF;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_C_OFF :
					LED_state[LED_C] = LED_OFF;
					spi_W = STATUS_W_ACK;
					break;
				case STATUS_R_LED_D_OFF :
					LED_state[LED_D] = LED_OFF;
					spi_W = STATUS_W_ACK;
					break;
			}
		}
	}
}

void alertD()
{
	PORTB |= 1<<PORTB0;
}

void clearD()
{
	PORTB &= ~(1<<PORTB0);
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
	DDRB = ((1<<PINB0) |
			(0<<PINB1) |
			(0<<PINB2) |
			(0<<PINB3) |
			(1<<PINB4) |
			(0<<PINB5) |
			(0<<PINB6) |	// Initialize unused pins to input (tristated).
			(0<<PINB7));	// Initialize unused pins to input (tristated).
	DDRC = ((0<<PINC0) |
			(1<<PINC1) |
			(1<<PINC2) |
			(1<<PINC3) |
			(0<<PINC4) |	// SDA pin initialized to input (reconfig after TWI init).
			(1<<PINC5) |
			(0<<PINC6));	// RESET pin initialized to input. No bit 7.
	DDRD = ((0<<PIND0) |
			(1<<PIND1) |
			(1<<PIND2) |
			(1<<PIND3) |
			(1<<PIND4) |
			(1<<PIND5) |
			(1<<PIND6) |
			(1<<PIND7));
	
	// (PORTx registers) Initialize outputs to 0 (LOW), and enable internal
	// pull-ups for the appropriate inputs. 1=pull-up resistor enabled. For
	// details, see the schematic for the DDRx registers' set-up.
	// SPI shouldn't need pull-up resistors. Nor do multiplexer read pins.
	PORTB = ((0<<PINB0) |
			 (1<<PINB1) |
			 (1<<PINB2) |	// We DO want SS to be pulled up.
			 (0<<PINB3) | // SPI doesn't need pull-up.
			 (0<<PINB4) |
			 (0<<PINB5) |
			 (0<<PINB6) |	// Unused pins shouldn't be pulled up.
			 (0<<PINB7));	// Unused pins shouldn't be pulled up.
	PORTC = ((0<<PINC0) |	// Misc. mux read pin doesn't need pull-up.
			 (0<<PINC1) |
			 (0<<PINC2) |
			 (0<<PINC3) |
			 (0<<PINC4) |	// SDA pin already has a strong pull-up on it.
			 (0<<PINC5) |
			 (0<<PINC6));	// Pull-up unnecessary for RESET pin. No bit 7.
	PORTD = ((0<<PIND0) |	// Bump mux read pin doesn't need pull-up.
			 (0<<PIND1) |
			 (0<<PIND2) |
			 (0<<PIND3) |
			 (0<<PIND4) |
			 (0<<PIND5) |
			 (0<<PIND6) |
			 (0<<PIND7));
}