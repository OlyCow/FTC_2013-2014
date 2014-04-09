// �׸��Ե���
// For collecting most of the data used in teleop (temp, IMU, etc.).
#include "Data_controller.h"



// Variables that are shared inside interrupts.
volatile uint8_t comm_x = 0;
volatile uint8_t comm_y = 0;
volatile uint8_t comm_z_L = 0;
volatile uint8_t comm_z_H = 0;
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
	
	int comm_x_temp = 0;
	int comm_y_temp = 0;
	int comm_z_temp = 0;
	double t_prev = 0.0; // TODO: Wrap all timers into a library.
	double t_current = 0.0;
	double dt = t_current - t_prev;
	double rot_x = 0.0;
	double rot_y = 0.0;
	double rot_z = 0.0;
	int vel_x_signed = 0;
	int vel_y_signed = 0;
	int vel_z_signed = 0;
	//int vel_x_signed_prev = 0;
	//int vel_y_signed_prev = 0;
	//int vel_z_signed_prev = 0;
	uint16_t vel_x = 0; // TODO: switch all of these over to unions.
	uint16_t vel_y = 0;
	uint16_t vel_z = 0;
	uint8_t vel_x_L = 0;
	uint8_t vel_x_H = 0;
	uint8_t vel_y_L = 0;
	uint8_t vel_y_H = 0;
	uint8_t vel_z_L = 0;
	uint8_t vel_z_H = 0;
	int vel_x_offset = 0;
	int vel_y_offset = 0;
	int vel_z_offset = 0;
	const double bit_to_gyro = 250.0/32768.0; // Also in MPU-6050 Register Map "Gyroscope Measurements".
	const double usec_to_sec = 1.0/1000000.0;
	short LED_timer = 0;
	
	// Initialize SPI.
	SPCR = ((1<<SPE) |	// Enable SPI.
			(0<<MSTR) |	// 0=slave, 1=master.
			(0<<DORD) |	// 0=MSB transmitted first.
			(0<<CPOL) |	// Setting both of these to 0 ="mode 0".
			(0<<CPHA));
	// TODO: Is this necessary? (Re-pulling up the SS' pin.)
	PORTB |= (1<<PINB2);
	
	// Initialize TWI.
	i2c_init();
	
	// Initialize and calibrate gyro.
	MPU::initialize();
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x03);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x01); // NOTE: This could be a very bad idea.
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);
	MPU::write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00);
	//// TODO: Make this calibration better.
	//_delay_us(10); // MAGIC_NUM
	//double offset_x_sum = 0.0;
	//double offset_y_sum = 0.0;
	//double offset_z_sum = 0.0;
	//// MAGIC_NUM: 100 is a decent number of samples :P
	//for (int i=0; i<2; ++i) {
	//	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, vel_x_L);
	//	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, vel_x_H);
	//	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, vel_y_L);
	//	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, vel_y_H);
	//	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, vel_z_L);
	//	MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, vel_z_H);
	//	vel_x = (vel_x_H<<8) + vel_x_L;
	//	vel_y = (vel_y_H<<8) + vel_y_L;
	//	vel_z = (vel_z_H<<8) + vel_z_L;
	//	offset_x_sum += MPU::convert_complement(vel_x);
	//	offset_y_sum += MPU::convert_complement(vel_y);
	//	offset_z_sum += MPU::convert_complement(vel_z);
	//	_delay_us(10); // MAGIC_NUM
	//}
	//vel_x_offset = offset_x_sum/2.0; // MAGIC_NUM: number of samples.
	//vel_y_offset = offset_y_sum/2.0; // MAGIC_NUM: number of samples.
	//vel_z_offset = offset_z_sum/2.0; // MAGIC_NUM: number of samples.

	_delay_us(500); // MAGIC_NUM
	for (int i=0; i<10; i++) { // MAGIC_NUM: NOTE: flush out bad readings?
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, vel_x_L);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, vel_x_H);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, vel_y_L);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, vel_y_H);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, vel_z_L);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, vel_z_H);
		vel_x = (vel_x_H<<8) + vel_x_L;
		vel_y = (vel_y_H<<8) + vel_y_L;
		vel_z = (vel_z_H<<8) + vel_z_L;
		vel_x_offset = MPU::convert_complement(vel_x);
		vel_y_offset = MPU::convert_complement(vel_y);
		vel_z_offset = MPU::convert_complement(vel_z);
		//vel_x_signed_prev = vel_x_offset;
		//vel_y_signed_prev = vel_y_offset;
		//vel_z_signed_prev = vel_z_offset;
	}
	
	// Set up interrupts.
	PCMSK0 |= (1<<PCINT2);
	PCICR |= (1<<PCIE0);
	PCIFR |= (1<<PCIF0);
	sei();
	alertA();
	
	// Wait to make sure we've established communication with the Comm_controller.
	while (is_comm_ready != true) {
		_delay_us(100);	// MAGIC_NUM: I have no clue what I'm doing.
	}
	alertB();
	// TODO: (SOMETIMES) THE ABOVE BLOCKS WHEN BOOTING RIGHT AFTER TURNING THE NXT OFF.
	// THEN TO HAVE IT WORK AGAIN, TURN OFF THE NXT, WAIT A WHILE, THEN TURN IT BACK ON.

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
		if (is_gyro_resetting == true) {
			rot_x = 0;
			rot_y = 0;
			rot_z = 0;
			is_gyro_resetting = false;
		}
		
		// TODO: It *might* be more efficient to calculate x, y, and z one-at-a-time.
		//vel_x_signed_prev = vel_x_signed;
		//vel_y_signed_prev = vel_y_signed;
		//vel_z_signed_prev = vel_z_signed;
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, vel_x_L); // TODO: Condense into a single "burst read".
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, vel_x_H);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, vel_y_L);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, vel_y_H);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, vel_z_L);
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, vel_z_H);
		vel_x = (vel_x_H<<8) + vel_x_L;
		vel_y = (vel_y_H<<8) + vel_y_L;
		vel_z = (vel_z_H<<8) + vel_z_L;
		vel_x_signed = MPU::convert_complement(vel_x);
		vel_y_signed = MPU::convert_complement(vel_y);
		vel_z_signed = MPU::convert_complement(vel_z);
		vel_x_signed -= vel_x_offset;
		vel_y_signed -= vel_y_offset;
		vel_z_signed -= vel_z_offset;

		// Yeah trapezoids. Also dividing is too much brain-work. Since we're multiplying already...
		double rect_x = (double)vel_x_signed;
		double rect_y = (double)vel_y_signed;
		double rect_z = (double)vel_z_signed;
		//rect_x += (double)vel_x_signed_prev;
		//rect_y += (double)vel_y_signed_prev;
		//rect_z += (double)vel_z_signed_prev;
		//rect_x /= 2.0;
		//rect_y /= 2.0;
		//rect_z /= 2.0;
		rect_x *= (bit_to_gyro * (double)dt * usec_to_sec);
		rect_y *= (bit_to_gyro * (double)dt * usec_to_sec);
		rect_z *= (bit_to_gyro * (double)dt * usec_to_sec);
		rot_x += rect_x;
		rot_y += rect_y;
		rot_z += rect_z;
		rot_x = fmod(rot_x, 360);
		rot_y = fmod(rot_y, 360);
		rot_z = fmod(rot_z, 360);
		if (rot_z<(-360.0)) {
			rot_z += 360.0;	// Making sure we're positive.
		}
		
		if (rot_x>=30.0) {
			comm_x_temp = 30;
		} else if (rot_x<=(-30.0)) {
			comm_x_temp = -30;
		} else {
			comm_x_temp = static_cast<int>(round(rot_x));
		}
		comm_x_temp += 30;
		if (rot_y>=30.0) {
			comm_y_temp = 30;
		} else if (rot_y<=(-30.0)) {
			comm_y_temp = -30;
		} else {
			comm_y_temp = static_cast<int>(round(rot_y));
		}
		comm_y_temp += 30;
		comm_z_temp = static_cast<int>(round(rot_z));
		// TODO: Split into four atomic blocks?
		// Pros:	Interrupts can happen before all three vars get assigned.
		// Cons:	Possibly more overhead and too much delay from capturing
		//			and releasing the processor (and might miss the SS window).
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			comm_x = static_cast<uint8_t>(comm_x_temp);
			comm_y = static_cast<uint8_t>(comm_y_temp);
			comm_z_L = static_cast<uint8_t>(comm_z_temp & 0x00FF);
			comm_z_H = static_cast<uint8_t>((comm_z_temp & 0x0100) >>8);
		}
		
		
		
		// TODO: This is just to make sure we are indeed executing inside the main loop.
		alertC();
		
		// TODO: DELETE THIS (TESTING)
		if (fabs(rot_z)<0.5) {
			alertD();
		} else {
			clearD();
		}
		
		// TODO: This is extremely hackish (and possibly dangerous).
		// Delete at some point. Also there's no debouncing at all.
		if ((PINB&(1<<PINB1))==0) {
			is_gyro_resetting = true;
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
	if ((PINB & (1<<PINB2)) != 0) {
		return; // We only care if the SS' pin is pulled low.
	} else {
		
		uint8_t spi_W = STATUS_W_INIT;
		uint8_t spi_R = 0;
		
		while ((PINB & (1<<PINB2)) == 0) {
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
				case STATUS_R_REQ_GYRO_Z_L :
					spi_W = comm_z_L;
					break;
				case STATUS_R_REQ_GYRO_Z_H :
					spi_W = comm_z_H;
					break;
				case STATUS_R_END :
					spi_W = STATUS_W_ACK;
					break;
					
				// Status setting codes:
				case STATUS_R_RESET_GYRO :
					//is_gyro_resetting = true;
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
					
				default :
					// TODO: Should probably do something other than break...
					break;
			}
		}
	}
}

void alertA() { PORTD |= 1<<PORTD5; }
void clearA() { PORTD &= ~(1<<PORTD5); }
void alertB() { PORTD |= 1<<PORTD6; }
void clearB() { PORTD &= ~(1<<PORTD6); }
void alertC() { PORTD |= 1<<PORTD7; }
void clearC() { PORTD &= ~(1<<PORTD7); }
void alertD() { PORTB |= 1<<PORTB0; }
void clearD() { PORTB &= ~(1<<PORTB0); }

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
			 (0<<PINB3) |	// SPI doesn't need pull-up.
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
