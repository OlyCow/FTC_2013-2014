// For communicating with the NXT (via the SuperPro board).
#include "Comm_controller.h"

int main(void)
{
	setupPins();
	TWI::setup();
	MPU::initialize();
	
	//// TODO: Uncomment this stuff when we get the ATmega328s.
	//// TODO: Move this interrupt registry stuff over to the
	//// header file when they have been confirmed to work.
	//PCMSK0 = (1<<PCINT1); 
	//PCICR = (1<<PCIE0);
	//
	//// When we're ready, enable interrupts.
	//sei();
	
	// Setting up a timer for debouncing.
	// When CS10=1 and CS11, CS12=0, clock prescaling = 1.
	// TODO: Is this the best way to set 3 different bits?
	TCCR1B |= (1 << CS10); // Set CS10 in control registry.
	
	// Setting up a timer for the gyro and stuffs.
	TCCR1A |= (1 << CS10);
	
	// Variables for I/O with the NXT (prototype board).
	bool clock_NXT_current = false;				// TODO: I don't think this initialization matters... Does it?
	bool clock_NXT_prev = false;				// DERP
	bool header_read = false;					// We want to default to "normal" data.
	bool header_write[NXT_LINE_NUM] = {false,false,false,false,false,false};
	uint32_t data_read = 0;
	uint32_t data_write[NXT_LINE_NUM] = {0,0,0,0,0,0};
	bool parity_read = false;					// TODO: Leaving this undefined really isn't a good idea either... Oh well.
	bool parity_read_check = parity_read;		// DERP
	bool parity_write[NXT_LINE_NUM] = {false,false,false,false,false,false}; // TODO: Same problem as `parity_read`.
	uint8_t byte_write = 0;
	uint8_t byte_read = 0; // TODO: This really could be a bool... Would that be more confusing?
	int data_pos = 0;
	enum IOstate {
		IO_STATE_RESET	= 0,
		IO_STATE_HEADER	= 1,
		IO_STATE_DATA	= 2,
		IO_STATE_PARITY	= 3
	};
	IOstate isIOstate = IO_STATE_RESET;
	int resetAckCounter = 0; // Goes up to 1 :P
		
	// Data gathered from various pins to report back.
	uint8_t cube_num = 0;
	
	// Variables to process pin inputs.
	bool cube_counter_current = false;
	bool cube_counter_prev = false;
	bool isDebouncing = false;
	
	// Variables to process MPU-6050 data.
	double dt = 0.0;
	//uint8_t	gyro_x_L = 0,
			//gyro_x_H = 0,
			//gyro_y_L = 0,
			//gyro_y_H = 0,
			//gyro_z_L = 0,
			//gyro_z_H = 0;
		uint8_t gyro_x_L[1] = {0};
	uint16_t gyro_x = 0,
			 gyro_y = 0,
			 gyro_z = 0;
	double	rot_x = 0.0,
			rot_y = 0.0,
			rot_z = 0.0;
	
	while (true) {
		// Process NXT (prototype board) I/O.
		clock_NXT_current = (PIND & (1<<PD0));
		if (clock_NXT_current != clock_NXT_prev) {
			clock_NXT_prev = clock_NXT_current;
			
			// Set `byte_write`.
			switch (isIOstate) {
				case IO_STATE_RESET :
					if ((PIND & (1<<PD1)) == true) {
						resetAckCounter = 0;
						switch (clock_NXT_current) {
							case true :
								byte_write = 0b11111111;
								break;
							case false :
								byte_write = 0b00000000;
								break;
						}
					} else {
						resetAckCounter++;
						byte_write = 0b00000000;
						if (resetAckCounter >= 2) {
							isIOstate = IO_STATE_HEADER;
						}
					}
					break;
				case IO_STATE_HEADER :
					header_read = PIND & (1<<PD1);
					byte_write = 0;
					for (int line=0; line<NXT_LINE_NUM; line++) {
						if (header_write[line]==true) {
							byte_write |= (1<<line);
						}
					}
					break;
				case IO_STATE_DATA :
					break;
				case IO_STATE_PARITY :
					break;
			}
			
			// Actually output values on the lines.
			// --------MOSI_NXT_A--------
			if ((byte_write&0b00000001) == true) {
				NXT_LINE_A_PORT |= (1<<NXT_LINE_A);
			} else {
				NXT_LINE_A_PORT &= (~(1<<NXT_LINE_A));
			}
			// --------MOSI_NXT_B--------
			if ((byte_write&0b00000010) == true) {
				NXT_LINE_B_PORT |= (1<<NXT_LINE_B);
				} else {
				NXT_LINE_B_PORT &= (~(1<<NXT_LINE_B));
			}
			// --------MOSI_NXT_C--------
			if ((byte_write&0b00000100) == true) {
				NXT_LINE_C_PORT |= (1<<NXT_LINE_C);
				} else {
				NXT_LINE_C_PORT &= (~(1<<NXT_LINE_C));
			}
			// --------MOSI_NXT_D--------
			if ((byte_write&0b00001000) == true) {
				NXT_LINE_D_PORT |= (1<<NXT_LINE_D);
				} else {
				NXT_LINE_D_PORT &= (~(1<<NXT_LINE_D));
			}
			// --------MOSI_NXT_E--------
			if ((byte_write&0b00010000) == true) {
				NXT_LINE_E_PORT |= (1<<NXT_LINE_E);
				} else {
				NXT_LINE_E_PORT &= (~(1<<NXT_LINE_E));
			}
			// --------MOSI_NXT_F--------
			if ((byte_write&0b00100000) == true) {
				NXT_LINE_F_PORT |= (1<<NXT_LINE_F);
				} else {
				NXT_LINE_F_PORT &= (~(1<<NXT_LINE_F));
			}
		}
		
		// Process cube counting.
		cube_counter_current = (PINB & (1<<PB1));
		if (cube_counter_current!=cube_counter_prev) {
			switch (isDebouncing) {
				case false :
					isDebouncing = true;
					TCNT1 = 0; // Clear this timer; start counting.
				case true :
					if (TCNT1 >= DEBOUNCE_COUNTS) {
						// Under the correct conditions, increment cube count.
						if (((~cube_counter_current)&cube_counter_prev) == true) {
							if (cube_num<4) {
								cube_num++; // Some really hackish error handling here :)
							}
						}
						// Get ready for the next cycle.
						TCNT1 = 0; // Clear clock.
						isDebouncing = false;
						cube_counter_prev = cube_counter_current;
					}
					break;
			}
		}
		
		// Process gyro data.
		MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, gyro_x_L, 1);
		if (gyro_x_L[0]>0) {
			alert();
		} else {
			clear();
		}
		//MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &gyro_z_L, 1);
		//MPU::read(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &gyro_z_H, 1);
		//gyro_z = gyro_z_L+(((uint16_t)(gyro_z_H))<<8);
		//if (gyro_z_L == 0) {
			//alert();
		//} else {
			////clear();
		//}
		
		
		// TODO: Get rid of the below. Debugging fun stuff.
		if (cube_num==4) {
			//alert();
		}
	}
}

//// TODO: Enable this when we get the ATmega328s.
//ISR(PCINT0_vect)
//{
	//
//}

void setupPins(void)
{
	// Set up I/O port directions with the DDRx registers. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	//----------------SCHEMATIC----------------
	//  1-PC6: RESET			28-PC5: LED_A (SCL)
	//  2-PD0: SCLK_NXT			27-PC4: LED_B (SDA)
	//  3-PD1: MISO_NXT			26-PC3: LIGHT_SEL_A
	//  4-PD2: SS_SEL_A			25-PC2: LIGHT_SEL_B
	//  5-PD3: SS_SEL_B			24-PC1: LIGHT_SEL_C
	//  6-PD4: SS_SEL_C			23-PC0: LIGHT_READ
	//  7-[VCC]					22-[GND]
	//  8-[GND]					21-[AREF]
	//  9-PB6: MOSI_NXT_A		20-[AVCC]
	// 10-PB7: MOSI_NXT_B		19-PB5: SCLK_MCU
	// 11-PD5: MOSI_NXT_C		18-PB4: MISO_MCU
	// 12-PD6: MOSI_NXT_D		17-PB3: MOSI_MCU
	// 13-PD7: MOSI_NXT_E		16-PB2: SS_MCU_WRITE
	// 14-PB0: MOSI_NXT_F		15-PB1: LIFT_RESET (cube counter)
	DDRB = ((1<<PB0) |
			(0<<PB1) |
			(1<<PB2) |
			(1<<PB3) |
			(0<<PB4) |
			(1<<PB5) |
			(1<<PB6) |
			(1<<PB7));
	DDRC = ((0<<PC0) |
			(1<<PC1) |
			(1<<PC2) |
			(1<<PC3) |
			(1<<PC4) |
			(1<<PC5) |
			(0<<PC6)); // No bit 7.
	DDRD = ((0<<PD0) |
			(0<<PD1) |
			(1<<PD2) |
			(1<<PD3) |
			(1<<PD4) |
			(1<<PD5) |
			(1<<PD6) |
			(1<<PD7));
	
	// (PORTx registers) Initialize outputs to 0 (LOW), and enable internal
	// pull-up resistors for the appropriate inputs (most notably the SDA &
	// SCL pins). 1=pull-up resistor enabled. For details, see schematic for
	// the DDRx registers' set-up.
	// SPI shouldn't need pull-up resistors. Nor do multiplexer read pins.
	PORTB = ((0<<PB0) |
			 (1<<PB1) |
			 (0<<PB2) |
			 (0<<PB3) |
			 (0<<PB4) |
			 (0<<PB5) |
			 (0<<PB6) |
			 (0<<PB7));
	PORTC = ((0<<PC0) |
			 (0<<PC1) |
			 (0<<PC2) |
			 (0<<PC3) |
			 (0<<PC4) |
			 (0<<PC5) |
			 (0<<PC6)); // Pull-up unnecessary for RESET pin. No bit 7.
	PORTD = ((0<<PD0) |
			 (0<<PD1) |
			 (0<<PD2) |
			 (0<<PD3) |
			 (0<<PD4) |
			 (0<<PD5) |
			 (0<<PD6) |
			 (0<<PD7));
}

void alert(void)
{
	PORTB |= (1<<PB2);
}
void clear(void)
{
	PORTB &= ~(1<<PB2);
}
