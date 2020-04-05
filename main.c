/*
 * nixie.c
 *
 * Created: 1/17/2016 3:05:42 PM
 * Author : Simon Winder
 * Impressive Machines LLC
 * simon@impressivemachines.com
 *
 * Modified 2020-04-04 15:25 EST
 * by Riley Scott Jacob for specific
 * use as VFD readout; my comments marked by ///
 */

#include <avr/io.h>
#include <avr/interrupt.h>

// FUSE BYTES MUST BE:
// EXTENDED = FF
// HIGH = D9
// LOW = DE (for 16MHz external crystal)

#define F_CPU	16000000 /// 16 MHz CPU clock

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

u8 g_r[4];
u8 g_g[4];
u8 g_b[4];

volatile u8 g_scan;
volatile u8 g_digit[4];
volatile u8 g_timer;

void send_bit(u8 b)
{
	PORTC = (PORTC&0xcf) | (b&1)<<4;
	PORTC = (PORTC&0xdf) | (1<<5);
	PORTC = (PORTC&0xdf);
}

void send_rgb()
{
	send_bit(1); // cmd
	send_bit(0); // cmd
	send_bit(0); // cmd
	send_bit(1); // cmd
	send_bit(0); // cmd
	send_bit(1); // cmd
	send_bit(0); // outtmg
	send_bit(0); // extgck
	send_bit(0); // tmgrst
	send_bit(1); // dsprpt
	send_bit(0); // blank
	u8 i,k;
	for(i=0; i<21; i++)
		send_bit(1);
	for(i=0; i<4; i++)
	{
		for(k=0; k<8; k++)
			send_bit(g_b[i]>>(7-k));
		for(k=0; k<8; k++)
			send_bit(0);
		for(k=0; k<8; k++)
			send_bit(g_g[i]>>(7-k));
		for(k=0; k<8; k++)
			send_bit(0);
		for(k=0; k<8; k++)
			send_bit(g_r[i]>>(7-k));
		for(k=0; k<8; k++)
			send_bit(0);
	}
}

/// This is the interrupt handler - he is using it to handle the tube multiplexing
ISR(TIMER0_OVF_vect)
{
	PORTC = (PORTC & 0xf0) | 0xf; // cathodes off
	if(g_digit[g_scan]==0xf)
	{
		PORTD = (PORTD & 0xf); // digit off
	}
	else
	{
		PORTD = (PORTD & 0xf) | (1<<((g_scan)+4)); // digit on
		PORTC = (PORTC & 0xf0) | (g_digit[g_scan]); // cathode on
	}

	g_scan = (g_scan+1) & 3;
	g_timer++;
}

int main(void)
{
    /// DDRx are the Data Direction Registers. The m328p has 3 ports - A, B, and
    /// C. Each port has 8 pins. DDRx is a single byte register which stores,
    /// for port x, the I/O state for each pin n. The registers are indexed from
    /// right to left.
    ///
    /// Example:
    /// Imagine we have another port with 8 pins, called port F. We want pins
    /// 0, 1, 3, and 7 to be output pins, and the rest to be inputs. Then, the
    /// value stored in DDRF ought to be 10001011. (HIGH -> O / LOW -> I)
    ///
    ///
    /// PORTx are the Port Data Registers. They store the states of the pins,
    /// kinda.
    /// If some pin Pxn is configured as an input pin, then its state in
    /// PORTx determines its default state via an internal pull-up resistor.
    /// I.e., if we want one of our fake input pins from port F, say PF2, to
    /// be HIGH by default, we need an internal pull-up to do so, and this is
    /// done by setting the corresponding bit HIGH in PORTF.
    /// If some pin Pxn is configured as an output pin, then we can drive its
    /// output either HIGH or LOW by setting its corresponding bit in PORTx.
    ///
    /// Example:
    /// Using our same mythical port F, lets say of our input pins (2, 4, 5,
    /// 6) we want 2 and 4 to default LOW and 5 and 6 to default HIGH. In
    /// addition, we also want to drive output pins 1 and 3 HIGH, and sink 0 and
    /// 7 LOW. Then, the value we want to store in PORTF ought to be 01101010.

    /// Port B setup
	DDRB = (1<<PB1)|(1<<PB0);
                     /// Pins 0 and 1 are set as outputs
	PORTB = (1<<PB0);
                     /// Pin 0 is driven HIGH

                     /// PB0 - HV        LOW -> HV ON        HIGH -> HV OFF
                     /// PB1 - colon     LOW -> colon OFF    HIGH -> colon OFF
                     /// PB2 - SS        Slave select
                     /// PB3 - MOSI      Master out, slave in
                     /// PB4 - MISO      Master in, slave out
                     /// PB5 - SCK       Serial clock
                     /// PB6 \           These pins are for the external
                     /// PB7 /           crystal resonator

    /// Port C setup
	DDRC = (1<<PC5)|(1<<PC4)|(1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0);
                     /// Pins 0, 1, 2, 3, 4, 5 are set as outputs
	PORTC = (1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0);
                     /// Pins 0, 1, 2, 3 are driven HIGH.

                     /// PC0 - BCD A \
                     /// PC1 - BCD B  \ BCD is binary coded decimal. These pins
                     /// PC2 - BCD C  / encode the digit to display (CATHODE)
                     /// PC3 - BCD D /
                     /// PC4 - RGB LED SDTI     Serial data for RGB control
                     /// PC5 - RGB LED SCKI     Serial clock for RGB control
                     /// PC6 - RESET            Reset line for programming

    /// Port D setup
	DDRD = (1<<PD7)|(1<<PD6)|(1<<PD5)|(1<<PD4)|(1<<PD1);
                     /// Pins 1, 4, 5, 6, 7 are set as outputs
	PORTD = (1<<PD3);
                     /// Pin 3 is driven HIGH

                     /// PD0 - RX
                     /// PD1 - TX
                     /// PD2 - PD2      This is passed to a header (unused)
                     /// PD3 - PD3      This is passed to a header (unused)
                     /// PD4 - N1 \
                     /// PD5 - N2  \    These pins select which tube to display
                     /// PD6 - N3  /    on during multiplexing (ANODE)
                     /// PD7 - N4 /

	//CLKPR = 0x80;
	//CLKPR = 0; // 16mhz

	g_scan = 0;
	g_timer = 0;

    /// This section selects the color to backlight with

	u8 i;

	for(i=0; i<4; i++)
	{
		g_r[i] = 0xff;
		g_g[i] = 0x00;
		g_b[i] = 0x00;
		g_digit[i] = 0xf;
	}

	send_rgb();

	TCCR0A = 0;
	TCCR0B = 4; // free run prescale divide by 256
	TIMSK0 = 1; // enable overflow interrupt (16mhz / 32768 = 488Hz)
	sei();

	// voltmeter mode
    /// This basically says "use ADC6 as the ADC input pin, and use 5V as our
    /// fullscale reference"
	ADMUX = 6; //00000110 // ADC6 input, Vref = 5V
    /// This means that the ADC samples once every 128 clock cycles
	ADCSRA = 0x87; //10000111 // Prescaler 128

	PORTB &= ~(1<<PB0); // turn on HV

	while(1)
	{
        /// This is interrupt timing stuff for the multiplexing
		g_timer = 0;
		while(g_timer<128);

		ADCSRA |= 0x40; /// Tells the ADC to start sampling
		while(ADCSRA & 0x40); /// Waits until the ADC says it has finished

		u16 result = ADC; /// Store the result from the ADC
		u32 val = result * 8008UL;///     \ This converts the voltage to RPM
		val = val / 1024UL;       ///     / (0 V -> 0 RPM, 5 V -> 8008 RPM)

        /// This is all pretty clear - basically finding the value of each
        /// digit and sending it to the tubes for display.
        g_digit[0] = val % 10;
		val /= 10;
		if(val>0)
		{
			g_digit[1] = val % 10;
			val /= 10;
			if(val>0)
			{
				g_digit[2] = val % 10;
				val /= 10;
				if(val>0)
					g_digit[3] = val % 10;
				else
					g_digit[3] = 0xf;
			}
			else
			{
				g_digit[2] = 0xf;
				g_digit[3] = 0xf;
			}
		}
		else
		{
			g_digit[1] = 0xf;
			g_digit[2] = 0xf;
			g_digit[3] = 0xf;
		}
	}
}
