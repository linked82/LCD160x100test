/*
 * main.c
 *
 * Circuit for controlling the NHD-C160100DIZ display from Newhaven
 * Graphic controller ST7528
 * Platform MSP430 Launchpad
 * Processor MSP430G2553
 *
 *
 * Display control pinout
 * P1.7		SDA 	(pin 5)
 * P1.6		SCL 	(pin 4)
 * P2.5		RST		(pin 2)
 * P2.5		CSB		(pin 1)
 *
 */

#include "msp430g2553.h"
#include "arial.h"

/*
 * Definition for GPIO
 */
#define SET_RST		P2OUT|=BIT5
#define CLR_RST		P2OUT&=~BIT5
#define SET_CSB		P2OUT|=BIT4
#define CLR_CSB		P2OUT&=~BIT4


/*
 * Definition for Address bytes
 */
#define Comsend 0x00
#define Datasend 0x40




/*****************************************************/

const unsigned char Init_Vector1[] =
{
	0xA2,
	0xAE,
	0x48,	// Set Duty ratio
	0x64, //0x80, 	// nop
	0xA0,	// Set scan direction
	0xC8,	// SHL select
	0x40,	// initial Com0 register
	0x00,	// scan from Com0
	0xAB,	// OSC on
	0x64	// 3x
};

	// delay(2);

const unsigned char Init_Vector2[] =
{
	0x65	// 4x
};
	//  delay(2);

const unsigned char Init_Vector3[] =
{
	0x66	// 5x
};
	// delay(2);

const unsigned char Init_Vector4[] =
{
	0x67	// 6x
};
	// delay(2);

const unsigned char Init_Vector5[] =
{
	0x25,	// Select regulator resistor
	0x81,	// Select electronic volumen register
	12,
	0x50,	//0x57,  	// Select LCD BIAS
	0x92,	// frc and pwm
	0x2C
};

	// delay(200);
const unsigned char Init_Vector6[] =
{
	0x2E
};

	// delay(200);

const unsigned char Init_Vector7[] =
{

	0x2F
};

// delay(200);

const unsigned char Init_Vector8[] =
{
	0x92,
	0x38,
	0x75,
	0x97,	// 3frc, 45 pwm
	0x80,	// start 16-level grayscale settings
	0x00,
	0x81,
	0x00,
	0x82,
	0x00,
	0x83,
	0x00,
	0x84,
	0x06,
	0x85,
	0x06,
	0x86,
	0x06,
	0x87,
	0x06,
	0x88,
	0x0B,
	0x89,
	0x0B,
	0x8A,
	0x0B,
	0x8B,
	0x0B,
	0x8C,
	0x10,
	0x8D,
	0x10,
	0x8E,
	0x10,
	0x8F,
	0x10,
	0x90,
	0x15,
	0x91,
	0x15,
	0x92,
	0x15,
	0x93,
	0x15,
	0x94,
	0x1A,
	0x95,
	0x1A,
	0x96,
	0x1A,
	0x97,
	0x1A,
	0x98,
	0x1E,
	0x99,
	0x1E,
	0x9A,
	0x1E,
	0x9B,
	0x1E,
	0x9C,
	0x23,
	0x9D,
	0x23,
	0x9E,
	0x23,
	0x9F,
	0x23,
	0xA0,
	0x27,
	0xA1,
	0x27,
	0xA2,
	0x27,
	0xA3,
	0x27,
	0xA4,
	0x2B,
	0xA5,
	0x2B,
	0xA6,
	0x2B,
	0xA7,
	0x2B,
	0xA8,
	0x2F,
	0xA9,
	0x2F,
	0xAA,
	0x2F,
	0xAB,
	0x2F,
	0xAC,
	0x32,
	0xAD,
	0x32,
	0xAE,
	0x32,
	0xAF,
	0x32,
	0xB0,
	0x35,
	0xB1,
	0x35,
	0xB2,
	0x35,
	0xB3,
	0x35,
	0xB4,
	0x38,
	0xB5,
	0x38,
	0xB6,
	0x38,
	0xB7,
	0x38,
	0xB8,
	0x3A,
	0xB9,
	0x3A,
	0xBA,
	0x3A,
	0xBB,
	0x3A,
	0xBC,
	0x3C,
	0xBD,
	0x3C,
	0xBE,
	0x3C,
	0xBF,
	0x3C,	// end grayscale settings
	0x38,
	0x74,
	0xAF 	// display on

};




static void t_delay (unsigned long i)
{
	unsigned long j = i*1000;

	while (j>0)
	{
		__no_operation();
		j--;
	}
}


unsigned char * block_ptr = 0;
unsigned int block_length = 0;
unsigned int block_repeat = 0;

void block_tansfer_LCD(unsigned char A0, unsigned char * data, unsigned int length)
{

	block_ptr = data;
	block_length = length;

	IE2 |= UCB0TXIE;                          // Enable TX ready interrupt
	UCB0CTL1 |= UCTR + UCTXSTT;               // I2C TX, start condition
	UCB0TXBUF = A0;                        // Write DAC control byte
	__bis_SR_register(CPUOFF + GIE);          // Enter LPM0 w/ interrupts

	IE2 &= ~UCB0TXIE;                          // Disable TX ready interrupt

	while (UCB0CTL1 & UCTXSTT);             // Loop until I2C STT is sent
	UCB0CTL1 |= UCTXSTP;                    // I2C stop condition after 1st TX
	__bic_SR_register(GIE);          // Enter LPM0 w/ interrupts
}


void init_LCD(void)
{
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector1, 10);
	t_delay(2);
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector2, 1);
	t_delay(2);
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector3, 1);
	t_delay(2);
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector4, 1);
	t_delay(2);
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector5, 6);
	t_delay(200);
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector6, 1);
	t_delay(200);
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector7, 1);
	t_delay(200);
	block_tansfer_LCD(Comsend, (unsigned char *) Init_Vector8, 135);
	t_delay(2);

}

volatile unsigned char dbuffer[160];
const unsigned char dummy[] = "Hola mundo";

void write_line(unsigned char *text)
{
	unsigned char n=0;
	unsigned int i=0;
	unsigned char k=0;


	while((text[n]!='\0')&&(i<155))
	{
		for(k=0;k<arial_6ptDescriptors[text[n]-0x20][0];k++)
		{
			dbuffer[i]=	arial_6ptBitmaps[arial_6ptDescriptors[text[n]-0x20][1]+k];
			i++;
		}
		dbuffer[i]=0;
		i++;
		n++;
	}
}

void send_line(unsigned char line)
{
	unsigned char page_column[] = {0xB0,0x10,0x01};
	page_column[0] = 0xB0+(line&0x0F);

	// Send page (line) and line Y9:Y2 (column) to start
	block_tansfer_LCD(Comsend, (unsigned char *) page_column, 3);

	// Send line
	block_repeat = 1;
	block_tansfer_LCD(Datasend, (unsigned char *) dbuffer, 160);
}


/*****************************************************/

void main(void) {
	
	WDTCTL = WDTPW + WDTHOLD;                 // Stop Watchdog Timer

	P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
	P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

	P2SEL &= ~(BIT4 + BIT5);
	P1SEL &= ~(BIT4 + BIT5);
	P2DIR |= (BIT4 + BIT5);
	P2OUT &= ~(BIT5);
	P2OUT |= BIT4;

	UCB0CTL1 |= UCSWRST;                      // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
	UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
	UCB0BR1 = 0;
	UCB0I2CSA = 0x3F;                         // Set slave address
	UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

	CLR_RST;
	CLR_CSB;
	t_delay(100);
	SET_RST;
	t_delay(100);
	SET_CSB;
	t_delay(100);

	// Las interrupciones se habilitan dentro
	init_LCD();

	write_line((unsigned char *) dummy);

	send_line(0);
	send_line(1);
	send_line(2);
	send_line(3);
	send_line(4);
	send_line(5);

	__bis_SR_register(CPUOFF);          // Enter LPM0 w/ interrupts

}



#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  static unsigned int ByteCtr;

  if(block_repeat)
  {
	  UCB0TXBUF = block_ptr[ByteCtr>>2];
	  ByteCtr++;

	  if(ByteCtr>(block_length<<2))
	  {
		  __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
		  ByteCtr=0;
		  block_repeat = 0;
	  }
  }
  else
  {
	  UCB0TXBUF = block_ptr[ByteCtr++];          // Transmit data byte

	  if(ByteCtr>block_length)
	  {
		  __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
		  ByteCtr=0;
	  }
  }
}
