/*
 * main.c
 */

#include "msp430g2553.h"

#define SET_RST		P2OUT|=BIT5
#define CLR_RST		P2OUT&=~BIT5
#define SET_CSB		P2OUT|=BIT4
#define CLR_CSB		P2OUT&=~BIT4

const char Slave = 0x7E;
const char Comsend = 0x00;
const char Datasend = 0x40;

unsigned char I2C_Tx=0;
void I2C_out(unsigned char data) //I2C Output
{

/*	I2C_Tx = data;
	IE2 |= UCB0TXIE;                          // Enable TX ready interrupt
	__bis_SR_register(CPUOFF + GIE);          // Enter LPM0 w/ interrupts
*/
	while (UCB0STAT & UCBUSY);
	UCB0TXBUF = data;

}

/*****************************************************/
void I2C_Start(void)
{
	UCB0CTL1 |= UCTR + UCTXSTT;               // I2C TX, start condition
    //while (UCB0CTL1 & UCTXSTT);             // Loop until I2C STT is sent
}
/*****************************************************/
void I2C_Stop(void)
{
	//while (UCB0CTL1 & UCTXSTT);             // Loop until I2C STT is sent
	UCB0CTL1 |= UCTXSTP;                    // I2C stop condition after 1st TX
}

/*****************************************************/

void delay (unsigned char i)
{
	unsigned long j = i*100;

	while (j) j--;
}


/*****************************************************/

void init_LCD(void)
{
	I2C_Start();
	//I2C_out(Slave);
	I2C_out(Comsend);
	I2C_out(0x48); //partial display duty ratio
	I2C_out(0x64); // 1/100 duty
	I2C_out(0xA0); //ADC select
	I2C_out(0xC8); //SHL select
	I2C_out(0x44); //initial Com0 register
	I2C_out(0x00); //scan from Com0
	I2C_out(0xAB); //OSC on
	I2C_out(0x26); //
	I2C_out(0x81); //set electronic volume
	I2C_out(0x15); //vopcode=0x1C
	I2C_out(0x56); //set 1/11 bias
	I2C_out(0x64); //3x
	delay(2);
	I2C_out(0x2C); //
	I2C_out(0x66); //5x
	delay(2);
	I2C_out(0x2E); //
	delay(2);
	I2C_out(0x2F); //power control
	I2C_out(0xF3); //bias save circuit
	I2C_out(0x00); //
	I2C_out(0x96); //frc and pwm
	I2C_out(0x38); //external mode
	I2C_out(0x75); //
	I2C_out(0x97); //3frc, 45 pwm
	I2C_out(0x80); //start 16-level grayscale settings
	I2C_out(0x00); //
	I2C_out(0x81); //
	I2C_out(0x00); //
	I2C_out(0x82); //
	I2C_out(0x00); //
	I2C_out(0x83); //
	I2C_out(0x00); //
	I2C_out(0x84); //
	I2C_out(0x06); //
	I2C_out(0x85); //

	I2C_out(0x06); //
	I2C_out(0x86); //
	I2C_out(0x06); //
	I2C_out(0x87); //
	I2C_out(0x06); //
	I2C_out(0x88); //
	I2C_out(0x0B); //
	I2C_out(0x89); //
	I2C_out(0x0B); //
	I2C_out(0x8A); //
	I2C_out(0x0B); //
	I2C_out(0x8B); //
	I2C_out(0x0B); //
	I2C_out(0x8C); //
	I2C_out(0x10); //
	I2C_out(0x8D); //
	I2C_out(0x10); //
	I2C_out(0x8E); //
	I2C_out(0x10); //
	I2C_out(0x8F); //
	I2C_out(0x10); //
	I2C_out(0x90); //
	I2C_out(0x15); //
	I2C_out(0x91); //
	I2C_out(0x15); //
	I2C_out(0x92); //
	I2C_out(0x15); //
	I2C_out(0x93); //
	I2C_out(0x15); //
	I2C_out(0x94); //
	I2C_out(0x1A); //
	I2C_out(0x95); //
	I2C_out(0x1A); //
	I2C_out(0x96); //
	I2C_out(0x1A); //
	I2C_out(0x97); //
	I2C_out(0x1A); //
	I2C_out(0x98); //
	I2C_out(0x1E); //
	I2C_out(0x99); //
	I2C_out(0x1E); //
	I2C_out(0x9A); //
	I2C_out(0x1E); //
	I2C_out(0x9B); //
	I2C_out(0x1E); //
	I2C_out(0x9C); //
	I2C_out(0x23); //
	I2C_out(0x9D); //
	I2C_out(0x23); //
	I2C_out(0x9E); //
	I2C_out(0x23); //
	I2C_out(0x9F); //
	I2C_out(0x23); //
	I2C_out(0xA0); //
	I2C_out(0x27); //
	I2C_out(0xA1); //
	I2C_out(0x27); //
	I2C_out(0xA2); //
	I2C_out(0x27); //
	I2C_out(0xA3); //
	I2C_out(0x27); //
	I2C_out(0xA4); //
	I2C_out(0x2B); //

	I2C_out(0xA5); //
	I2C_out(0x2B); //
	I2C_out(0xA6); //
	I2C_out(0x2B); //
	I2C_out(0xA7); //
	I2C_out(0x2B); //
	I2C_out(0xA8); //
	I2C_out(0x2F); //
	I2C_out(0xA9); //
	I2C_out(0x2F); //
	I2C_out(0xAA); //
	I2C_out(0x2F); //
	I2C_out(0xAB); //
	I2C_out(0x2F); //
	I2C_out(0xAC); //
	I2C_out(0x32); //
	I2C_out(0xAD); //
	I2C_out(0x32); //
	I2C_out(0xAE); //
	I2C_out(0x32); //
	I2C_out(0xAF); //
	I2C_out(0x32); //
	I2C_out(0xB0); //
	I2C_out(0x35); //
	I2C_out(0xB1); //
	I2C_out(0x35); //
	I2C_out(0xB2); //
	I2C_out(0x35); //
	I2C_out(0xB3); //
	I2C_out(0x35); //
	I2C_out(0xB4); //
	I2C_out(0x38); //
	I2C_out(0xB5); //
	I2C_out(0x38); //
	I2C_out(0xB6); //
	I2C_out(0x38); //
	I2C_out(0xB7); //
	I2C_out(0x38); //
	I2C_out(0xB8); //
	I2C_out(0x3A); //
	I2C_out(0xB9); //
	I2C_out(0x3A); //
	I2C_out(0xBA); //
	I2C_out(0x3A); //
	I2C_out(0xBB); //
	I2C_out(0x3A); //
	I2C_out(0xBC); //
	I2C_out(0x3C); //
	I2C_out(0xBD); //
	I2C_out(0x3C); //
	I2C_out(0xBE); //
	I2C_out(0x3C); //
	I2C_out(0xBF); //
	I2C_out(0x3C); //end grayscale settings
	I2C_out(0x38); //
	I2C_out(0x74); //
	I2C_out(0xAF); //display on
	I2C_Stop();
}


/*****************************************************/

void main(void) {
	
	WDTCTL = WDTPW + WDTHOLD;                 // Stop Watchdog Timer

	P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
	P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

	P2SEL &= ~(BIT4 + BIT5);
	P1SEL &= ~(BIT4 + BIT5);
	P2DIR |= (BIT4 + BIT5);
	P2OUT &= ~(BIT4 + BIT5);

	UCB0CTL1 |= UCSWRST;                      // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
	UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
	UCB0BR1 = 0;
	UCB0I2CSA = 0x3F;                         // Set slave address
	UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

	init_LCD();

	__bis_SR_register(CPUOFF);          // Enter LPM0 w/ interrupts

}







// USCI_B0 Data ISR
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  UCB0TXBUF = I2C_Tx;
  IE2 &= ~UCB0TXIE;
  __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
}
