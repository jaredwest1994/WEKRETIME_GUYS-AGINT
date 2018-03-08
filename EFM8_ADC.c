// ADC.c:  Shows how to use the 14-bit ADC.  This program
// measures the voltage from some pins of the EFM8LB1 using the ADC.
//
// (c) 2008-2018, Jesus Calvino-Fraga
//

#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include <math.h>

// ~C51~  

#define SYSCLK 72000000L
#define BAUDRATE 115200L

#define LCD_RS P2_6
// #define LCD_RW Px_x // Not used in this code.  Connect to GND
#define LCD_E  P2_5
#define LCD_D4 P2_4
#define LCD_D5 P2_3
#define LCD_D6 P2_2
#define LCD_D7 P2_1
#define CHARS_PER_LINE 16

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
  	
	return 0;
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADC0CN1 = 0b_10_000_000; //14-bit,  Right justified no shifting applied, perform and Accumulate 1 conversion.
	ADC0CF0 = 0b_11111_0_00; // SYSCLK/32
	ADC0CF1 = 0b_0_0_011110; // Same as default for now
	ADC0CN0 = 0b_0_0_0_0_0_00_0; // Same as default for now
	ADC0CF2 = 0b_0_01_11111 ; // GND pin, Vref=VDD
	ADC0CN2 = 0b_0_000_0000;  // Same as default for now. ADC0 conversion initiated on write of 1 to ADBUSY.
	ADEN=1; // Enable ADC
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned int us)
{
	unsigned int i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;

	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}

#define VDD 3.309 // The measured value of VDD in volts

void LCD_pulse (void)
{
	LCD_E=1;
	Timer3us(40);
	LCD_E=0;
}

void LCD_byte (unsigned char x)
{
	// The accumulator in the C8051Fxxx is bit addressable!
	ACC=x; //Send high nible
	LCD_D7=ACC_7;
	LCD_D6=ACC_6;
	LCD_D5=ACC_5;
	LCD_D4=ACC_4;
	LCD_pulse();
	Timer3us(40);
	ACC=x; //Send low nible
	LCD_D7=ACC_3;
	LCD_D6=ACC_2;
	LCD_D5=ACC_1;
	LCD_D4=ACC_0;
	LCD_pulse();
}

void WriteData (unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT (void)
{
	LCD_E=0; // Resting state of LCD's enable is zero
	// LCD_RW=0; // We are only writing to the LCD in this program
	waitms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, bit clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	waitms(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

int getsn (char * buff, int len)
{
	int j;
	char c;
	
	for(j=0; j<(len-1); j++)
	{
		c=getchar();
		if ( (c=='\n') || (c=='\r') )
		{
			buff[j]=0;
			return j;
		}
		else
		{
			buff[j]=c;
		}
	}
	buff[j]=0;
	return len;
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0://can not use P0.0 and P0.3 for analog input
			P0MDIN &= (~mask); // Set pin as analog input
			//P0|=mask; // Set the bit associated with the pin in the Pn register to 1
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			//P1|=mask; // Set the bit associated with the pin in the Pn register to 1
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2://can not use P2.0 and P2.7 for analog input
			P2MDIN &= (~mask); // Set pin as analog input
			//P2|=mask; // Set the bit associated with the pin in the Pn register to 1
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}


unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADBUSY=1;       // Dummy conversion first to select new pin
	while (ADBUSY); // Wait for dummy conversion to finish
	ADBUSY = 1;     // Convert voltage at the pin
	while (ADBUSY); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/16383.0);
}

/********************************************
*											*
* 	Runs Timer0 To Calculate Half Period	*
*		in Units of 12/SYSCLK				*
*											*
********************************************/
float CALC_Half_Period(unsigned char pin)
{
	float half_period = 0;
	
// Start tracking the reference signal
	ADC0MX=pin;
	ADBUSY=1;
	while (ADBUSY); // Wait for conversion to complete
//set timer 0 as a 16 bit timer
	TMOD &= 0B_1111_0000;
	TMOD |= 0B_0000_0001;
//Reset the timer
	TL0=0; TH0=0;
	while (Volts_at_Pin(pin)!=0); // Wait for the signal to be zero
	while (Volts_at_Pin(pin)==0); // Wait for the signal to be positive
	TR0=1; // Start the timer 0
	while (Volts_at_Pin(pin)!=0); // Wait for the signal to be zero again
	TR0=0; // Stop timer 0
	return half_period=(float)(TH0*256.0+TL0); // The 16-bit number [TH0-TL0]
}


/************************************************
*												*
*	Returns the Period as measured at PX_X		*
*												*
************************************************/
float Period_at_Pin(unsigned char pin)
{
 return 1000.0*(CALC_Half_Period(pin)*(12.0/(float)SYSCLK))*2.0;
}

/********************************************
*											*
* Measures the Quarter Period of the Wave   *
*	will be used to calculate peak voltage	*
*											*
********************************************/
float Quart_Period(unsigned char pin)
{
 return CALC_Half_Period(pin)/2;
}

/****************************************************
*													*
*	Calculates the Peak Voltage of the Sine Wave	*
*													*
****************************************************/
float Calc_Peak_Voltage(unsigned char pin)
{

	float voltage=0;
	float quarter_period = Period_at_Pin(pin)*1000/4;

//Set ADC to Read the correct pin
	ADC0MX=pin;
	ADBUSY=1;
	while (ADBUSY);//wait for the conversion to complete
	while (Volts_at_Pin(pin)!=0); // Wait for the signal to be zero
	while (Volts_at_Pin(pin)==0); // Wait for the signal to be positive
//delay by quarter_period before measuring the voltage
	Timer3us(quarter_period);
	voltage = Volts_at_Pin(pin);
	return voltage;	
}

float Phase_Difference(unsigned char pin1, unsigned char pin2, float period)
{
	float time_between=0;
	ADBUSY=1;
	while (ADBUSY);//wait for the conversion to complete
	TR0=0;
	TL0=0;TH0=0;
	while(Volts_at_Pin(pin1)!=0 && Volts_at_Pin(pin2) !=0);
		if(Volts_at_Pin(pin1)==0){
			while(Volts_at_Pin(pin1)==0);
			TR0=1;
			P1_5=1;
			while(Volts_at_Pin(pin2)==0);
			TR0=0;
			P1_5=0;
			time_between=(TH0*256.0+TL0)*(12.0/(float)SYSCLK);
			return time_between*1000*(360.0/period);
		}
		else{
			while(Volts_at_Pin(pin2)==0);
			TR0=1;
			P1_5=1;
			while(Volts_at_Pin(pin1)==0);
			TR0=0;
			P1_5=0;
			time_between=(TH0*256.0+TL0)*(12.0/(float)SYSCLK);
			return time_between*1000*(360.0/period)*(-1.0);
		}
//	time_between=(TH0*256.0+TL0)*(12.0/(float)SYSCLK);
//	return time_between*1000*(360.0/period);
}

void main (void)
{
	float v[2];
	float vrms[2];
	float Period[2];
	float quarter_period = 0;
	float phase_diff = 0;
	float frequency = 0;
	char buffer[16];
	LCD_4BIT ();
	
    waitms(500); // Give PuTTy a chance to start before sending
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	
	printf ("Magnitude and Phase test program\n"
	        "File: %s\n"
	        "Compiled: %s, %s\n\n",
	        __FILE__, __DATE__, __TIME__);
	
	InitPinADC(1, 7); // Configure P1.7 as analog input
	InitPinADC(1, 6); // Configure P1.6 as analog input

    InitADC();
	
	while(1)
	{
	    // Read 14-bit value from the pins configured as analog inputs
		Period[0] = Period_at_Pin(QFP32_MUX_P1_7);
		Period[1] = Period_at_Pin(QFP32_MUX_P1_6);
		v[0] = Calc_Peak_Voltage(QFP32_MUX_P1_7);
		v[1] = Calc_Peak_Voltage(QFP32_MUX_P1_6);
		vrms[0] = v[0]/1.414214;
		vrms[1] = v[1]/1.414214;
		phase_diff = Phase_Difference(QFP32_MUX_P1_7, QFP32_MUX_P1_6, Period[0]);
		frequency = 1.0/(Period[0]/1000.0);
		sprintf(buffer,"R %.3fVrms  P", vrms[0]);
		LCDprint(buffer,1,1);
		sprintf(buffer,"T %.3fVrms %2.1f", vrms[1],phase_diff);
		LCDprint(buffer,2,1);
		printf ("f=%.2fHz, V@1.7=%.4fVrms, V@1.6=%.4fVrms, diff=%.4fdegrees\r", frequency,vrms[0],vrms[1], phase_diff);
		waitms(500);
	 }  
}