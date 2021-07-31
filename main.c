// Author: Vatsal Asitkumar Joshi
// Date: Oct 1st, 2019
// This code changes the LED on the device from red to green to blue to red again.
// However, it uses bitband aliases to achieve it.
//
// "If you are done writing the code, now is a good time to debug it."
//

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tm4c123gh6pm.h"

// Function definition of waitMicroSeconds written in assembly language
extern void waitMicroSeconds(uint32_t ms);

// Macros that can be used to set a specific bit
#define bit0    0x1
#define bit1    0x2
#define bit2    0x4
#define bit3    0x8
#define bit4    0x10
#define bit5    0x20
#define bit6    0x40
#define bit7    0x80
#define bit8    0x100
#define bit9    0x200
#define bit10   0x400
#define bit11   0x800
#define bit12   0x1000
#define bit13   0x2000
#define bit14   0x4000
#define bit15   0x8000
#define bit16   0x10000
#define bit17   0x20000
#define bit18   0x40000
#define bit19   0x80000
#define bit20   0x100000
#define bit21   0x200000
#define bit22   0x400000
#define bit23   0x800000
#define bit24   0x1000000
#define bit25   0x2000000
#define bit26   0x4000000
#define bit27   0x8000000
#define bit28   0x10000000
#define bit29   0x20000000
#define bit30   0x40000000
#define bit31   0x80000000
#define bit32	0x100000000

//	Macro to convert a register and bit number to respective bitband alias
#define peripheralBitBand(regAddress,bitNumber) (*((volatile uint32_t *) (0x42000000 + ((uint32_t)&regAddress-0x40000000)*32 + bitNumber*4)))

#define setRegisterBit(regAddress,bitNumber)	\
{												\
	regAddress |= (bitNumber);					\
}

#define clrRegisterBit(regAddress,bitNumber)	\
{												\
	regAddress &= (~(bitNumber));				\
}

#define flpRegisterBit(regAddress,bitNumber)	\
{												\
	regAddress ^= (bitNumber);					\
}

#define setRegisterVal(regAddress,value)		\
{												\
	regAddress = value;							\
}

// Macros and function definitions for UART0
#define MAX_CHARS 80
#define MAX_FIELDS 6

struct termInput
{
	char str[MAX_CHARS+1];
	uint8_t pos[MAX_FIELDS], argCount;
};

void parseString(struct termInput* userInput);
char* getArgString(struct termInput* userInput, uint8_t argNumber);

#define initUART0()																																			\
{																																							\
	setRegisterBit(SYSCTL_RCGC2_R, SYSCTL_RCGC2_GPIOA);																										\
	setRegisterBit(SYSCTL_RCGCUART_R, SYSCTL_RCGCUART_R0);										/*Activate UART0*/											\
																																							\
	setRegisterBit(GPIO_PORTA_DIR_R, bit1);														/*Enable output on UART0 TX pin*/							\
	setRegisterBit(GPIO_PORTA_DEN_R, bit0 | bit1);												/*Enable digital I/O on UART0 pins*/						\
	setRegisterBit(GPIO_PORTA_AFSEL_R, bit0 | bit1);											/*Use peripheral to drive PA0, PA1 as UART0*/				\
	clrRegisterBit(GPIO_PORTA_PCTL_R, bit0 | bit1 | bit2 | bit3 | bit4 | bit5 | bit6 | bit7);	/*Set fields for PA0 and PA1 to zero*/						\
	setRegisterBit(GPIO_PORTA_PCTL_R, bit0 | bit4);												/*Assign UART signals to pin PA0 and PA1*/					\
																																							\
	clrRegisterBit(UART0_CTL_R, UART_CTL_UARTEN);												/*Turn-off UART0 to allow safe programming*/				\
	setRegisterVal(UART0_IBRD_R, 21);															/*BRD = BRDI + BRDF = Clk / (16 * Baud Rate)*/				\
	setRegisterVal(UART0_FBRD_R, 45);															/*BRDI = floor(BRD) and BRDF = floor((BRD-BRDI)*64+0.5)*/	\
	setRegisterBit(UART0_LCRH_R, UART_LCRH_WLEN_8 | UART_LCRH_FEN);								/*Configure for 8N1 w/ 16-level FIFO*/						\
	clrRegisterBit(UART0_CC_R, bit0 | bit1 | bit2 | bit3);										/*Set the clock source to zero (For safety)*/				\
	setRegisterBit(UART0_CTL_R, UART_CTL_UARTEN);												/*Turn on UART0*/											\
}

void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
void getsUart0(struct termInput* userInput);

// Macros and function definitions to initialize ADC0 and ADC1 and read voltages at AIN0 and AIN1
#define ADC0_CALI (3.3/3.2166)
#define ADC1_CALI (3.3/3.252)

#define initAIN0()																										\
{																														\
	setRegisterBit(SYSCTL_RCGC2_R, SYSCTL_RCGC2_GPIOE);																	\
	setRegisterBit(SYSCTL_RCGCADC_R, SYSCTL_RCGCADC_R0);		/* Activate ADC0								*/		\
																														\
	clrRegisterBit(GPIO_PORTE_DEN_R, bit3);						/* Disable digital I/O on PE3 pin				*/		\
	setRegisterBit(GPIO_PORTE_AFSEL_R, bit3);					/* Use alternate function on PE3 pin			*/		\
	setRegisterBit(GPIO_PORTE_AMSEL_R, bit3);					/* turn on analog operation on pin PE3			*/		\
																														\
	setRegisterVal(ADC0_CC_R, ADC_CC_CS_SYSPLL);				/* select PLL as the time base (default value)	*/		\
	clrRegisterBit(ADC0_ACTSS_R, ADC_ACTSS_ASEN3);				/* disable SS3 for safe set‐up					*/		\
	setRegisterBit(ADC0_PC_R, ADC_PC_SR_125K);					/* select 125Ksps rate							*/		\
    setRegisterBit(ADC0_SAC_R, ADC_SAC_AVG_64X);				/* 64-sample HW averaging						*/		\
	clrRegisterBit(ADC0_SSMUX3_R, bit0 | bit1 | bit2 | bit3);	/* set first sample to AIN0						*/		\
	setRegisterBit(ADC0_SSCTL3_R, ADC_SSCTL3_END0);				/* mark first sample as the last sample			*/		\
	setRegisterBit(ADC0_ACTSS_R, ADC_ACTSS_ASEN3);				/* enable SS3 for operation						*/		\
}

#define initAIN1()																										\
{																														\
	setRegisterBit(SYSCTL_RCGC2_R, SYSCTL_RCGC2_GPIOE);																	\
	setRegisterBit(SYSCTL_RCGCADC_R, SYSCTL_RCGCADC_R1);		/* Activate ADC1								*/		\
																														\
	clrRegisterBit(GPIO_PORTE_DEN_R, bit2);						/* Disable digital I/O on PE3 pin				*/		\
	setRegisterBit(GPIO_PORTE_AFSEL_R, bit2);					/* Use alternate function on PE3 pin			*/		\
	setRegisterBit(GPIO_PORTE_AMSEL_R, bit2);					/* turn on analog operation on pin PE3			*/		\
																														\
	setRegisterVal(ADC1_CC_R, ADC_CC_CS_SYSPLL);				/* select PLL as the time base (default value)	*/		\
	clrRegisterBit(ADC1_ACTSS_R, ADC_ACTSS_ASEN3);				/* disable SS3 for safe set‐up					*/		\
	setRegisterBit(ADC1_PC_R, ADC_PC_SR_125K);					/* select 125Ksps rate							*/		\
    setRegisterBit(ADC1_SAC_R, ADC_SAC_AVG_64X);				/* 64-sample HW averaging						*/		\
	setRegisterBit(ADC1_SSMUX3_R, bit0);						/* set first sample to AIN1						*/		\
	setRegisterBit(ADC1_SSCTL3_R, ADC_SSCTL3_END0);				/* mark first sample as the last sample			*/		\
	setRegisterBit(ADC1_ACTSS_R, ADC_ACTSS_ASEN3);				/* enable SS3 for operation						*/		\
}

float readAIN0();
float readAIN1();
void iirAnalog(float* vFiltered, uint8_t channel);
void ftoa(float fVal, char* str, int strLength);

// Macros to initialize the SPI interface and write values to the DAC.
#define SSI2_Clk bit4
#define SSI2_Tx  bit7

#define	DAC_SHDN bit12
#define DAC_GA   bit13
#define	DAC_OUT  bit15

#define DAC_LDAC_MASK bit7
#define DAC_CS_MASK   bit5

#define DAC_LDAC peripheralBitBand(GPIO_PORTA_DATA_R, 7)
#define DAC_CS	 peripheralBitBand(GPIO_PORTB_DATA_R, 5)

#define dacASlope	-383.0
#define dacAInter	2017.0
#define dacBSlope	-384.0
#define dacBInter	2025.0

#define initDAC()																								\
{																												\
	setRegisterBit(SYSCTL_RCGC2_R, SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOA);									\
	setRegisterBit(SYSCTL_RCGCSSI_R, SYSCTL_RCGCSSI_R2);														\
																												\
	setRegisterBit(GPIO_PORTB_DIR_R, SSI2_Clk | DAC_CS_MASK | SSI2_Tx);											\
	setRegisterBit(GPIO_PORTB_AFSEL_R, SSI2_Clk | DAC_CS_MASK | SSI2_Tx);										\
	setRegisterBit(GPIO_PORTB_PCTL_R, GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB5_SSI2FSS);	\
	setRegisterBit(GPIO_PORTB_DEN_R, SSI2_Clk | DAC_CS_MASK | SSI2_Tx);											\
	setRegisterBit(GPIO_PORTB_PUR_R, SSI2_Clk | DAC_CS_MASK);													\
																												\
	setRegisterBit(GPIO_PORTA_DIR_R, DAC_LDAC_MASK);															\
	setRegisterBit(GPIO_PORTA_DEN_R, DAC_LDAC_MASK);															\
	setRegisterBit(GPIO_PORTA_PUR_R, DAC_LDAC_MASK);															\
																												\
	clrRegisterBit(SSI2_CR1_R, SSI_CR1_SSE);																	\
	clrRegisterBit(SSI2_CR1_R, SSI_CR1_MS);																		\
	setRegisterBit(SSI2_CC_R, SSI_CC_CS_SYSPLL);																\
	setRegisterBit(SSI2_CPSR_R, 10);																			\
	setRegisterBit(SSI2_CR0_R, SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16);												\
	setRegisterBit(SSI2_CR1_R, SSI_CR1_SSE);																	\
																												\
	DAC_LDAC = 1;																								\
}

#define putDACOutA(VoutA)								\
{														\
	clrRegisterBit(VoutA, DAC_OUT);						\
	setRegisterBit(VoutA, DAC_GA | DAC_SHDN);			\
	SSI2_DR_R = VoutA;									\
}
#define putDACOutB(VoutB)								\
{														\
	setRegisterBit(VoutB, DAC_OUT);						\
	setRegisterBit(VoutB, DAC_GA | DAC_SHDN);			\
	SSI2_DR_R = VoutB;									\
}

#define convVoutA(VoutA) (uint16_t)(dacASlope*(VoutA) + dacAInter)
#define convVoutB(VoutB) (uint16_t)(dacBSlope*(VoutB) + dacBInter)
#define convVout2Dac(Vout, channel) (uint16_t)((channel-1) ? (dacBSlope*(Vout) + dacBInter) : (dacASlope*(Vout) + dacAInter))
#define convDac2Vout(Dac, channel) ((channel-1) ? ((Dac - dacBInter)/dacBSlope) : ((Dac - dacAInter)/dacASlope))

// Macro to initialize the timer
#define initTimer0()													\
{																		\
    setRegisterBit(SYSCTL_RCGCTIMER_R, SYSCTL_RCGCTIMER_R0);			\
																		\
    clrRegisterBit(TIMER0_CTL_R, TIMER_CTL_TAEN | TIMER_CTL_TBEN);		\
    setRegisterBit(TIMER0_CFG_R, TIMER_CFG_32_BIT_TIMER);				\
    setRegisterBit(TIMER0_TAMR_R, TIMER_TAMR_TAMR_PERIOD);				\
    setRegisterVal(TIMER0_TAILR_R, 400); /*400*/						\
    setRegisterBit(TIMER0_IMR_R, TIMER_IMR_TATOIM);						\
    setRegisterBit(NVIC_EN0_R, 1 << (INT_TIMER0A - 16));				\
    setRegisterBit(TIMER0_CTL_R, TIMER_CTL_TAEN);						\
}

// Structure for signal information and macro to generate the Look-up tables
#define	M_PI		3.14159265358979323846

#define swapData(data1,data2)				\
{											\
	uint16_t* tempData = data1;				\
	data1 = data2;							\
	data2 = tempData;						\
}

enum signalType {DirectCurrent = 2, Sinusoidal, Square, Sawtooth, Triangle, Differential};
struct signalInfo
{
	enum signalType type;
	bool state;
	float bias, amp, frq;
	uint32_t phi, dPhi;
	uint16_t* data;
	uint32_t nCycles, cycleCount;
	float dutyCycle;
};

#define generateSignal(signal, buffer, channel, uartInput)															\
{																													\
	switch (signal.type)																							\
	{																												\
	case DirectCurrent:																								\
        signal.bias = atof(getArgString(uartInput,2));																\
        signal.amp = 0; signal.frq = 0; signal.dPhi = 0;															\
		for (uint16_t i = 0; i<4096; ++i)																			\
			buffer[i] = convVout2Dac(signal.bias, channel);															\
		break;																										\
	case Sinusoidal:																								\
        signal.bias = atof(getArgString(uartInput,4));																\
        signal.amp = fabs(atof(getArgString(uartInput,3)));															\
        signal.frq = atof(getArgString(uartInput,2));																\
        signal.dPhi = bit32/100000.0*signal.frq;																	\
		for (uint16_t i = 0; i<4096; ++i)																			\
		{																											\
			buffer[i] = convVout2Dac(signal.bias + signal.amp*sin(2*M_PI/4096*i), channel);							\
		}																											\
		break;																										\
	case Square:																									\
        signal.bias = atof(getArgString(uartInput,4));																\
        signal.amp = fabs(atof(getArgString(uartInput,3)));															\
        signal.frq = atof(getArgString(uartInput,2));																\
        signal.dPhi = bit32/100000.0*signal.frq;																	\
        signal.dutyCycle = atof(getArgString(uartInput,5)) ? atof(getArgString(uartInput,5)) : 50;					\
		for (uint16_t i = 0; i<(uint16_t)signal.dutyCycle*4096/100; ++i)											\
			buffer[i] = convVout2Dac(signal.bias + signal.amp, channel);											\
		for (uint16_t i = (uint16_t)signal.dutyCycle*4096/100; i<4096; ++i)											\
			buffer[i] = convVout2Dac(signal.bias, channel);															\
		break;																										\
	case Sawtooth:																									\
        signal.bias = atof(getArgString(uartInput,4));																\
        signal.amp = fabs(atof(getArgString(uartInput,3)));															\
        signal.frq = atof(getArgString(uartInput,2));																\
        signal.dPhi = bit32/100000.0*signal.frq;																	\
		for (uint16_t i = 0; i<4096; ++i)																			\
			buffer[i] = convVout2Dac(signal.bias + signal.amp/4096*i, channel);										\
		break;																										\
	case Triangle:																									\
        signal.bias = atof(getArgString(uartInput,4));																\
        signal.amp = fabs(atof(getArgString(uartInput,3)));															\
        signal.frq = atof(getArgString(uartInput,2));																\
        signal.dPhi = bit32/100000.0*signal.frq;																	\
		for (uint16_t i = 0; i<4096; ++i)																			\
			if (i<2048)																								\
				buffer[i] = convVout2Dac(signal.bias + signal.amp/2048*i, channel);									\
			else																									\
				buffer[i] = convVout2Dac(signal.bias + signal.amp/2048*(4096-i), channel);							\
		break;																										\
	}																												\
	swapData(signal.data,buffer);																					\
}

// Macro to calculate the gain of a circuit connected to an output
#define gain(signal, frq1, frq2)															\
float iirVal1, iirVal2;																		\
char strGain[10], strFrq[10];																\
putsUart0("Frequency(Hz)\tGain(Vout/Vin)\r\n");												\
putsUart0("----------------------------------\r\n");										\
iirAnalog(&iirVal1,1); iirAnalog(&iirVal2,2);												\
iirVal2 = iirVal2/iirVal1;																	\
ftoa(iirVal2, strGain, 10); ftoa(signal.frq, strFrq, 10);									\
putsUart0(strFrq); putsUart0("\t"); putsUart0(strGain); putsUart0("\r\n");					\
																							\
for (uint32_t i = 1; i<100000; i *= 10)														\
	for (uint32_t j = 1; j<10; ++j)															\
	{																						\
		signal.frq = frq1 + i*j;															\
		if (signal.frq >= frq2)																\
			break;																			\
		signal.dPhi = bit32/100000.0*signal.frq;											\
		iirAnalog(&iirVal1,1); iirAnalog(&iirVal2,2);										\
		iirVal2 = iirVal2/iirVal1;															\
		ftoa(iirVal2, strGain, 10); ftoa(signalA.frq, strFrq, 10);							\
		putsUart0(strFrq); putsUart0("\t"); putsUart0(strGain); putsUart0("\r\n");			\
	}																						\
if (signal.frq == frq2)																		\
{																							\
	signal.dPhi = bit32/100000.0*signal.frq;												\
	iirAnalog(&iirVal1,1); iirAnalog(&iirVal2,2);											\
	iirVal2 = iirVal2/iirVal1;																\
	ftoa(iirVal2, strGain, 10); ftoa(signal.frq, strFrq, 10);								\
	putsUart0(strFrq); putsUart0("\t"); putsUart0(strGain); putsUart0("\r\n");				\
}																							\
else																						\
{																							\
	signal.frq = frq2; signal.dPhi = bit32/100000.0*signal.frq;								\
	iirAnalog(&iirVal1,1); iirAnalog(&iirVal2,2);											\
	iirVal2 = iirVal2/iirVal1;																\
	ftoa(iirVal2, strGain, 10); ftoa(signal.frq, strFrq, 10);								\
	putsUart0(strFrq); putsUart0("\t"); putsUart0(strGain); putsUart0("\r\n");				\
}

#define RED_LED_MASK bit1
#define RED_LED peripheralBitBand(GPIO_PORTF_DATA_R,1)
#define BLUE_LED_MASK bit2
#define BLUE_LED peripheralBitBand(GPIO_PORTF_DATA_R,2)
#define GREEN_LED_MASK bit3
#define GREEN_LED peripheralBitBand(GPIO_PORTF_DATA_R,3)

uint16_t signalData1[4096], signalData2[4096], signalData3[4096];
struct signalInfo signalA = {DirectCurrent,0,0,0,0,0,0,signalData1,0,0,0};
struct signalInfo signalB = {DirectCurrent,0,0,0,0,0,0,signalData2,0,0,0};
uint16_t* dataBuffer = signalData3;

void initChannel(uint16_t* data, uint8_t channel)
{
    for (uint16_t i = 0; i<4096; ++i)
		data[i] = convVout2Dac(0, channel);
}

void timer0IntHandler()
{
	static uint16_t vOutA, vOutB;
    DAC_LDAC = 0; DAC_LDAC = 1;
	putDACOutA(vOutA);
	putDACOutB(vOutB);

	if (signalA.state)
		{
			signalA.state = signalA.cycleCount ? (bool)(--signalA.cycleCount) : signalA.state;
			vOutA = signalA.data[(signalA.phi += signalA.dPhi) >> 20]; 
		}
	else
		vOutA = convVoutA(0);

	if (signalB.state)
		{
			signalB.state = signalB.cycleCount ? (bool)(--signalB.cycleCount) : signalB.state;
			vOutB = signalB.data[(signalB.phi += signalB.dPhi) >> 20]; 
		}
	else
		vOutB = convVoutB(0);

	setRegisterBit(TIMER0_ICR_R, TIMER_ICR_TATOCINT);
}

#define flashLED()				\
{								\
	GREEN_LED = 1;				\
	waitMicroSeconds(500000);	\
	GREEN_LED = 0;				\
	waitMicroSeconds(500000);	\
}

void initHw(void)
{
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	setRegisterBit(SYSCTL_RCGC2_R, SYSCTL_RCGC2_GPIOF);										// SYSCTL_RCGC2_GPIOF -> Enable GPIO port F peripherals

	setRegisterBit(GPIO_PORTF_DIR_R, GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK);		// Sets PF3(Green LED) to be an output
	setRegisterBit(GPIO_PORTF_DEN_R, GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK);		// Sets PF3(Green LED) to be a digital I/O

	flashLED();											// Flash the green LED

	// Initialize UART0 and display greeting
	initUART0();
	putsUart0("\r\n\r\n"); waitMicroSeconds(100000);
	putsUart0("======================================================>\r\n"); waitMicroSeconds(100000);
	putsUart0("EE 5314 Embedded Microcontroller System Design\r\n\r\n"); waitMicroSeconds(100000);
	putsUart0("Low-Cost Programmable Pulse Generator with\r\n"); waitMicroSeconds(100000);
	putsUart0("Automatic Level Control and Network Gain Calculation\r\n\r\n"); waitMicroSeconds(100000);
	putsUart0("Author: Vatsal Asitkumar Joshi\r\n"); waitMicroSeconds(100000);
	putsUart0("======================================================>\r\n"); waitMicroSeconds(100000);

	initAIN0();											// Initialize AIN0
	initAIN1();											// Initialize AIN1
	initDAC();											// Initialize SSI2 module
	initTimer0();										// Initialize the timer
}

int main(void)
{
	initHw();											// Initialize hardware
	initChannel(signalA.data, 1);
	initChannel(signalB.data, 2);

	struct termInput uartInput;

	while(1)
	{
		putsUart0("\r\n>");
		getsUart0(&uartInput);
		char* command = getArgString(&uartInput,0);
		if (!strcmp(command,"reset") && uartInput.argCount)
		{
			setRegisterVal(NVIC_APINT_R,NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ);
			while(1);
		}

		else if (!strcmp(command,"redled") && 1<=uartInput.argCount)
		{
			RED_LED = atoi(getArgString(&uartInput,1));
		}

		else if (!strcmp(command,"blueled") && 1<=uartInput.argCount)
		{
			BLUE_LED = atoi(getArgString(&uartInput,1));
		}

		else if (!strcmp(command,"greenled") && 1<=uartInput.argCount)
		{
			GREEN_LED = atoi(getArgString(&uartInput,1));
		}

		else if (!strcmp(command,"run"))
		{
			signalA.phi = 0; signalB.phi = 0;
			signalA.cycleCount = signalA.nCycles; signalB.cycleCount = signalB.nCycles;
			signalA.state = true; signalB.state = true;
		}

		else if (!strcmp(command,"stop"))
		{
			signalA.state = false; signalB.state = false;
		}

		else if (!strcmp(command,"dc") && 2<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				signalA.type = DirectCurrent; generateSignal(signalA, dataBuffer, 1, &uartInput);
				if (signalA.bias < -4 || signalA.bias > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				signalB.type = DirectCurrent; generateSignal(signalB, dataBuffer, 2, &uartInput);
				if (signalB.bias < -4 || signalB.bias > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else
			{
				putsUart0("Error: 'dc' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!strcmp(command,"cycles") && 2<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
				if (!strcmp(getArgString(&uartInput,2),"continuous"))
				{
					signalA.nCycles = 0; signalA.cycleCount = signalA.nCycles;
				}
				else if (signalA.frq)
				{
					signalA.nCycles = 100000/signalA.frq*atoi(getArgString(&uartInput,2)); signalA.cycleCount = signalA.nCycles;
				}
				else
				{
					putsUart0("Error: Number of cycles cannot bec counted if the frequency is 0.\r\n");
				}
			else if (atoi(getArgString(&uartInput,1)) == 2)
				if (!strcmp(getArgString(&uartInput,2),"continuous"))
				{
					signalB.nCycles = 0; signalB.cycleCount = signalB.nCycles;
				}
				else if (signalB.frq)
				{
					signalB.nCycles = 100000/signalB.frq*atoi(getArgString(&uartInput,2)); signalB.cycleCount = signalB.nCycles;
				}
				else
				{
					putsUart0("Error: Number of cycles cannot be counted if the signal frequency is 0.\r\n");
				}
			else
			{
				putsUart0("Error: 'cycles' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!strcmp(command,"sine") && 2<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				signalA.type = Sinusoidal; generateSignal(signalA, dataBuffer, 1, &uartInput);
				if ((signalA.bias - signalA.amp) < -4 || (signalA.bias + signalA.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				signalB.type = Sinusoidal; generateSignal(signalB, dataBuffer, 2, &uartInput);
				if ((signalB.bias - signalB.amp) < -4 || (signalB.bias + signalB.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else
			{
				putsUart0("Error: 'sine' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!strcmp(command,"square") && 2<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				if ((uartInput.argCount > 5) && (atof(getArgString(&uartInput,5)) <= 0 || atof(getArgString(&uartInput,5)) >= 100))
				{
					putsUart0("Error: Duty cycle has to be between 0 and 100. Else, 'dc' command can be used.\r\n");
					continue;
				}
				signalA.type = Square; generateSignal(signalA, dataBuffer, 1, &uartInput);
				if ((signalA.bias) < -4 || (signalA.bias + signalA.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				if ((uartInput.argCount > 5) && (atof(getArgString(&uartInput,5)) <= 0 || atof(getArgString(&uartInput,5)) >= 100))
				{
					putsUart0("Error: Duty cycle has to be between 0 and 100. Else, 'dc' command can be used.\r\n");
					continue;
				}
				signalB.type = Square; generateSignal(signalB, dataBuffer, 2, &uartInput);
				if ((signalB.bias) < -4 || (signalB.bias + signalB.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else
			{
				putsUart0("Error: 'square' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!strcmp(command,"sawtooth") && 2<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				signalA.type = Sawtooth; generateSignal(signalA, dataBuffer, 1, &uartInput);
				if ((signalA.bias) < -4 || (signalA.bias + signalA.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				signalB.type = Sawtooth; generateSignal(signalB, dataBuffer, 2, &uartInput);
				if ((signalB.bias) < -4 || (signalB.bias + signalB.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
		}

		else if (!strcmp(command,"triangle") && 2<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				signalA.type = Triangle; generateSignal(signalA, dataBuffer, 1, &uartInput);
				if ((signalA.bias) < -4 || (signalA.bias + signalA.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				signalB.type = Triangle; generateSignal(signalB, dataBuffer, 2, &uartInput);
				if ((signalB.bias) < -4 || (signalB.bias + signalB.amp) > 4)
					putsUart0("Warning: This device can only generate -4V to 4V accurately.\r\n");
			}

			else
			{
				putsUart0("Error: 'triangle' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!strcmp(command,"voltage") && 1<=uartInput.argCount)
		{
			char strVoltage[10];
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				ftoa(readAIN0(), strVoltage, 10);
				putsUart0("\tCH1: "); putsUart0(strVoltage); putsUart0("\r\n");
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				ftoa(readAIN1(), strVoltage, 10);
				putsUart0("\tCH2: "); putsUart0(strVoltage); putsUart0("\r\n");
			}
			else
			{
				putsUart0("Error: 'voltage' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!strcmp(command,"gain") && 3<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				float frq1 = atof(getArgString(&uartInput,2)), frq2 = atof(getArgString(&uartInput,3));
				signalA.state = false;
				signalA.type = Sinusoidal; signalA.bias = 0; signalA.amp = 4;
				signalA.frq = frq1; signalA.dPhi = bit32/100000.0*signalA.frq;
				for (uint16_t i = 0; i<4096; ++i)
					signalA.data[i] = convVout2Dac(signalA.bias + signalA.amp*sin(2*M_PI/4096*i), 1);
				signalA.state = true;
				gain(signalA, frq1, frq2);
				signalA.state = false;
				initChannel(signalA.data,1);
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				float frq1 = atof(getArgString(&uartInput,2)), frq2 = atof(getArgString(&uartInput,3));
				signalB.state = false;
				signalB.type = Sinusoidal; signalB.bias = 0; signalB.amp = 4;
				signalB.frq = frq1; signalB.dPhi = bit32/100000.0*signalB.frq;
				for (uint16_t i = 0; i<4096; ++i)
					signalB.data[i] = convVout2Dac(signalB.bias + signalB.amp*sin(2*M_PI/4096*i), 2);
				signalB.state = true;
				gain(signalB, frq1, frq2);
				signalB.state = false;
				initChannel(signalB.data,2);
			}
			else
			{
				putsUart0("Error: 'gain' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!strcmp(command, "differential") && 2<=uartInput.argCount)
		{
			if (atoi(getArgString(&uartInput,1)) == 1)
			{
				if (!strcmp(getArgString(&uartInput,2),"on") && signalA.bias == 0)
				{
					signalB.type = Differential; signalB.state = 0; signalB.cycleCount = 0;
					signalB.amp = signalA.amp; signalB.bias = signalA.bias; signalB.dPhi = signalA.dPhi;
					for (uint16_t i = 0; i<4096; ++i)
						signalB.data[i] = convVout2Dac(-convDac2Vout(signalA.data[i],1),2);
					signalB.state = signalA.state; signalB.phi = signalA.phi;
				}
				else if (!strcmp(getArgString(&uartInput,2),"on") && signalA.bias != 0)
					putsUart0("Error: The offset of this signal needs to be zero.");
				else if (!strcmp(getArgString(&uartInput,2),"off") && signalB.type == Differential)
				{
					signalB.type = DirectCurrent; signalB.state = 0; signalB.cycleCount = 0;
					signalB.amp = 0; signalB.bias = 0; signalB.dPhi = 0;
					for (uint16_t i = 0; i<4096; ++i)
						signalB.data[i] = convVout2Dac(0,2);
					signalB.state = 0; signalB.phi = 0;
				}
				else
					putsUart0("Error: Differential not activated.");
			}
			else if (atoi(getArgString(&uartInput,1)) == 2)
			{
				if (!strcmp(getArgString(&uartInput,2),"on") && signalB.bias == 0)
				{
					signalA.type = Differential; signalA.state = 0; signalA.cycleCount = 0;
					signalA.amp = signalB.amp; signalA.bias = signalB.bias; signalA.dPhi = signalB.dPhi;
					for (uint16_t i = 0; i<4096; ++i)
						signalA.data[i] = convVout2Dac(-convDac2Vout(signalB.data[i],1),2);
					signalA.state = signalB.state; signalA.phi = signalB.phi;
				}
				else if (!strcmp(getArgString(&uartInput,2),"on") && signalB.bias != 0)
					putsUart0("Error: The offset of this signal needs to be zero.");
				else if (!strcmp(getArgString(&uartInput,2),"off") && signalB.type == Differential)
				{
					signalA.type = DirectCurrent; signalA.state = 0; signalA.cycleCount = 0;
					signalA.amp = 0; signalA.bias = 0; signalA.dPhi = 0;
					for (uint16_t i = 0; i<4096; ++i)
						signalA.data[i] = convVout2Dac(0,2);
					signalA.state = 0; signalA.phi = 0;
				}
				else
					putsUart0("Error: Differential not activated.");
			}
			else
			{
				putsUart0("Error: 'differential' command supports either 1 or 2 as channel number.\r\n");
			}
		}

		else if (!uartInput.argCount)
		{}

		else
		{
			putsUart0("\t"); putsUart0("Invalid Command\r\n");
		}
	}
}

// Functions for UART0
void parseString(struct termInput* userInput)
{
	bool prevCharState = 0;								// Previous Character state. 0 = Character not useful, 1 = Character is useful
	userInput->argCount = 0;
	uint8_t length = strlen(userInput->str);
	for (uint8_t i = 0; i<length; ++i)
	{
		char c = userInput->str[i];
		if ((c==45 || c==46) || (c>47 && c<58) || (c>64 && c<91) || (c>96 && c<123))
		{
			if (prevCharState==0)
			{
				if (userInput->argCount == MAX_FIELDS)
				{
					putsUart0("\r\n> Number of arguments entered is more than maximum limit.");
					putsUart0("\r\n> All the arguments after maximum limit will be ignored.");
					break;
				}
				userInput->pos[userInput->argCount++] = i;
				prevCharState = 1;
			}
			if (c>64 && c<91)
			{
				userInput->str[i] += 32;
			}
		}
		else
		{
			prevCharState = 0;
			userInput->str[i] = '\0';
		}
	}
}

char* getArgString(struct termInput* userInput, uint8_t argNumber)
{
	if (argNumber <= userInput->argCount)
		return userInput->str+userInput->pos[argNumber];
	else
		return NULL;
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

void putsUart0(char* str)	// Blocking function that writes a string when the UART buffer is not full
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

char getcUart0()	// Blocking function that returns with serial data once the buffer is not empty
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getsUart0(struct termInput* userInput)		// Blocking function that returns with input data written in terminal
{
	uint8_t count = 0;
	char c;

	while (count<MAX_CHARS)
	{
		c = getcUart0();
		if (c==8 || c==127)
		{
			if (count==0)
				continue;
			count--;
		}
		else if (c==13 || c==10)
		{
			userInput->str[count] = '\0';
			break;
		}
		else if (c >= 32)
		{
			userInput->str[count++] = c;
		}
		else
		{
			continue;
		}
	}
	parseString(userInput);
	putsUart0("\r\n");
	// for (uint8_t i = 0; i<userInput->argCount; ++i)
	// {
	// 	putsUart0("\r\n"); putsUart0("\t"); putsUart0(getArgString(userInput,i));
	// }
	// putsUart0("\r\n> ");
}

// Functions to read voltages on AIN0 and AIN1
float readAIN0()
{
	setRegisterBit(ADC0_PSSI_R, ADC_PSSI_SS3);													// set start bit
	while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);														// wait until SS3 is not busy
	return ((ADC0_SSFIFO3_R+0.5)*3.3/4096)*ADC0_CALI;											// get single result from the FIFO
}

float readAIN1()
{
	setRegisterBit(ADC1_PSSI_R, ADC_PSSI_SS3);													// set start bit
	while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);														// wait until SS3 is not busy
	return ((ADC1_SSFIFO3_R+0.5)*3.3/4096)*ADC1_CALI;											// get single result from the FIFO
}

void iirAnalog(float* vFiltered, uint8_t channel)
{
	float alf = 0.99;
	for (uint16_t i = 0; i<1000; ++i)
		*vFiltered = alf*(*vFiltered) + (1-alf)*(channel==1 ? readAIN0() : readAIN1() );
}

void ftoa(float fVal, char* str, int strLength)
{
	uint8_t exp = 0;
	for (exp = 0; exp<100; ++exp)
	{
		if (fVal < 10)
			break;
		else
			fVal /= 10;
	}
	int iVal = fVal;
	fVal = (fVal - iVal)*10;
	str[0] = '0' + iVal;
	str[1] = '.';
	for (uint8_t i = 2; i<strLength-4; i++)
	{
		iVal = fVal;
		str[i] = '0' + iVal;
		fVal = (fVal - iVal)*10;
	}
	str[strLength-4] = 'e';
	str[strLength-3] = '0' + exp/10;
	str[strLength-2] = '0' + exp - (exp/10)*10;
	str[strLength-1] = '\0';
}

