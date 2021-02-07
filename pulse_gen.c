//Project Steps
//Shreyas Gawali
//UTA ID: 1001765578
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// List of valid commands
//-----------------------------------------------------------------------------

// reset
// adcread<>channel number
// dc<>channel number<>voltage
// sine<>channel number<>frequency<>amplitude<>offset
// square<>channel number<>frequency<>amplitude<>offset<>(optional: duty cycle)
// triangle<>channel number<>frequency<>amplitude<>offset
// sawtooth<>channel number<>frequency<>amplitude<>offset
// run
// stop
// cycles<>channel number<>number of cycles
// hilbert
// hilbert off
// differential
// differential off

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

#define GREEN_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define PA0             (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 0*4)))
#define PA1             (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 1*4)))
#define LDAC            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define FSS             (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define TX              (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define CLK             (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define GREEN_LED_MASK 8
#define BLUE_LED_MASK 4
#define RED_LED_MASK 2
#define AIN0_MASK 8
#define AIN1_MASK 4
#define MAX_CHARS 80
#define TX_MASK 8
#define FSS_MASK 2
#define CLK_MASK 1
#define LDAC_MASK 64
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

char strg[MAX_CHARS + 1],out[80][80],buffer[50];
uint8_t pos[40],argCount,argString,argNum,n,trg,dfl;
uint16_t LUT[2][4096],chl,count,dc;
uint32_t phi,delP,data,dataOr;
float v1,v2,rawVtg1,rawVtg2,amp,freq;
double volt,x,offset;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1; //enable clock for SSI1 module
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;

    // Enable GPIO ports A and F peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOD;
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;

    // Configure LED pin
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;  // enable LED

    // Configure LDAC
    GPIO_PORTA_DIR_R |= LDAC_MASK;       // make bit an output
    GPIO_PORTA_DR2R_R |= LDAC_MASK;      // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= LDAC_MASK;       // enable LDAC

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2; // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3; // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;       // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX; // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R |= UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R |= UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R |= UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure AIN0 and AIN1 as an analog inputs
    GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AIN0 (PE3)
    GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3
    GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AIN1 (PE2)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

    // Configure ADC0
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R |= ADC_EMUX_EM3_PROCESSOR;           // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R |= 0;                              // set first sample to AIN0
    ADC0_SSCTL3_R |= ADC_SSCTL3_END0;                // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure ADC1
    ADC1_CC_R |= ADC_CC_CS_SYSPLL;                   // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R |= ADC_EMUX_EM3_PROCESSOR;           // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R |= 1;                              // set first sample to AIN1
    ADC1_SSCTL3_R |= ADC_SSCTL3_END0;                // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;                            // set load value to 400 for 100 kHz interrupt rate

    // Configure SSI1 pins for SPI configuration
    GPIO_PORTD_DIR_R |= TX_MASK | FSS_MASK | CLK_MASK; // make SSI1 TX, FSS, and CLK outputs
    GPIO_PORTD_DR2R_R |= TX_MASK | FSS_MASK | CLK_MASK; // set drive strength to 2mA
    GPIO_PORTD_AFSEL_R |= TX_MASK | FSS_MASK | CLK_MASK; // select alternative functions
    GPIO_PORTD_PCTL_R = GPIO_PCTL_PD3_SSI1TX | GPIO_PCTL_PD1_SSI1FSS | GPIO_PCTL_PD0_SSI1CLK; // map alternate functions to SSI1
    GPIO_PORTD_DEN_R |= TX_MASK | FSS_MASK | CLK_MASK; // enable digital operation
    GPIO_PORTD_PUR_R |= CLK_MASK;                      // SCLK must be enabled when SPO=1 (see 15.4)

    // Configure the SSI1 as a SPI master, mode 3, 12 bit operation, 1 MHz bit rate
    SSI1_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI1 to allow re-configuration
    SSI1_CR1_R = 0;                                    // select master mode
    SSI1_CC_R = 0;                                     // select system clock as the clock source
    SSI1_CPSR_R = 10;                                  // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI1_CR0_R |= SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16;   // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
    SSI1_CR1_R |= SSI_CR1_SSE;                         // turn on SSI1
    LDAC = 0x01;
    //FSS = 1;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

void timer1Isr()
{
    if(trg==1)
    {
        while(count!=0)
        {
            uint16_t ch1,ch2;
            phi = phi + delP;
            if(chl==1)
            {
                LDAC = 0x01;
                ch1 = LUT[0][phi >> 20];
                SSI1_DR_R = ch1+12288;
                LDAC = 0x00;
            }
            else if(chl==2)
            {
                LDAC = 0x01;
                ch2 = LUT[1][phi >> 20];
                SSI1_DR_R = ch2+45056;
                LDAC = 0x00;
            }
            count--;
        }
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;
    }
    uint16_t ch1,ch2;
    phi = phi + delP;
    LDAC = 0x01;
    ch1 = LUT[0][phi >> 20];
    SSI1_DR_R = ch1+12288;

    ch2 = LUT[1][phi >> 20];
    SSI1_DR_R = ch2+45056;
    LDAC = 0x00;

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;             // clear interrupt flag
}

//STEP_1
void ledBlink()
{
    GREEN_LED = 1;
    waitMicrosecond(500000);
    GREEN_LED = 0;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
    putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

//STEP_2
void getString()
{
uint8_t count=0;
char c;
while(1)
{
    label: c = getcUart0();
    putcUart0(c);
    if((c==8)||(c==127))
    {
        if(count>0)
        {
            count--;
            goto label;
        }
        else
        {
            goto label;
        }
    }
    else
    {
        if((c==10)||(c==13))
        {
            label1: strg[count] = 0;
            break;
        }
        else
        {
            if(c>=32)
            {
                strg[count++] = c;
                if(count == MAX_CHARS)
                {
                    goto label1;
                }
                else
                {
                    goto label;
                }
            }
            else
            {
                goto label;
            }
        }
    }
}
}

//STEP_3
bool alfNum(char a)
{
    if((a>=45 && a<=57)||(a>=65 && a<=90)||(a>=97 && a<=122))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool splCh(char b)
{
    if((b>=32 && b<=44)||(b==47)||(b>=58 && b<=64)||(b>=91 && b<=96)||(b>=123 && b<=126))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void parseString()
{
    uint8_t j=0,l,i=0,r=0,c=0,flag=0;
    l = strlen(strg);
    while(i<=l)
    {
        while(alfNum(strg[i]))
        {
            flag =1;
            if((i==0)||(strg[i-1]=='\0'))
            {
                pos[j]=i;
                j++;
            }
            out[j-1][c] = strg[i];
            c++;
            i++;
        }
        c=0;
        if(flag==1)
        {
            r++;
        }
        if(splCh(strg[i+1]) && flag==1)
        {
            r--;
        }
        strg[i]='\0';

        i++;
    }
argCount=r;
}

//STEP_4
char *getargString(uint8_t argNum)
{
    if(argNum == argCount-1)
    {
        return &strg[pos[argNum]];
    }
    else
    {
        putsUart0("Error: Invalid number of arguments");
    }
}

uint32_t getargInt(uint8_t argNum)
{
    return atoi((getargString(argNum)));
}

float getargFloat(uint8_t argNum)
{
    return atof((getargString(argNum)));
}

bool isCommand(char *command, uint8_t argNum)
{
    uint16_t compareStr;
    compareStr = strcmp(&strg[pos[0]], command);
    if(argNum == argCount-1 && compareStr == 0)
    {
        //putsUart0("Valid command");
        //putsUart0("\r\n");
        return true;
    }
    else
    {
        //putsUart0("Invalid command");
        //putsUart0("\r\n");
        return false;
    }
}

//STEP_5
void reset()
{
    putsUart0("System reset initiated");
    putsUart0("\r\n");
    waitMicrosecond(1000000);
//  NVIC_APINT_R = 0xFA05|0x0004;
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}

//STEP_6
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}

void adcRead()
{
    chl= atoi(&strg[pos[1]]);
    if(chl==1)
    {
        v1 = readAdc0Ss3();
        rawVtg1 = (v1*3.3/4096)+0.01;
        putsUart0("\r\n");
        sprintf(buffer, "Voltage at AIN0: %f", rawVtg1);
        putsUart0(buffer);
        putsUart0("\r\n");
    }
    else if(chl==2)
    {
        v2 = readAdc1Ss3();
        rawVtg2 = (v2*3.3/4096)+0.01;
        putsUart0("\r\n");
        sprintf(buffer, "Voltage at AIN1: %f", rawVtg2);
        putsUart0(buffer);
        putsUart0("\r\n");
    }
    else
    {
        putsUart0("Invalid channel number");
    }
}

//STEP_7
void dcCommand()
{
    chl = atoi(&strg[pos[1]]);
    volt = atof(&strg[pos[2]]);
    if (chl == 1)
    {
        x = -0.1989*volt + 0.9981;         //Transfer function for VOUT1
        data = (x*4096)/2.048;
        dataOr = 12288+data;
        FSS = 0x00;                        // CS_bar low assert chip select
        LDAC = 0x01;
        __asm (" NOP");                    // allow line to settle
        __asm (" NOP");
        __asm (" NOP");
        __asm (" NOP");
        SSI1_DR_R = dataOr;                // write data
        while (SSI1_SR_R & SSI_SR_BSY);    // wait for transmission to stop
        //FSS = 0x01;                      // CS_bar high: de-assert chip select
        LDAC = 0x00;
    }
    else if (chl == 2)
    {
        x = -0.1985*volt + 1.0052;         //Transfer function for VOUT2
        data = (x*4096)/2.048;
        dataOr = 45056+data;
        FSS = 0x00;                        // CS_bar low assert chip select
        LDAC = 0x01;
        __asm (" NOP");                    // allow line to settle
        __asm (" NOP");
        __asm (" NOP");
        __asm (" NOP");
        SSI1_DR_R = dataOr;                // write data
        while (SSI1_SR_R & SSI_SR_BSY);    // wait for transmission to stop
        //FSS = 0x01;                      // CS_bar high: de-assert chip select
        LDAC = 0x00;
     }
    else
    {
        putsUart0("Invalid channel number");
    }
}

//STEP_8
void sineWave()
{
    dfl=1;
    uint16_t i;
    chl = atoi(&strg[pos[1]]);
    freq = atof(&strg[pos[2]]);
    amp = atof(&strg[pos[3]]);
    offset = atof(&strg[pos[4]]);
    delP = freq*pow(2,32)/100000;
    if(chl==1)
    {
        for(i=0;i<4096;i++)
        {
            LUT[0][i] = (-397.8*((sin((2*3.142*i)/4096))*amp + offset))+1996.2;
        }
     }
     else if(chl==2)
     {
         for(i=0;i<4096;i++)
         {
             LUT[1][i] = (-397*((sin((2*3.142*i)/4096))*amp + offset))+2010.4;
         }
     }
     else
     {
         putsUart0("Invalid channel number");
     }
}

//STEP_9
void stopCommand()
{
    TIMER1_IMR_R = 0x0000;                              // turn-off interrupts
    NVIC_EN0_R &= ~(1 << (INT_TIMER1A-16));             // turn-off interrupt 37 (TIMER1A)
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                    // turn-off timer
    uint16_t data1,data2;
    data1 = 1996+12288;
    data2 = 2010+45056;

    LDAC = 0x01;
    SSI1_DR_R = data1;
    while (SSI1_SR_R & SSI_SR_BSY);
    LDAC = 0x00;

    LDAC = 0x01;
    SSI1_DR_R = data2;
    while (SSI1_SR_R & SSI_SR_BSY);
    LDAC = 0x00;
}

void runCommand()
{
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

//STEP_10
void cyclesChannel()
{
    trg=1;
    chl = atoi(&strg[pos[1]]);
    n = atoi(&strg[pos[2]]);
    count = (100000/freq)*n;
}

//STEP_11
void squareWave()
{
    dfl=2;
    uint16_t i;
    chl = atoi(&strg[pos[1]]);
    freq = atof(&strg[pos[2]]);
    amp = atof(&strg[pos[3]]);
    offset = atof(&strg[pos[4]]);
    delP = freq*pow(2,32)/100000;
    if(chl==1)
    {
        for(i=0;i<4096;i++)
        {
            if(i<2048)
            {
                LUT[0][i] = (-397.8*(amp + offset)) + 1996.2;
            }
            if(i>2048)
            {
                LUT[0][i] = (-397.8*((-1)*amp + offset)) + 1996.2;
            }
        }
    }
    else if(chl==2)
    {
        for(i=0;i<4096;i++)
        {
            if(i<2048)
            {
                LUT[1][i] = (-397*(amp + offset)) + 2010.4;
            }
            if(i>2048)
            {
                LUT[1][i] = (-397*((-1)*amp + offset)) + 2010.4;
            }
        }
    }
    else
    {
        putsUart0("Invalid channel number");
    }
}

void squareWavedc()
{
    dfl=4;
    uint16_t i;
    chl = atoi(&strg[pos[1]]);
    freq = atof(&strg[pos[2]]);
    amp = atof(&strg[pos[3]]);
    offset = atof(&strg[pos[4]]);
    dc = atoi(&strg[pos[5]]);
    dc = (dc*2048)/50;
    delP = freq*pow(2,32)/100000;
    if(chl==1)
    {
        for(i=0;i<4096;i++)
        {
            if(i<dc)
            {
                LUT[0][i] = (-397.8*(amp + offset)) + 1996.2;
            }
            if(i>dc)
            {
                LUT[0][i] = (-397.8*((-1)*amp + offset)) + 1996.2;
            }
        }
    }
    else if(chl==2)
    {
        for(i=0;i<4096;i++)
        {
            if(i<dc)
            {
                LUT[1][i] = (-397*(amp + offset)) + 2010.4;
            }
            if(i>dc)
            {
                LUT[1][i] = (-397*((-1)*amp + offset)) + 2010.4;
            }
        }
    }
    else
    {
        putsUart0("Invalid channel number");
    }
}

//STEP_12
void triangleWave()
{
    dfl=3;
    uint16_t i;
    chl = atoi(&strg[pos[1]]);
    freq = atof(&strg[pos[2]]);
    amp = atof(&strg[pos[3]]);
    offset = atof(&strg[pos[4]]);
    delP = freq*pow(2,32)/100000;
    if(chl==1)
    {
        for(i=0;i<4096;i++)
        {
            if(i<2048)
            {
                LUT[0][i] = (-397.8*((asin(sin(2*3.142*i/4096)))*(2*amp/3.142) + offset)) + 1996.2;
            }
            if(i>2048)
            {
                LUT[0][i] = (-397.8*((asin(sin(2*3.142*i/4096)))*(2*(-1)*amp/3.142) + offset)) + 1996.2;
            }
        }
    }
    else if(chl==2)
    {
        for(i=0;i<4096;i++)
        {
            if(i<2048)
            {
                LUT[1][i] = (-397*((asin(sin(2*3.142*i/4096)))*(2*amp/3.142) + offset)) + 2010.4;
            }
            if(i>2048)
            {
                LUT[1][i] = (-397*((asin(sin(2*3.142*i/4096)))*(2*(-1)*amp/3.142) + offset)) + 2010.4;
            }
        }
    }
    else
    {
        putsUart0("Invalid channel number");
    }
}

//STEP_13
void sawtoothWave()
{
    dfl=5;
    uint16_t i;
    chl = atoi(&strg[pos[1]]);
    freq = atof(&strg[pos[2]]);
    amp = atof(&strg[pos[3]]);
    offset = atof(&strg[pos[4]]);
    delP = freq*pow(2,32)/100000;
    if(chl==1)
    {
        for(i=0;i<4096;i++)
        {
            LUT[0][i] = (-397.8*((amp*i/4096) + offset)) + 1996.2;
        }
    }
    else if(chl==2)
    {
        for(i=0;i<4096;i++)
        {
            LUT[1][i] = (-397*((amp*i/4096) + offset)) + 2010.4;
        }
    }
    else
    {
        putsUart0("Invalid channel number");
    }
}

//STEP_14
void hilbert()
{
    uint16_t i;
    for(i=0;i<4096;i++)
    {
        LUT[1][i] = (-397*((cos((2*3.142*i)/4096))*amp + offset))+2010.4;
    }
}

void hilbertOff()
{
    uint16_t i;
    for(i=0;i<4096;i++)
    {
        LUT[1][i] = 2010;
    }
}

//STEP_15
void differential()
{
    uint16_t i;
    if(dfl==1)
    {
        for(i=0;i<4096;i++)
        {
        LUT[1][i] = (-397.8*((sin((2*3.142*i)/4096))*(-amp) + offset))+1996.2;
        }
    }
    else if(dfl==2)
    {
        for(i=0;i<4096;i++)
        {
            if(i<2048)
            {
                LUT[1][i] = (-397*((-amp) + offset)) + 2010.4;
            }
            if(i>2048)
            {
                LUT[1][i] = (-397*((-1)*(-amp) + offset)) + 2010.4;
            }
        }
    }
    else if(dfl==3)
    {
        for(i=0;i<4096;i++)
        {
            if(i<2048)
            {
                LUT[1][i] = (-397*((asin(sin(2*3.142*i/4096)))*(2*(-amp)/3.142) + offset)) + 2010.4;
            }
            if(i>2048)
            {
                LUT[1][i] = (-397*((asin(sin(2*3.142*i/4096)))*(2*(-1)*(-amp)/3.142) + offset)) + 2010.4;
            }
        }
    }
    else if(dfl==4)
    {
        for(i=0;i<4096;i++)
        {
            if(i<dc)
            {
                LUT[1][i] = (-397*((-amp) + offset)) + 2010.4;
            }
            if(i>dc)
            {
                LUT[1][i] = (-397*((-1)*(-amp) + offset)) + 2010.4;
            }
        }
    }
    else if(dfl==5)
    {
        for(i=0;i<4096;i++)
        {
            LUT[1][i] = (-397*(((-amp)*i/4096) + offset)) + 2010.4;
        }
    }
}

void differentialOff()
{
    uint16_t i;
    for(i=0;i<4096;i++)
    {
        LUT[1][i] = 2010;
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void main()
{
    initHw();                                   // Initialize hardware
    ledBlink();                                 // Toggle GREEN_LED every half second
    while(1)
    {
        putsUart0("\r\n");
        putsUart0("Command input: ");
        putsUart0("\r\n");
        getString();
        putsUart0(strg);
        putsUart0("\r\n");
        parseString();
        putsUart0("\r\n");
        if(isCommand("reset", 0))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            reset();
        }
        else if(isCommand("adcread", 1))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            adcRead();
        }
        else if(isCommand("dc", 2))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            dcCommand();
        }
        else if(isCommand("sine",4))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            sineWave();
        }
        else if(isCommand("square",4))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            squareWave();
        }
        else if(isCommand("square",5))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            squareWavedc();
        }
        else if(isCommand("triangle",4))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            triangleWave();
        }
        else if(isCommand("sawtooth",4))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            sawtoothWave();
        }
        else if(isCommand("run",0))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            runCommand();
        }
        else if(isCommand("stop",0))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            stopCommand();
        }
        else if(isCommand("cycles",2))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            cyclesChannel();
        }
        else if(isCommand("hilbert",0))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            hilbert();
        }
        else if(isCommand("hilbert",1))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            hilbertOff();
        }
        else if(isCommand("differential",0))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            differential();
        }
        else if(isCommand("differential",1))
        {
            putsUart0("Valid command");
            putsUart0("\r\n");
            differentialOff();
        }
        else
        {
            putsUart0("Invalid command");
        }
    }
    //return 0;
}
