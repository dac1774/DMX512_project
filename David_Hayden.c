// David Chavez ID: 1********4
// Hayden Williams ID: 1********0

// Modified from:
// Stop Go C Example (Bitbanding)
// Jason Losh
//      and
// Timing C Example
// Jason Losh

//-----------------------------------------------------------------------------
// Steps Implemented
//-----------------------------------------------------------------------------
// Step 1: Completed 3/6/19
// Step 2: Started 4/3/19  (YIKES!!) Completed 4/3/19
// Step 3: Started 4/3/19 Completed 4/4/19

// note I had to migrate past functions to now use globals

// Step 4: Started 4/4/19 Completed 4/4/19
// Step 5: Started 4/5/19 Completed 4/5/19
// Step 6: Started 4/7/19 Completed 4/8/19
// Note: step 6 has a hack workaround, where the DMX stream is haulted, then
// resumed: possible reasons is that the break or mark after break is incorrect

// Found the fix, the pin needs to idle high for 50-60ms after each frame
// with testing, found that pin needs to be high for around 3500 cycles to be
// stable implemented using the timer1 interrupts (in uart1 ISR)
//              Hack is fixed

// Step 7: Started 4/26/19

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdbool.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
//#define RED_LED_MASK 2    //placeholder (debug)

#define MAX_CHARS   80
//-----------------------------------------------------------------------------
//  Globals
//-----------------------------------------------------------------------------
uint16_t address = 1;
char StringInput[MAX_CHARS + 1];
char type[4];
uint8_t pos[4];
uint8_t fieldCount = 0;
volatile uint8_t dmxData[512];
bool dmxTx = false; // true = transmit, false = do not transmit
bool deviceMode = false; // true = controller, false = receiver
uint16_t dmxMax = 512;
uint16_t phase = 0;
char result[50];
uint8_t trash = 0;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// from Timing.c //J.Losh
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
// D.Chavez
/* This function when called, sets up UART1 for either transmit mode or receive mode,
 * depending on deviceMode global.
 */
void setUpUart1()
{
    GPIO_PORTC_DIR_R |= 0x40 | 0x80; // pin 6 & 7(debug) port C is an output
    GPIO_PORTC_DEN_R |= 0x40 | 0x80;

    if(deviceMode){
        //controller mode
        GPIO_PORTC_DATA_R |= 0x40;  // set DE pin to one

        // Configure UART1 to 250k baud
        UART1_CTL_R = 0;
        UART1_CC_R = UART_CC_CS_SYSCLK;
        UART1_IBRD_R = 10;
        UART1_FBRD_R = 0;
        // Configure with 2 stop bits, 8 bit word, and NO FIFO buffer
        UART1_LCRH_R = UART_LCRH_WLEN_8 /*| UART_LCRH_FEN*/ | UART_LCRH_STP2;
        UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
        UART1_IM_R = UART_IM_TXIM;                       // turn-on TX interrupt
        NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22 (UART1)
        TIMER1_CTL_R = 0;                                //Turn off timer
        GREEN_LED = 0;
        UART1_IM_R &= ~UART_IM_RXIM;                     // turn-off RX and Break interrupt
        UART1_IM_R &= ~UART_IM_BEIM;
    }
    else
    {
        //RX mode
        GPIO_PORTC_DATA_R &= ~0x40;  // set DE pin to zero

        UART1_CTL_R = 0;
        UART1_CC_R = UART_CC_CS_SYSCLK;
        UART1_IBRD_R = 10;
        UART1_FBRD_R = 0;
        // Configure with 2 stop bits, 8 bit word, and FIFO buffer
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_STP2;
        UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
        UART1_IM_R = UART_IM_RXIM | UART_IM_BEIM;                    // turn-on RX and Break interrupt
        NVIC_EN0_R |= 1 << (INT_UART1-16);                 //--------// turn-on interrupt 22 (UART1)

        UART1_IM_R &= ~UART_IM_TXIM;                       // turn-off TX interrupt
        TIMER1_CTL_R = 0;           //Turn off timer
        TIMER1_TAILR_R = 0x4C4B400; // 2 second delay load
        TIMER1_CTL_R |= TIMER_CTL_TAEN; //turn on timer
    }
}
//H. Williams
void init_EEPROM()
{
    //Enable EEPROM Clock
    SYSCTL_RCGCEEPROM_R = 1;

    //Wait atleast 6 cycles
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");

    //Wait for EEPROM module to finish
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);

    //Check for errors
    if((EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY) || (EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY))
    {
        putsUart0("Critical error in EEPROM\r\n");
        return;
    }

    //Reset EEPROM module
    SYSCTL_SREEPROM_R = 1;
    SYSCTL_SREEPROM_R = 0;

    //Wait atleast 6 cycles
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");

    //Wait for EEPROM module to finish
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);

    //Check for errors
    if((EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY) || (EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY))
    {
        putsUart0("Critical error in EEPROM\r\n");
        return;
    }

}
// J.Losh, D.Chavez
// Initialize Hardware: modified from Stop GO C Example
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;// Note UART on port A must use APB

    // Enable GPIO port A C F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC;
    //timer clock
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer


    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1, 2 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button


    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
    // Configure EEPROM
    init_EEPROM();

    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
    GPIO_PORTC_DIR_R |= 0x20; //change
    GPIO_PORTC_DEN_R |= 0x30;                         // for pins 4 and 5
    GPIO_PORTC_AFSEL_R |= 0x30;
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_U1RX | GPIO_PCTL_PC5_U1TX;   //use pins 4 and 5 in port C for UART



    // Configure Timer 1
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;                       // set load value to get one second (saftey)
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;                // turn-on timer

    setUpUart1();   //set up uart 1 (has timer stuff)
}

/* Step 1
 * Error helping, by flashing Red LED to detect boot loops
 */
void flashRedLED()
{
    // wait 500 ms
    waitMicrosecond(500000);
    // turn on red LED
    RED_LED = 1;

    // wait 500 ms
    waitMicrosecond(500000);
    // turn off red LED
    RED_LED = 0;

}
// Blink green LED
void blinkGreenLED()
{
    GREEN_LED = 1;
    waitMicrosecond(10000);
    GREEN_LED = 0;
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
//writing to DMX UART
void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = c;
}
//clear serial screen helper function
void clearScreen()
{
    putcUart0(27);  //esc
    putsUart0("[2J");
    putcUart0(27);  //esc
    putsUart0("[H");
}

/* Step 2 - D.Chavez
 * This function will take character input from uart0 (incoming from the terminal)
 * and will return a string with acquired values. It does basic character validation
 * and has backspace support
 * Warnings: calls a Blocking Function
 */
void getsUart0(uint8_t maxChars)
{
    uint8_t count = 0;
    putcUart0('>');
    while(1)
    {
        char c = getcUart0();   //Blocking func
        if(c == 13) //carriage return
        {
            StringInput[count] = 0;
            blinkGreenLED();
            putsUart0("\r\n");
            return;
        }
        if(c == 8) //backspace
        {
            if(count > 0)
            {
                count--;
                putcUart0(8);
                putcUart0(' ');
                putcUart0(8);
            }
        }
        if(c >= ' ') //valid character
        {
            StringInput[count++] = tolower(c);
            putcUart0(tolower(c));
        }
        if(count == maxChars)
        {
            StringInput[count] = 0;
            blinkGreenLED();
            putsUart0("\r\n");
            return;
        }
    }
}
/* Step 3 - D.Chavez
 * lightweight parser, will be used to create pointers to main string to where
 * the useful data is located, will replace all other characters in the string
 * with null
 */
void parceStr()
{
    uint8_t i = 0;
    uint8_t d = 1;  //Delimiter flag
    fieldCount = 0;
    while(true)
    {
        if(fieldCount == 4) // This is to avoid overflowing the size 4 matrixes
        {
            return;
        }
        if(StringInput[i] == 0) //end of command string <null>
        {
            return;
        }
        if(((StringInput[i]>= 45)&&(StringInput[i]<=57))&&(StringInput[i]!=47)) //number type
        {
            if(d == 1)
            {
                d = 0;
                type[fieldCount] = 'n';
                pos[fieldCount++] = i;
            }
        }
        else if((StringInput[i]>= 97)&&(StringInput[i]<=122))    //alpha type
        {
            if(d == 1)
            {
                d = 0;
                type[fieldCount] = 'a';
                pos[fieldCount++] = i;
            }
        }
        else    // Assume delimiter
        {
            StringInput[i] = 0;
            d = 1;
        }
        i++;
    }
}

/*Step 4 - D.Chavez
 * helper functions for step 5
 *
 * isCommand returns true when the first value of the string equals the passed
 * command string, and has the number of minimum arguments
 *
 * getValue returns the raw bit value of a number in a string
 *
 * getString returns the substring requested
 *
 * NOTE: that both get helper functions are offset by one, to ignore the command
 * that is in position zero
 */
bool isCommand(char* strVerb, uint8_t minArgs)
{
    return(strcmp(&StringInput[pos[0]],strVerb)==0 &&fieldCount>minArgs);
}
uint16_t getValue(int8_t argNum)
{
    return atoi(&StringInput[pos[argNum+1]]);
}
char* getString(uint8_t argNum)
{
    return &StringInput[pos[argNum +1]];
}
// Acquired from http://www.strudel.org.uk/itoa/
/*
 *  This function has been put together by contributions from Stuart Lowe,
 *  Robert Jan Schaper, Ray-Yuan Sheu, Rodrigo de Salvo Braz, Wes Garland,
 *  John Maloney, Brian Hunt, Fernando Corradi and Lukás Chmela.
 */
char * itoa (int value, char *result, int base)
{
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}
//D.Chavez
//Timer Interrupt Handler
void timerISR()
{
    if(deviceMode)
    {
        if(phase == 1)
        {

            TIMER1_CTL_R = 0;   //Turn off timer

            GPIO_PORTC_AFSEL_R |= 0x20;
            phase = 2;
            putcUart1(0);   //first sent word is zero Start Code
            UART1_IM_R = UART_IM_TXIM;  //re-enable interrupts if not done so yet
        }
        if(phase == 0)
        {
            GPIO_PORTC_DATA_R |= 0x20;  //set TX pin to be high (mark after break)

            TIMER1_TAILR_R = 0x230;  //14 us delay
            phase = 1;
        }

        if(phase > 2 && dmxTx)
        {
            GPIO_PORTC_DATA_R &= 0xDF;  // set pin TX to be low for break
            TIMER1_TAILR_R = 0x1B80;  //176 us delay
            phase = 0;
        }
    }
    else
    {
        TIMER1_TAILR_R = 0x4C4B400; // 2 second delay
        GREEN_LED ^=1;  //Blink green LED for lost DMX data stream
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}
void processData()
{
    if(dmxData[address-1]>0)
    {
        BLUE_LED = 1;
    }
    else
    {
        BLUE_LED = 0;
    }
    //debug//
    //putsUart0(itoa(dmxData[address],result,10));
    //putsUart0("\r\n");
    //putsUart0(itoa(address,result,10));
    //putsUart0("\r\n");
}
void uart1ISR()
{
    if(deviceMode)
    {
        if(phase< (dmxMax + 2))
        {
            putcUart1(dmxData[phase-2]);
            phase++;
        }
        else
        {

            while (UART1_FR_R & UART_FR_BUSY);
            //waitMicrosecond(3500); //change
            UART1_IM_R &= ~UART_IM_TXIM;    // turn off uart 1 tx interrupt
            UART1_ICR_R |= 0x20;
            if(dmxTx)
            {
                GPIO_PORTC_DATA_R |= 0x80;  //pulse
                GPIO_PORTC_DATA_R &= 0x7F;
                //primeDmxTx();

                TIMER1_TAILR_R = 0x88B8;  //3500 clock delay
                //GPIO control
                GPIO_PORTC_AFSEL_R &= ~0x20;
                GPIO_PORTC_PCTL_R &= ~0x20;
                GPIO_PORTC_DIR_R |= 0x20;
                GPIO_PORTC_DATA_R |= 0x20; //set pin high before break
                phase = 999;
                TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

            }
        }
    }
    else
    {
        if(UART1_MIS_R & UART_MIS_BEMIS)
        {
            phase = 0;

            processData();
            uint8_t throw = UART1_DR_R;
            //waitMicrosecond();        // Debug
            TIMER1_TAILR_R = 0x4C4B400; // 2 second delay reload timer
            UART1_IM_R &= ~UART_IM_BEIM;    //change turn off break interrupts
            UART1_ICR_R |= UART_ICR_BEIC;
            //putsUart0("break");
            //putsUart0(UART1_MIS_R);
            GREEN_LED = 1;
        }
        else
        {
            //putsUart0("in else\r\n");
            if(phase>0)
            {

                dmxData[phase-1] = UART1_DR_R & 0xFF;
                //if(phase == 1)
                //{
                //    putsUart0(itoa(dmxData[phase-1],result,10));
                //}
                phase++;
                //UART1_ICR_R |= UART_ICR_RXIC;
                //putsUart0(itoa(phase,result,10));
                //putsUart0("\r\n");
                if(phase>dmxMax)
                {
                    UART1_IM_R |= UART_IM_BEIM;    //change turn on break interrupts
                    //waitMicrosecond(116);   //untested change
                    //putsUart0("\r\n");
                    //putsUart0("\r\n");
                }
            }
            else
            {
                //putsUart0("in start code else\r\n");
                uint8_t t = UART1_DR_R & 0xFF;
                //t = UART1_DR_R & 0xFF;
                if(t == 0)
                {
                    //putcUart0(t + 48);
                    //putsUart0("in start code\r\n");
                    phase = 1;

                    //UART1_ICR_R |= UART_ICR_RXIC;
                }
            }
        }
    }
}
void primeDmxTx()
{
    TIMER1_TAILR_R = 0x1B80;  //176 us delay
    //GPIO control
    GPIO_PORTC_DATA_R &= ~0x20;
    GPIO_PORTC_AFSEL_R &= ~0x20;
    //GPIO_PORTC_PCTL_R &= ~0x20; //change
    //GPIO_PORTC_DIR_R |= 0x20;  //change
    GPIO_PORTC_DATA_R &= 0xDF;  // set pin TX to be low for break
    phase = 0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}
/*Step 9*/

uint32_t get_EEPROM(char block, char offset)
{
    uint32_t data = 0;

    //Wait for EEPROM module to finish
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);

    //Set Block
    EEPROM_EEBLOCK_R = block;

    //Set Block Offset
    EEPROM_EEOFFSET_R = offset;

    //Read Data
    data = EEPROM_EERDWR_R;

    return data;
}

void set_EEPROM(char block, char offset, uint32_t data)
{
    //Wait for EEPROM module to finish
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);

    //Set Block
    EEPROM_EEBLOCK_R = block;

    //Set Block Offset
    EEPROM_EEOFFSET_R = offset;

    //Write Data
    EEPROM_EERDWR_R = data;
}

//D.Chavez
//recovery after reset or power failure
void recover()
{
    uint32_t init = get_EEPROM(0,0);
    if(init == 777)
    {
        deviceMode = get_EEPROM(0,1);
        address = get_EEPROM(0,2);
        setUpUart1();
        putsUart0("Address: ");
        char* val[5];
        itoa(address,val,10);
        putsUart0(val);
        putsUart0("\r\n");
        if(deviceMode){
            putsUart0("controller mode\r\n");
        }
        else
        {
            putsUart0("device mode\r\n");
        }
        putsUart0("\r\n");
    }
    else
    {
        set_EEPROM(0,0,777);
        set_EEPROM(0,1,false);
        set_EEPROM(0,2,1);
    }
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
/*Step5 - D.Chavez
 * Command Processing
 */
int main(void)
{
    // Initialize hardware
    initHw();

    // Flash red LED for confirmation/debugging
    flashRedLED();

    //clear terminal screen
    clearScreen();
    recover();

    // Endless loop variables
    uint16_t add = 0; // Address
    uint8_t data = 0;
    bool ok;
    char * val[4];

    while(1){
        ok = false;
        getsUart0(MAX_CHARS);
        parceStr();
        if(isCommand("set",2))
        {
            if(deviceMode){
                add = getValue(0);
                data = getValue(1);
                if((add>0&&add<=512)&&(getValue(1)<=255))
                {
                    dmxData[add-1] = data;
                    ok = true;
                    if(fieldCount>3)
                    {
                        putsUart0("Warning extra arguments are ignored\r\n");
                    }
                }
            }
            else
            {
                putsUart0("warning in device mode\r\n");
            }
        }
        if(isCommand("get",1))
        {
            add = getValue(0);
            if(add>0&&add<=512)
            {
                itoa(dmxData[add-1],val,10);
                putsUart0("Address ");
                putsUart0(&StringInput[pos[1]]);
                putsUart0(" is set to: ");
                putsUart0(val);
                putsUart0("\r\n");
                ok = true;
            }
        }
        if(isCommand("clear",0))
        {
            if(deviceMode)
            {
                uint16_t i = 0;
                for(i;i<512;i++)
                {
                    dmxData[i]=0;
                }
                ok = true;
            }
            else
            {
                putsUart0("warning in device mode\r\n");
            }
        }
        if(isCommand("on",0))
        {
            if(deviceMode)
            {
                if(!dmxTx)
                {
                    ok = true;
                    dmxTx = true;
                    RED_LED = 1;
                    phase = 0;
                    primeDmxTx();
                }
                else
                {
                    putsUart0("DMX Tx is already enabled\r\n");
                    ok = true;
                }
            }
            else
            {
                putsUart0("warning in device mode\r\n");
            }
        }
        if(isCommand("off",0))
        {
            if(deviceMode)
            {
                if(dmxTx)
                {
                    ok = true;
                    dmxTx = false;
                    RED_LED = 0;
                }
                else
                {
                    putsUart0("DMX Tx is already disabled\r\n");
                    ok = true;
                }
            }
            else
            {
                putsUart0("warning in device mode\r\n");
            }
        }
        if(isCommand("cs",0))
        {
            clearScreen();
            ok = true;
        }
        if(isCommand("max",1))
        {
                add = getValue(0);
                if(add<=512)
                {
                    dmxMax = add;
                    ok = true;
                }
        }
        if(isCommand("address",1))
        {
            if(!deviceMode)
            {
                add = getValue(0);
                if(add>0&&add<=512)
                {
                    address = add;
                    set_EEPROM(0,2,address);
                    ok = true;
                }
            }
            else
            {
                putsUart0("warning in controller mode\r\n");
            }
        }
        if(isCommand("controller",0))
        {
            deviceMode = true;
            set_EEPROM(0,1,deviceMode);
            setUpUart1();
            ok = true;
        }
        if(isCommand("device",0))
        {
            if(!dmxTx){
                deviceMode = false;
                set_EEPROM(0,1,deviceMode);
                setUpUart1();
                ok = true;
            }
            else
            {
                putsUart0("Turn off transmit before switching modes\r\n");
            }
        }
        if(!ok)
        {
            putsUart0("Error in command\r\n");
            putsUart0(&StringInput);
            putsUart0("\r\n");

            putcUart0(fieldCount+48);
        }
    }
}
