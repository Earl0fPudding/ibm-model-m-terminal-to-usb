/****************************************************************************
 Title	:   HD44780U LCD library
 Author:    Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 File:	    $Id: lcd.c,v 1.13 2003/07/13 07:33:10 peter Exp $

 DESCRIPTION
       Basic routines for interfacing a HD44780U-based text lcd display

       Originally based on Volker Oth's lcd library,
       changed lcd_init(), added additional constants for lcd_command(),
       added 4-bit I/O mode, improved and optimized code.

       Library can be operated in memory mapped mode (LCD_IO_MODE=0) or in
       4-bit IO port mode (LCD_IO_MODE=1). 8-bit IO port mode not supported.

       Memory mapped mode compatible with Kanda STK200, but supports also
       generation of R/W signal through A8 address line.

 USAGE
       See the C include lcd.h file for a description of each function

*****************************************************************************/
#ifndef _bool_h
#define _bool_h

#ifndef bool
typedef unsigned char bool;
#endif
#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif

#endif

#include <inttypes.h>
#include "lcd.h"



/*
** constants/macros
*/
//#define PIN(x) (*(&x - 2))  /* address of data direction register of port x */
//#define DDR(x) (*(&x - 1))  /* address of input register of port x          */


//#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );
#define lcd_e_high()    LCD_E_PORT->ODR  |=  (1 << LCD_E_PIN);
#define lcd_e_low()     LCD_E_PORT->ODR  &= ~(1 << LCD_E_PIN);
#define lcd_e_toggle()  toggle_e()
#define lcd_rw_high()   LCD_RW_PORT->ODR |=  (1 << LCD_RW_PIN)
#define lcd_rw_low()    LCD_RW_PORT->ODR &= ~(1 << LCD_RW_PIN)
#define lcd_rs_high()   LCD_RS_PORT->ODR |=  (1 << LCD_RS_PIN)
#define lcd_rs_low()    LCD_RS_PORT->ODR &= ~(1 << LCD_RS_PIN)
//#define set_data(nib)   LCD_DATA_PORT = (LCD_DATA_PORT & 0x0f) | (nib)

#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES
#endif




static void set_data(uint8_t nib){
    LCD_DATA_PORT->ODR &= ~( (1 << LCD_DATA_PIN_0) | (1 << LCD_DATA_PIN_1) | (1 << LCD_DATA_PIN_2) | (1 << LCD_DATA_PIN_3) );
    if(nib & 0b00010000) { LCD_DATA_PORT->ODR|=(1 << LCD_DATA_PIN_0); }
    if(nib & 0b00100000){ LCD_DATA_PORT->ODR|=(1 << LCD_DATA_PIN_1); }
    if(nib & 0b01000000){ LCD_DATA_PORT->ODR|=(1 << LCD_DATA_PIN_2); }
    if(nib & 0b10000000){ LCD_DATA_PORT->ODR|=(1 << LCD_DATA_PIN_3); }
}

/*
** function prototypes
*/
static void delay_us(uint16_t us);
static void toggle_e(void);

static void lcd_e_delay(void){
    wait_us(1);
}

/*
** local functions
*/

/*************************************************************************
delay for a minimum of <us> microseconds
with a 4Mhz crystal, the resolution is 1 us
*************************************************************************/
static void delay_us(uint16_t us)
{
    uint32_t time=us*10;
    for(volatile long i=0; i<time; i++){}
}


/* toggle Enable Pin to initiate write */
static void toggle_e(void)
{
    lcd_e_high();
    lcd_e_delay();
    lcd_e_low();
}

static void init_gpio_registers(void){
    // RCC->AHB2ENR|= (1 << RCC_AHB2ENR_GPIOCEN) | (1 << RCC_AHB2ENR_GPIOBEN) | (1 << RCC_AHB2ENR_GPIOBEN) | (1 << RCC_AHB2ENR_GPIOBEN); // enable clock for gpios

    // Data port registers
    LCD_DATA_PORT->MODER &= ~( (0b10 << LCD_DATA_PIN_0*2) | (0b10 << LCD_DATA_PIN_1*2) | (0b10 << LCD_DATA_PIN_2*2) | (0b10 << LCD_DATA_PIN_3*2) );
    LCD_DATA_PORT->MODER |= (1 << LCD_DATA_PIN_0*2) | (1 << LCD_DATA_PIN_1*2) | (1 << LCD_DATA_PIN_2*2) | (1 << LCD_DATA_PIN_3*2);
    LCD_DATA_PORT->OTYPER &= ~( (1 << LCD_DATA_PIN_0) | (1 << LCD_DATA_PIN_1) | (1 << LCD_DATA_PIN_2) | (1 << LCD_DATA_PIN_3) );
    LCD_DATA_PORT->OSPEEDR |= (0b11 << LCD_DATA_PIN_0*2) | (0b11 << LCD_DATA_PIN_1*2) | (0b11 << LCD_DATA_PIN_2*2) | (0b11 << LCD_DATA_PIN_3*2);
    LCD_DATA_PORT->PUPDR &= ~((0b11 << LCD_DATA_PIN_0*2) | (0b11 << LCD_DATA_PIN_1*2) | (0b11 << LCD_DATA_PIN_2*2) | (0b11 << LCD_DATA_PIN_3*2));

    // rs port registers
    LCD_RS_PORT->MODER &= ~( (0b10 << LCD_RS_PIN*2) );
    LCD_RS_PORT->MODER |= (1 << LCD_RS_PIN*2);
    LCD_RS_PORT->OTYPER &= ~( (1 << LCD_RS_PIN) );
    LCD_RS_PORT->OSPEEDR |= (0b11 << LCD_RS_PIN*2);
    LCD_RS_PORT->PUPDR &= ~((0b11 << LCD_RS_PIN*2));

    // rw port registers
    LCD_RW_PORT->MODER &= ~( (0b10 << LCD_RW_PIN*2) );
    LCD_RW_PORT->MODER |= (1 << LCD_RW_PIN*2);
    LCD_RW_PORT->OTYPER &= ~( (1 << LCD_RW_PIN) );
    LCD_RW_PORT->OSPEEDR |= (0b11 << LCD_RW_PIN*2);
    LCD_RW_PORT->PUPDR &= ~((0b11 << LCD_RW_PIN*2));

    // e port registers
    LCD_E_PORT->MODER &= ~( (0b10 << LCD_E_PIN*2) );
    LCD_E_PORT->MODER |= (1 << LCD_E_PIN*2);
    LCD_E_PORT->OTYPER &= ~( (1 << LCD_E_PIN) );
    LCD_E_PORT->OSPEEDR |= (0b11 << LCD_E_PIN*2);
    LCD_E_PORT->PUPDR &= ~((0b11 << LCD_E_PIN*2));

}

/*************************************************************************
Low-level function to write byte to LCD controller
Input:    data   byte to write to LCD
          rs     1: write data
                 0: write instruction
Returns:  none
*************************************************************************/
static void lcd_write(uint8_t data,uint8_t rs)
{
    /* configure data pins as output */
    lcd_rw_low();

    if (rs) {   /* write data        (RS=1, RW=0) */

        lcd_rs_high();

    } else {    /* write instruction (RS=0, RW=0) */

        lcd_rs_low();
    }

    /* output high nibble first */
    set_data(data & 0xF0);
    lcd_e_toggle();

    /* output low nibble */
    set_data((data<<4)&0xF0);


    lcd_e_toggle();

    /* all data pins high (inactive) */
    set_data(0xF0);
}


/*************************************************************************
Low-level function to read byte from LCD controller
Input:    rs     1: read data
                 0: read busy flag / address counter
Returns:  byte read from LCD controller
*************************************************************************/
/*
#if LCD_IO_MODE
static uint8_t lcd_read(uint8_t rs)
{
    register uint8_t dataH, dataL;

    if (rs)
        lcd_rs_high();                       /* RS=1: read data
    else
        lcd_rs_low();                        /* RS=0: read busy flag
    lcd_rw_high();                           /* RW=1  read mode

    /* configure data pins as input
    DDR(LCD_DATA_PORT) &= 0x0f;

    lcd_e_high();
    lcd_e_delay();
    dataH = PIN(LCD_DATA_PORT);              /* read high nibble first
    lcd_e_low();

    lcd_e_delay();                           /* Enable 500ns low

    lcd_e_high();
    lcd_e_delay();
    dataL = PIN(LCD_DATA_PORT);              /* read low nibble
    lcd_e_low();

    return ( (dataH&0xf0) | (dataL>>4) );
}
#else
#define lcd_read(rs) (rs) ? *(volatile uint8_t*)(LCD_IO_DATA+LCD_IO_READ) : *(volatile uint8_t*)(LCD_IO_FUNCTION+LCD_IO_READ)
/* rs==0 -> read instruction from LCD_IO_FUNCTION */
/* rs==1 -> read data from LCD_IO_DATA */
//#endif


/*************************************************************************
loops while lcd is busy, returns address counter
*************************************************************************/
static uint8_t lcd_waitbusy(void)
{
    wait_us(64);
    /* register uint8_t c;

    /* wait until busy flag is cleared
    while ( (c=lcd_read(0)) & (1<<LCD_BUSY)) {}

    /* the address counter is updated 4us after the busy flag is cleared
    delay(2);

    /* now read the address counter
    return (lcd_read(0));  // return address counter
*/
}/* lcd_waitbusy */


/*************************************************************************
Move cursor to the start of next line or to the first line if the cursor
is already on the last line.
*************************************************************************/
static void lcd_newline(uint8_t pos)
{
    register uint8_t addressCounter;


#if LCD_LINES==1
    addressCounter = 0;
#endif
#if LCD_LINES==2
    if ( pos < (LCD_START_LINE2) )
        addressCounter = LCD_START_LINE2;
    else
        addressCounter = LCD_START_LINE1;
#endif
#if LCD_LINES==4
    if ( pos < LCD_START_LINE3 )
        addressCounter = LCD_START_LINE2;
    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE4) )
        addressCounter = LCD_START_LINE3;
    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE2) )
        addressCounter = LCD_START_LINE4;
    else
        addressCounter = LCD_START_LINE1;
#endif
    lcd_command((1<<LCD_DDRAM)+addressCounter);

}/* lcd_newline */


/*
** PUBLIC FUNCTIONS
*/

/*************************************************************************
Send LCD controller instruction command
Input:   instruction to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_command(uint8_t cmd)
{
    lcd_waitbusy();
    lcd_write(cmd,0);
}


/*************************************************************************
Send LCD controller data send
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_data(uint8_t dat)
{
    lcd_waitbusy();
    lcd_write(dat,1);
}


/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
#if LCD_LINES==1
    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
#endif
#if LCD_LINES==2
    if ( y==0 )
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
#endif
#if LCD_LINES==4
    if ( y==0 )
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else if ( y==1)
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
    else if ( y==2)
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE3+x);
    else /* y==3 */
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE4+x);
#endif

}/* lcd_gotoxy */


/*************************************************************************
*************************************************************************/
int lcd_getxy(void)
{
    return lcd_waitbusy();
}


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
    lcd_command(1<<LCD_CLR);
}


/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
    lcd_command(1<<LCD_HOME);
}


/*************************************************************************
Display character at current cursor position
Input:    character to be displayed
Returns:  none
*************************************************************************/
void lcd_putc(char c)
{
    uint8_t pos;

    lcd_waitbusy();
    // pos = lcd_waitbusy();   // read busy-flag and address counter
    if (c=='\n')
    {
        lcd_newline(0);
    }
    else
    {
#if LCD_WRAP_LINES==1
        #if LCD_LINES==1
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH )
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
#elif LCD_LINES==2
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH )
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);
        else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH )
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
#elif LCD_LINES==4
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH )
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);
        else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH )
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE3,0);
        else if ( pos == LCD_START_LINE3+LCD_DISP_LENGTH )
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE4,0);
        else if ( pos == LCD_START_LINE4+LCD_DISP_LENGTH )
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
#endif
        lcd_waitbusy();
#endif
        lcd_write(c, 1);
    }

}/* lcd_putc */


/*************************************************************************
Display string without auto linefeed
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */


/*************************************************************************
Initialize display and select type of cursor
Input:    dispAttr LCD_DISP_OFF            display off
                   LCD_DISP_ON             display on, cursor off
                   LCD_DISP_ON_CURSOR      display on, cursor on
                   LCD_DISP_CURSOR_BLINK   display on, cursor on flashing
Returns:  none
*************************************************************************/
void lcd_init(uint8_t dispAttr)
{
    /*
     *  Initialize LCD to 4 bit I/O mode
     */

    init_gpio_registers();
    lcd_rs_low();
    lcd_rw_low();
    lcd_e_low();
    wait_ms(16);                           /* wait 16ms after power-on     */

    /* initial write to lcd is 8bit */
    set_data(LCD_FUNCTION_8BIT_1LINE);

    lcd_e_toggle();
    wait_ms(5);         /* delay, busy flag can't be checked here */

    set_data(LCD_FUNCTION_8BIT_1LINE);
    lcd_e_toggle();
    wait_us(64);           /* delay, busy flag can't be checked here */

    set_data(LCD_FUNCTION_8BIT_1LINE);
    lcd_e_toggle();
    wait_us(64);           /* delay, busy flag can't be checked here */

    set_data(LCD_FUNCTION_4BIT_1LINE); /* set IO mode to 4bit */
    lcd_e_toggle();
    wait_us(64);           /* some displays need this additional delay */

    /* from now the LCD only accepts 4 bit I/O, we can use lcd_command() */
    lcd_command(LCD_FUNCTION_DEFAULT);      /* function set: display lines  */
    lcd_command(LCD_DISP_OFF);              /* display off                  */
    lcd_clrscr();                           /* display clear                */
    lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
    lcd_command(dispAttr);                  /* display/cursor control       */

}/* lcd_init */
