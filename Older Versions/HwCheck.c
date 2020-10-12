#include <msp430.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*Notes:
LCD_byte should be a define


*/

//#define LCD_NEG         BIT1  //P2.1 Charge pump
#define LCD_EN          BIT0  //P2.0 LCD clock "Enable"

#define ADDRESS_LATCH   BIT3  //P1.3 SRAM 595 latches
#define MEM_LATCH       BIT1  //P2.1 SRAM 165 latch

#define RAM_OE          BIT0  //P1.0 SRAM #OE
#define RAM_WE          BIT1  //P1.1 SRAM #WE
#define RAM_595         BIT2  //P1.2 SRAM 595 #OE
#define RAM_BANK        BIT2  //P2.2 SRAM bank

#define IN_LATCH        BIT4  //P1.4 Aux 165 latch (shared with MEM_LATCH)
#define OUT_LATCH       BIT4  //P2.4 Aux 595 latch. pin 12
#define SR_CLOCK        BIT5  //P2.5 Aux clocks. pin 11
#define SR_DATA         BIT3  //P2.3 Aux data lines. pin 14

//#define CPU16MHZ
#define CPU1MHZ

#ifdef CPU16MHZ
  #define MS_CYCLES 16000     //Length of a millisecond
  #define CHARGE_CYCLES 1600  //Charge pump discharge time
  #define LCD_CYCLES 300      //LCD Enable delay (Max 2MHz)
#endif
#ifdef CPU1MHZ
  #define MS_CYCLES 1000     //Length of a millisecond
  #define CHARGE_CYCLES 200  //Charge pump discharge time
  #define LCD_CYCLES 0      //LCD Enable delay (Max 2MHz)
#endif

void delay_ms(int ms);
void LCD_Nibble(char nibble, unsigned char RS);
void LCD_Byte(char byte, unsigned char RS);
void LCD_Init();
void LCD_Text(const char *msg);
void LCD_Num(unsigned char num);

unsigned char RAM_Read(unsigned char *a1);
void RAM_Write(unsigned char *a1, unsigned char byte);

void HardwareTest();

unsigned char GetKeys();

int main(void)
{
  unsigned char buff,key;

  WDTCTL=WDTPW + WDTHOLD;

  #ifdef CPU16MHZ
  BCSCTL1=CALBC1_16MHZ;
  DCOCTL=CALDCO_16MHZ;
  #endif
  #ifdef CPU1MHZ
  BCSCTL1=CALBC1_1MHZ;
  DCOCTL=CALDCO_1MHZ;
  #endif

  BCSCTL3 |= LFXT1S_2;

  UCB0CTL1=UCSWRST;
  P1SEL=BIT5|BIT6|BIT7;
  P1SEL2=BIT5|BIT6|BIT7;
  UCB0CTL0=UCCKPL|UCMST|UCSYNC;//|UCMSB;
  UCB0CTL1|=UCSSEL_2;
  UCB0BR1=0;
  UCB0BR0=0;
  UCB0CTL1&=~UCSWRST;

  P2OUT=OUT_LATCH|MEM_LATCH;
  //This is important!
  P1OUT=RAM_OE|RAM_WE|RAM_595;
  P1OUT|=IN_LATCH;

  P1DIR=0xFF;
  P2DIR=0xFF;

  TACCR0 = 200;
  TACCTL0 = CCIE;
  TACTL = MC_1|ID_0|TASSEL_1|TACLR;
  __enable_interrupt();

  LCD_Init();

  HardwareTest(255,255);

  for (;;) {_BIS_SR(LPM3_bits);}

  return 0;
}

//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void Timer_A (void)
__attribute__((interrupt(TIMER0_A0_VECTOR))) static void TA0_ISR(void)
{
  //P2OUT&=~LCD_NEG;
  //__delay_cycles(CHARGE_CYCLES);
  //P2OUT|=LCD_NEG;
}

unsigned char RAM_Read(unsigned char *a1)
{
  //while (UCB0STAT & UCBUSY);
  UCB0TXBUF=0;
  UCB0TXBUF=((unsigned int)a1)>>8;
  while(!(UC0IFG & UCA0TXIFG));
  UCB0TXBUF=((unsigned int)a1)&0xFF;
  while (UCB0STAT & UCBUSY);
  P1OUT&=~ADDRESS_LATCH;
  P1OUT|=ADDRESS_LATCH;
  P1OUT&=~RAM_OE;
  P2OUT&=~MEM_LATCH;
  P2OUT|=MEM_LATCH;
  UCB0TXBUF=0;
  while (UCB0STAT & UCBUSY);
  P1OUT|=RAM_OE;
  return UCB0RXBUF;
}

void RAM_Write(unsigned char *a1, unsigned char byte)
{
  /*LCD_Byte(1,0);
  delay_ms(2);
  LCD_Num(((unsigned int)a1)>>8);
  LCD_Byte(' ',1);
  LCD_Num(((unsigned int)a1)&0xFF);
  delay_ms(1000);*/

  //while (UCB0STAT & UCBUSY);
  UCB0TXBUF=byte;
  UCB0TXBUF=((unsigned int)a1)>>8;
  while(!(UC0IFG & UCA0TXIFG));
  UCB0TXBUF=((unsigned int)a1)&0xFF;
  while (UCB0STAT & UCBUSY);
  P1OUT&=~ADDRESS_LATCH;
  P1OUT|=ADDRESS_LATCH;
  P1OUT|=RAM_OE;
  P1OUT&=~RAM_595;
  P1OUT&=~RAM_WE;
  P1OUT|=RAM_WE;
  P1OUT|=RAM_595;
}

void HardwareTest(unsigned int jmax, unsigned int imax)
{
  unsigned char buff,key;
  LCD_Byte(1,0);
  delay_ms(2);
  LCD_Text("RAM test");
  LCD_Byte(0x80+0x40,0);
  LCD_Text("Press any key");
  //while(!GetKeys()){}

  P2OUT&=~RAM_BANK;
  unsigned int i,j,k;
  for (k=0;k<2;k++)
  {
    LCD_Byte(1,0);
    delay_ms(2);
    LCD_Text("Port ");
    if (P2OUT&RAM_BANK) LCD_Byte('1',1);
    else  LCD_Byte('0',1);
    LCD_Byte(0x80+0x40,0);
    LCD_Text("Writing: ");
    for (j=0;j<=jmax;j++)
    {
      LCD_Byte(0x80+0x49,0);
      LCD_Num(j);
      for (i=0;i<=imax;i++) RAM_Write((unsigned char*)((i<<8)+j),i+j);
    }

    LCD_Byte(1,0);
    delay_ms(2);
    LCD_Text("Port ");
    if (P2OUT&RAM_BANK) LCD_Byte('1',1);
    else  LCD_Byte('0',1);
    LCD_Byte(0x80+0x40,0);
    LCD_Text("Reading: ");
    for (j=0;j<=jmax;j++)
    {
      LCD_Byte(0x80+0x49,0);
      LCD_Num(j);
      for (i=0;i<=imax;i++)
      {
        buff=RAM_Read((unsigned char*)((i<<8)+j));
        if (buff!=((i+j)&0xFF))
        {
          LCD_Byte(1,0);
          delay_ms(2);
          LCD_Text("Test Failed");
          LCD_Byte(0x80+0x40,0);
          LCD_Text("A:");
          LCD_Num(i);
          LCD_Num(j);
          LCD_Text("|");
          LCD_Num(buff);
          LCD_Text("=");
          LCD_Num(i+j);
          for (;;) {_BIS_SR(LPM3_bits);}
        }
      }
    }
    P2OUT|=RAM_BANK;
  }
  LCD_Byte(1,0);
  delay_ms(2);
  LCD_Text("Test Complete");
  LCD_Byte(0x80+0x40,0);
  LCD_Text("Press any key");
  while(!GetKeys()){}

  LCD_Byte(1,0);
  delay_ms(2);
  LCD_Text("Key test");
  LCD_Byte(0x80+0x40,0);
  LCD_Text("Press any key");
  while(!GetKeys()){}
  LCD_Byte(0x80+0x40,0);
  LCD_Text("Key:       ");
  for (;;)
  {
    key=GetKeys();
    if (key)
    {
      LCD_Byte(0x80+0x45,0);
      LCD_Num(key);
    }
    delay_ms(100);
  }
}

unsigned char GetKeys()
{
  unsigned char i,j;

  for (j=0;j<8;j++)
  {
    P2DIR|=SR_DATA;
    P2OUT&=~SR_DATA;

    for (i=0;i<8;i++)
    {
      if (i==j) P2OUT|=SR_DATA;
      else P2OUT&=~SR_DATA;
      P2OUT&=~SR_CLOCK;
      P2OUT|=SR_CLOCK;
    }

    //P2OUT|=OUT_LATCH;
    P2OUT&=~OUT_LATCH;
    P2OUT|=OUT_LATCH;

    delay_ms(1);

    //P1OUT|=IN_LATCH;
    P1OUT&=~IN_LATCH;
    P1OUT|=IN_LATCH;

    P2DIR&=~SR_DATA;

    for (i=0;i<8;i++)
    {
      if ((P2IN & SR_DATA)==0)
      {
     //   result++;
      }
      else
      {
        return j*8+i;
      }
      P2OUT&=~SR_CLOCK;
      P2OUT|=SR_CLOCK;
    }
  }
  return 0;
}

void delay_ms(int ms)
{
  volatile int i;
  for (i=0;i<ms;i++) __delay_cycles(MS_CYCLES);
}

void LCD_Nibble(char nibble, unsigned char RS)
{
  volatile int j=0;

  P2DIR|=SR_DATA;

  for (j=8;j>0;j/=2)
  {
    if (nibble&j) P2OUT|=SR_DATA;
    else P2OUT&=~SR_DATA;
    P2OUT&=~SR_CLOCK;
    P2OUT|=SR_CLOCK;
  }

  if (RS!=0) P2OUT|=SR_DATA;
  else P2OUT&=~SR_DATA;

  for (j=0;j<4;j++)
  {
    P2OUT&=~SR_CLOCK;
    P2OUT|=SR_CLOCK;
  }

  P2OUT&=~OUT_LATCH;
  P2OUT|=OUT_LATCH;

  P2OUT|=LCD_EN;
  __delay_cycles(LCD_CYCLES);
  P2OUT&=~LCD_EN;
}

void LCD_Byte(char byte, unsigned char RS)
{
  LCD_Nibble(byte>>4,RS);
  LCD_Nibble(byte,RS);
}

#define DELAY_SMALL 5
#define DELAY_MEDIUM 30
#define DELAY_LARGE 500

void LCD_Init()
{
  delay_ms(DELAY_LARGE);
  LCD_Nibble(0x3,0);
  delay_ms(DELAY_MEDIUM);
  LCD_Nibble(0x3,0);
  delay_ms(DELAY_SMALL);
  LCD_Nibble(0x3,0);
  delay_ms(DELAY_SMALL);
  LCD_Nibble(0x2,0);
  delay_ms(DELAY_SMALL);

  LCD_Byte(0x28,0);
  delay_ms(DELAY_SMALL);
  LCD_Byte(0xE,0);
  delay_ms(DELAY_SMALL);
  LCD_Byte(0x1,0);
  delay_ms(DELAY_SMALL);
  LCD_Byte(0x6,0);
  delay_ms(DELAY_SMALL);

  LCD_Byte(0xC,0);
  delay_ms(DELAY_SMALL);
}

void LCD_Text(const char *msg)
{
  int ptr=0;

  while (msg[ptr]!=0)
  {
    LCD_Byte(msg[ptr],1);
    ptr++;
  }
}

void LCD_Num(unsigned char num)
{
  LCD_Byte(num/100+'0',1);
  num-=(num/100*100);
  LCD_Byte(num/10+'0',1);
  num-=(num/10*10);
  LCD_Byte(num+'0',1);
}

