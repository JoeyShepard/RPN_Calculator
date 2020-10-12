#include <msp430.h>
#include <stdio.h>

//XIN = P2.6 - connected to P1.0
//XOUT = P2.7 - connected to P1.1
#define SLAVE_DATA      BIT0  //P1.0
#define SLAVE_CLOCK     BIT1  //P1.1

#define ADDRESS_LATCH   BIT2  //P1.2 SRAM 595 latches
#define RAM_OE          BIT3  //P1.3 SRAM #OE
#define RAM_WE          BIT4  //P1.4 SRAM #WE
#define LED             BIT6  //P1.6 LED



enum SlaveCommands {ECHO, LISTEN};
#define SLAVE_WAIT 200
#define SLAVE_WAIT_SMALL 100

static void delay_ms(int ms);
static void RAM_Write(const unsigned char *a1, const unsigned char byte);
static unsigned char RAM_Read(const unsigned char *a1);
static void SlaveSend(unsigned int value);
static unsigned int SlaveReceive();

int main(void)
{
  WDTCTL=WDTPW + WDTHOLD;

  BCSCTL1=CALBC1_16MHZ;
  DCOCTL=CALDCO_16MHZ;

  UCB0CTL1=UCSWRST;
  P1SEL=BIT5|BIT7;
  P1SEL2=BIT5|BIT7;
  P2SEL=0;
  P2SEL2=0;
  UCB0CTL0=UCCKPL|UCMST|UCSYNC;//|UCMSB;
  UCB0CTL1|=UCSSEL_2;
  UCB0BR1=0;
  UCB0BR0=0;
  UCB0CTL1&=~UCSWRST;

  //This is important!
  P1OUT=RAM_OE|RAM_WE;
  //P1OUT&=~SLAVE_CLOCK;
  P1DIR=RAM_OE|RAM_WE|ADDRESS_LATCH|LED;

  P2DIR=0;
  unsigned int value,v2,address;
  unsigned int i=0,j;

  for (;;)
  {
    for (i=0;i<256;i++)
    {
      address=i<<8;
      SlaveSend(0);
      SlaveSend(i);
      delay_ms(10);
      for (j=0;j<256;j++)
      {
        RAM_Write((unsigned char *)address,j&0xFF);
        value=RAM_Read((unsigned char *)address);
        if (value!=j)
        {
          SlaveSend(1);
          SlaveSend(j);
          SlaveSend(value);
          P1OUT|=LED;
          delay_ms(1000);
        }
        address++;
      }
    }
  }

  for (;;)
  {
    value=SlaveReceive();
    switch(value)
    {
      case ECHO:
        value=SlaveReceive();
        address=SlaveReceive();
        v2=value;
        RAM_Write((unsigned char *)address,value&0xFF);
        value=RAM_Read((unsigned char *)address);
        //problem with ram, not link
        if (value!=v2)
        {
          P1OUT|=LED;
          //for (;;);
        }
        //__delay_cycles(200);
        SlaveSend(value);
        SlaveSend(address);
        break;
    }
    if (i++==256)
    {
      i=0;
      //P1OUT^=LED;
    }
  }
}

static void delay_ms(int ms)
{
  volatile int i;
  for (i=0;i<ms;i++) __delay_cycles(16000);
}

static unsigned char RAM_Read(const unsigned char *a1)
{
  unsigned char value;
  UCB0TXBUF=((unsigned int)a1)>>8;
  UCB0TXBUF=((unsigned int)a1)&0xFF;
  __delay_cycles(50);
  P1OUT&=~ADDRESS_LATCH;
  P1OUT|=ADDRESS_LATCH;
  P1OUT&=~RAM_OE;

  //__delay_cycles(10);

  value=P2IN;
  P1OUT|=RAM_OE;
  return value;
}

//make while into nop
static void RAM_Write(const unsigned char *a1, const unsigned char byte)
{
  //*(((unsigned char*)&foo)+1) should be faster
  UCB0TXBUF=((unsigned int)a1)>>8;
  UCB0TXBUF=((unsigned int)a1)&0xFF;
  __delay_cycles(50);
  P1OUT&=~ADDRESS_LATCH;
  P1OUT|=ADDRESS_LATCH;
  //this is redundant
  P1OUT|=RAM_OE;
  P2OUT=byte;
  P2DIR=0xFF;
  __delay_cycles(10);
  P1OUT&=~RAM_WE;
  P1OUT|=RAM_WE;
  __delay_cycles(10);
  P2DIR=0;
}

static void SlaveSend(unsigned int value)
{
  int i;
  P1OUT&=~(SLAVE_DATA+SLAVE_CLOCK);
  P1DIR|=(SLAVE_DATA+SLAVE_CLOCK);
  __delay_cycles(SLAVE_WAIT);
  P1OUT|=SLAVE_CLOCK;
  for (i=0;i<16;i++)
  {
    if (value&1) P1OUT|=SLAVE_DATA;
    else P1OUT&=~SLAVE_DATA;
    value/=2;
    __delay_cycles(SLAVE_WAIT);
    P1OUT&=~SLAVE_CLOCK;
    __delay_cycles(SLAVE_WAIT);
    P1OUT|=SLAVE_CLOCK;
  }
  P1DIR&=~(SLAVE_DATA+SLAVE_CLOCK);
  P1OUT&=~(SLAVE_DATA+SLAVE_CLOCK);
}

static unsigned int SlaveReceive()
{
  unsigned int i,value=0;
  __delay_cycles(SLAVE_WAIT_SMALL);
  while (!(P1IN&SLAVE_CLOCK));
  for (i=0;i<16;i++)
  {
    while (P1IN&SLAVE_CLOCK);
    if (P1IN&SLAVE_DATA) value+=(1U<<i);
    //P1OUT|=LED;
    while (!(P1IN&SLAVE_CLOCK));
    //P1OUT&=~LED;
  }
  return value;
}
