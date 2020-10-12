#include <msp430.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*Notes:
LCD_byte should be a define

*/

#define LCD_NEG         BIT1  //P2.1 Charge pump
#define LCD_EN          BIT0  //P2.0 LCD clock "Enable"

#define ADDRESS_LATCH   BIT3  //P1.3 SRAM 595 latches
#define MEM_LATCH       BIT4  //P1.4 SRAM 165 latch (shared with IN_LATCH)

#define RAM_OE          BIT0  //P1.0 SRAM #OE
#define RAM_WE          BIT1  //P1.1 SRAM #WE
#define RAM_595         BIT2  //P1.2 SRAM 595 #OE
#define RAM_BANK        BIT2  //P2.2 SRAM bank

#define IN_LATCH        BIT4  //P1.4 Aux 165 latch (shared with MEM_LATCH)
#define OUT_LATCH       BIT4  //P2.4 Aux 595 latch. pin 12
#define SR_CLOCK        BIT5  //P2.5 Aux clocks. pin 11
#define SR_DATA         BIT3  //P2.3 Aux data lines. pin 14

#define CPU16MHZ
//#define CPU1MHZ

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

#define BCD_SIGN 0
#define BCD_LEN  1
#define BCD_DEC  2

#define putchar(x) LCD_Byte(x,1)

#pragma MM_READ RAM_Read
#pragma MM_WRITE RAM_Write
#pragma MM_ON

#define DEC_PLACES 6

#pragma MM_VAR p0
#pragma MM_VAR p1
#pragma MM_VAR p2
#pragma MM_VAR p3
#pragma MM_VAR p4
#pragma MM_VAR p5
#pragma MM_VAR p6
#pragma MM_VAR p7
#pragma MM_VAR buffer
#pragma MM_VAR perm_buff1
#pragma MM_VAR perm_buff2
#pragma MM_VAR perm_buff3

#pragma MM_OFFSET 2000
#pragma MM_GLOBALS
  unsigned char p0[260];
  unsigned char p1[260];
  unsigned char p2[260];
  unsigned char p3[260];
  unsigned char p4[260];
  unsigned char p5[260];
  unsigned char p6[260];
  unsigned char p7[260];
  unsigned char buffer[260];
  unsigned char perm_buff1[260];
  unsigned char perm_buff2[260];
  unsigned char perm_buff3[260];
#pragma MM_END

void delay_ms(int ms);
void LCD_Nibble(char nibble, unsigned char RS);
void LCD_Byte(char byte, unsigned char RS);
void LCD_Init();
void LCD_Text(const char *msg);
void LCD_Num(unsigned char num);

unsigned char GetKeys();

unsigned char RAM_Read(const unsigned char *a1);
void RAM_Write(const unsigned char *a1, const unsigned char byte);
void AddBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2);
void SubBCD(unsigned char *result, const unsigned char *n1, unsigned char *n2);
void ImmedBCD(unsigned char *text, unsigned char *BCD);
void PrintBCD(const unsigned char *BCD);
void BufferBCD(const unsigned char *BCD, unsigned char *buffer);
void MultBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2);
void DivBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2);
void ShrinkBCD(unsigned char *n1);
void FullShrinkBCD(unsigned char *n1);
void PadBCD(unsigned char *n1, int amount);
bool IsZero(unsigned char *n1);
void CopyBCD(unsigned char *dest, unsigned char *src);
void LnBCD(unsigned char *result, unsigned char *arg);

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

  P2OUT=OUT_LATCH|LCD_NEG;
  //This is important!
  P1OUT=RAM_OE|RAM_WE|RAM_595;
  P1OUT|=IN_LATCH|MEM_LATCH;

  P1DIR=0xFF;
  P2DIR=0xFF;

  TACCR0 = 200;
  TACCTL0 = CCIE;
  TACTL = MC_1|ID_0|TASSEL_1|TACLR;
  __enable_interrupt();

  LCD_Init();

  #pragma MM_ASSIGN_GLOBALS

  #pragma MM_OFFSET 1000
  #pragma MM_DECLARE
    unsigned char result[15];
    unsigned char n1[15];
    unsigned char n2[15];
  #pragma MM_END

  ImmedBCD("0.5",n1);
  ImmedBCD("7",n2);
  LCD_Text("ln(");
  PrintBCD(n1);
  LCD_Text(")=");

  LnBCD(result,n1);
  PrintBCD(result);
  LCD_Byte(0x80+0x40,0);
  LCD_Text("ln(");
  PrintBCD(n2);
  LCD_Text(")=");
  LnBCD(result,n2);
  PrintBCD(result);

  for (;;) {_BIS_SR(LPM3_bits);}
  return 0;
}

//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void Timer_A (void)
__attribute__((interrupt(TIMER0_A0_VECTOR))) static void TA0_ISR(void)
{
  P2OUT&=~LCD_NEG;
  __delay_cycles(CHARGE_CYCLES);
  P2OUT|=LCD_NEG;
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

  LCD_Byte(1,0);
  delay_ms(2);
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

unsigned char RAM_Read(const unsigned char *a1)
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
  P1OUT&=~MEM_LATCH;
  P1OUT|=MEM_LATCH;
  UCB0TXBUF=0;
  while (UCB0STAT & UCBUSY);
  P1OUT|=RAM_OE;
  return UCB0RXBUF;
}

void RAM_Write(const unsigned char *a1, const unsigned char byte)
{
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

void AddBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2)
{
  #pragma MM_VAR result
  #pragma MM_VAR n1
  #pragma MM_VAR n2

  unsigned char carry;
  const unsigned char *temp;
  unsigned char sign;
  int BCD_ptr;
  //where to stop counting if whole parts are different
  int n1_whole=0, n2_whole=0;
  //where to start counting if decimal lengths are different
  int n1_dec=0, n2_dec=0;
  //number to add when carrying. set to 9 for subtraction.
  unsigned char carry_number=0;

  if ((n1[BCD_SIGN]==0)&&(n2[BCD_SIGN]==0)) sign=0;
  else if ((n1[BCD_SIGN]==1)&&(n2[BCD_SIGN]==1)) sign=1;
  else sign=2;

  if ((n1[BCD_SIGN]==1)&&(n2[BCD_SIGN]==0))
  {
    temp=n1;
    n1=n2;
    n2=temp;
  }

  if ((n1[BCD_SIGN]==0)&&(n2[BCD_SIGN]==1))
  {
    buffer[BCD_DEC]=n2[BCD_DEC]+1;
    buffer[BCD_LEN]=n2[BCD_LEN]+1;
    buffer[3]=9;
    carry=1;
    for (BCD_ptr=n2[BCD_LEN]+2;BCD_ptr>=3;BCD_ptr--)
    {
      buffer[BCD_ptr+1]=9-n2[BCD_ptr]+carry;
      if (buffer[BCD_ptr+1]==10) buffer[BCD_ptr+1]=0;
      else carry=0;
    }
    carry_number=9;
    if (carry==1)
    {
      buffer[3]=0;
      carry_number=0;
    }
    n2=buffer;
  }

  //make result whole part equal to the greater of operand whole parts + 1
  if (n1[BCD_DEC]>n2[BCD_DEC])
  {
    n2_whole=n1[BCD_DEC]-n2[BCD_DEC];
    result[BCD_DEC]=n1[BCD_DEC]+1;
  }
  else
  {
    n1_whole=n2[BCD_DEC]-n1[BCD_DEC];
    result[BCD_DEC]=n2[BCD_DEC]+1;
  }

  //make decimal length equal to the greater of operand decimal lengths
  if ((n1[BCD_LEN]-n1[BCD_DEC])>(n2[BCD_LEN]-n2[BCD_DEC]))
  {
    n2_dec=(n1[BCD_LEN]-n1[BCD_DEC])-(n2[BCD_LEN]-n2[BCD_DEC]);
    result[BCD_LEN]=result[BCD_DEC]+n1[BCD_LEN]-n1[BCD_DEC];
  }
  else
  {
    n1_dec=(n2[BCD_LEN]-n2[BCD_DEC])-(n1[BCD_LEN]-n1[BCD_DEC]);
    result[BCD_LEN]=result[BCD_DEC]+n2[BCD_LEN]-n2[BCD_DEC];
  }

  carry=0;
  for (BCD_ptr=result[BCD_LEN]+2;BCD_ptr>=3;BCD_ptr--)
  {
    result[BCD_ptr]=carry;

    if ((BCD_ptr<=result[BCD_LEN]+2-n1_dec)&&(BCD_ptr>n1_whole+3)) result[BCD_ptr]+=n1[BCD_ptr-n1_whole-1];
    if ((BCD_ptr<=result[BCD_LEN]+2-n2_dec)&&(BCD_ptr>n2_whole+3)) result[BCD_ptr]+=n2[BCD_ptr-n2_whole-1];
    if (BCD_ptr<=n2_whole+3) result[BCD_ptr]+=carry_number;
    if (result[BCD_ptr]>9)
    {
      result[BCD_ptr]-=10;
      carry=1;
    }
    else carry=0;
  }

  if ((carry==0)&&(result[3]==9)&&(sign==2))
  {
    carry=1;
    for (BCD_ptr=result[BCD_LEN]+2;BCD_ptr>=3;BCD_ptr--)
    {
      result[BCD_ptr]=9-result[BCD_ptr]+carry;
      if (result[BCD_ptr]==10) result[BCD_ptr]=0;
      else carry=0;
    }
    sign=1;
  }
  else if (sign==2) sign=0;
  result[BCD_SIGN]=sign;

  //could this mess things up?
}

void SubBCD(unsigned char *result, const unsigned char *n1, unsigned char *n2)
{
  #pragma MM_VAR n2
  n2[BCD_SIGN]=!n2[BCD_SIGN];
  AddBCD(result,n1,n2);
  n2[BCD_SIGN]=!n2[BCD_SIGN];
}

void ImmedBCD(unsigned char *text, unsigned char *BCD)
{
  #pragma MM_VAR BCD
  int BCD_ptr=3,text_ptr=0;
  char found=0;

  if (text[0]=='-')
  {
    text++;
    BCD[BCD_SIGN]=1;
  }
  else BCD[BCD_SIGN]=0;

  BCD[BCD_LEN]=0;
  BCD[BCD_DEC]=0;

  while (text[text_ptr])
  {
    if (text[text_ptr]=='.')
    {
      BCD[BCD_DEC]=text_ptr;
      found=1;
    }
    else
    {
      BCD[BCD_ptr]=text[text_ptr]-'0';
      BCD_ptr++;
    }
    text_ptr++;
  }
  BCD[BCD_LEN]=text_ptr-found;
  if (found==0) BCD[BCD_DEC]=BCD[BCD_LEN];
}

void PrintBCD(const unsigned char *BCD)
{
  #pragma MM_VAR BCD
  int BCD_ptr;
  bool zero=true;

  if (BCD[BCD_SIGN]==1) putchar('-');
  for (BCD_ptr=3;BCD_ptr<BCD[BCD_LEN]+3;BCD_ptr++)
  {
    if (BCD_ptr==BCD[BCD_DEC]+3) putchar('.');
    putchar('0'+BCD[BCD_ptr]);
  }
}

void MultBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2)
{
  #pragma MM_VAR result
  #pragma MM_VAR n1
  #pragma MM_VAR n2
  #pragma MM_VAR temp

  #pragma MM_DECLARE
    unsigned char temp[5];
  #pragma MM_END

  int i,j,k,flip=0;
  ImmedBCD("0",result);
  ImmedBCD("0",perm_buff1);

  temp[BCD_SIGN]=0;
  temp[BCD_LEN]=2;

  for (i=0;i<n1[BCD_LEN];i++)
  {
    for (j=0;j<n2[BCD_LEN];j++)
    {
      //The next 3 lines are 212 bytes
      temp[4]=n1[i+3]*n2[j+3];
      //maybe this takes a lot of flash
      temp[3]=temp[4]/10;
      temp[4]=temp[4]%10;
      temp[BCD_DEC]=2+n1[BCD_LEN]-i+n2[BCD_LEN]-j-2;
      if (flip==0)
      {
        AddBCD(result,temp,perm_buff1);
        ShrinkBCD(result);
      }
      else
      {
        AddBCD(perm_buff1,temp,result);
        ShrinkBCD(perm_buff1);
      }
      flip=!flip;
    }
  }

  if (flip==0)
  {
    //Just CopyBCD?
    i=perm_buff1[BCD_LEN];
    for (j=0;j<i+3;j++) result[j]=perm_buff1[j];
  }
  i=(n1[BCD_LEN]-n1[BCD_DEC])+(n2[BCD_LEN]-n2[BCD_DEC]);

  if (i>DEC_PLACES)
  {
    result[BCD_LEN]-=(i-DEC_PLACES-1);
    result[BCD_DEC]=result[BCD_LEN];

    if (result[result[BCD_LEN]+2]>4)
    {
      ImmedBCD("10",temp);
      AddBCD(perm_buff1,result,temp);
      CopyBCD(result,perm_buff1);
    }
    result[BCD_LEN]-=1;
    i=DEC_PLACES+1;
  }
  result[BCD_DEC]-=i;
  result[BCD_SIGN]=n1[BCD_SIGN]^n2[BCD_SIGN];
  FullShrinkBCD(result);
}

void DivBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2)
{
  #pragma MM_VAR result
  #pragma MM_VAR n1
  #pragma MM_VAR n2

  int i,j;
  int result_ptr,n1_ptr;
  int max_offset,dec_offset=0;
  int post_offset=0, pre_offset=0;
  unsigned char remainder=0, res_ptr_off=0;
  bool logic;

  max_offset=n1[BCD_LEN]-n1[BCD_DEC];
  if ((n2[BCD_LEN]-n2[BCD_DEC])>max_offset) max_offset=n2[BCD_LEN]-n2[BCD_DEC];
  if (DEC_PLACES>max_offset) max_offset=DEC_PLACES;

  //result[BCD_SIGN]=0;
  result[BCD_LEN]=0;

  result_ptr=3;

  post_offset=n1[BCD_LEN]-n2[BCD_LEN];

  if ((n1[BCD_DEC]-n2[BCD_DEC]+1)<post_offset)
  {
    pre_offset=-1*(n1[BCD_DEC]-n2[BCD_DEC]+1);
    post_offset=0;
    if (pre_offset<0)
    {
      result[BCD_DEC]=-pre_offset;
    }
    else
    {
      result[BCD_DEC]=0;
      result_ptr+=pre_offset;
      res_ptr_off+=pre_offset;
      for (i=3;i<(pre_offset+3);i++) result[i]=0;
    }
  }
  else if (post_offset>0)
  {
    post_offset=0;
    result[BCD_DEC]=n1[BCD_DEC]-n2[BCD_DEC]+1;
  }
  else
  {
    post_offset*=-1;
    result[BCD_DEC]=post_offset+n1[BCD_DEC]-n2[BCD_DEC]+1;
  }

  perm_buff2[BCD_SIGN]=1;
  perm_buff2[BCD_LEN]=n2[BCD_LEN];
  perm_buff2[BCD_DEC]=n2[BCD_LEN];

  perm_buff1[BCD_SIGN]=0;
  perm_buff1[BCD_LEN]=n2[BCD_LEN]+1;
  perm_buff1[BCD_DEC]=perm_buff1[BCD_LEN];
  perm_buff1[3]=0;

  for (i=3;i<n2[BCD_LEN]+3;i++)
  {
    //<=? was <
    if ((i-3)<post_offset)
    {
      perm_buff1[i+1]=0;
    }
    else if ((i-post_offset)>(n1[BCD_LEN]+2))
    {
      perm_buff1[i+1]=0;
    }
    else
    {
      perm_buff1[i+1]=n1[i-post_offset];
    }
    perm_buff2[i]=n2[i];
  }

  n1_ptr=n2[BCD_LEN]+3+post_offset;
  do
  {
    result[result_ptr]=0;
    result[BCD_LEN]+=1;

    do
    {
      AddBCD(perm_buff3,perm_buff1,perm_buff2);

      if ((perm_buff3[BCD_SIGN]==0)||(IsZero(perm_buff3)))
      {
        result[result_ptr]+=1;
        if (result[result_ptr]==10)
        {
          result[result_ptr]=0;
          for (i=result_ptr-1;i>=3;i--)
          {
            result[i]+=1;
            if (result[i]<10) break;
            else result[i]=0;
          }
          if (i==2)
          {
            result[3]=1;
            for (i=4;i<result_ptr;i++) result[i]=0;
            result_ptr++;
            result[BCD_LEN]+=1;
            result[BCD_DEC]+=1;
            result[result_ptr]=0;
          }
        }
        for (i=3;i<perm_buff1[BCD_LEN]+3;i++) perm_buff1[i]=perm_buff3[i+1];
      }
    } while ((perm_buff3[BCD_SIGN]==0)&&(!IsZero(perm_buff3)));

    for (i=3;i<n2[BCD_LEN]+3;i++)
    {
      perm_buff1[i]=perm_buff1[i+1];
    }

    if ((n1_ptr-3)>=n1[BCD_LEN])
    {
      perm_buff1[i]=0;
    }
    else
    {
      perm_buff1[i]=n1[n1_ptr];
      n1_ptr++;
    }
    result_ptr++;

    //< or <=?
    if ((result_ptr-3)<(result[BCD_DEC]))
    {
      logic=true;
    }
    else if (((result[BCD_LEN]-result[BCD_DEC])<(max_offset+1))/*&&(!IsZero(perm_buff1))*/)
    {
      logic=true;
    }
    else logic=false;
  } while (logic);

  if ((result[BCD_LEN]-result[BCD_DEC])>max_offset)
  {
    if (result[result[BCD_LEN]+2]>4)
    {
      for (i=3;i<result[BCD_LEN]+2;i++) perm_buff3[i]=result[i];
      i=result[BCD_LEN];
      j=result[BCD_DEC];
      perm_buff3[BCD_SIGN]=0;
      perm_buff3[BCD_LEN]=result[BCD_LEN]-1;
      perm_buff3[BCD_DEC]=perm_buff3[BCD_LEN];
      perm_buff1[BCD_SIGN]=0;
      perm_buff1[BCD_LEN]=1;
      perm_buff1[BCD_DEC]=1;
      perm_buff1[3]=1;
      AddBCD(result,perm_buff3,perm_buff1);
      result[BCD_DEC]=j;
      if (result[BCD_LEN]==i) result[BCD_DEC]+=1;
    }
    else
    {
      result[BCD_LEN]-=1;
    }
  }
  result[BCD_SIGN]=n1[BCD_SIGN]^n2[BCD_SIGN];
  FullShrinkBCD(result);
}

void ShrinkBCD(unsigned char *n1)
{
  #pragma MM_VAR n1
  int BCD_ptr;
  if ((n1[3]==0)&&(n1[BCD_DEC]!=0))
  {
    for (BCD_ptr=3;BCD_ptr<n1[BCD_LEN]+2;BCD_ptr++) n1[BCD_ptr]=n1[BCD_ptr+1];
    n1[BCD_LEN]-=1;
    n1[BCD_DEC]-=1;
  }
}

void FullShrinkBCD(unsigned char *n1)
{
  #pragma MM_VAR n1
  int BCD_ptr;
  while ((n1[3]==0)&&(n1[BCD_DEC]>1)) ShrinkBCD(n1);
}


void PadBCD(unsigned char *n1, int amount)
{
  #pragma MM_VAR n1
  int i;
  for (i=n1[BCD_LEN]+3;i>=3;i--) n1[i+amount]=n1[i];
  for (i=3;i<(amount+3);i++) n1[i]=0;
  n1[BCD_LEN]+=amount;
  n1[BCD_DEC]+=amount;
}

bool IsZero(unsigned char *n1)
{
  #pragma MM_VAR n1
  int i;
  for (i=3;i<n1[BCD_LEN]+3;i++) if (n1[i]!=0) return false;
  return true;
}

//see if using this in other places makes things smaller
void CopyBCD(unsigned char *dest, unsigned char *src)
{
  #pragma MM_VAR dest
  #pragma MM_VAR src
  int i;
  for (i=0;i<src[BCD_LEN]+3;i++) dest[i]=src[i];
}

void LnBCD(unsigned char *result, unsigned char *arg)
{
  #pragma MM_VAR result
  #pragma MM_VAR arg
  int i=0,j;
  ImmedBCD("1",p1);
  SubBCD(p0,arg,p1);

  if (IsZero(p0))
  {
    ImmedBCD("0",result);
    return;
  }

  SubBCD(p3,p1,arg);
  AddBCD(p4,p1,arg);
  DivBCD(result,p3,p4);

  ImmedBCD("2",p5);
  CopyBCD(p2,result);
  CopyBCD(p6,result);

  //LCD_Byte('A',1);

  do
  {
    MultBCD(p3,p2,result);
    CopyBCD(p2,p3);
    MultBCD(p3,p2,result);
    CopyBCD(p2,p3);

    AddBCD(p3,p1,p5);
    CopyBCD(p1,p3);
    ShrinkBCD(p1);

    DivBCD(p4,p2,p1);
    AddBCD(p3,p6,p4);
    CopyBCD(p6,p3);
    ShrinkBCD(p6);
  } while (!IsZero(p4));

  //LCD_Byte('B',1);

  MultBCD(result,p6,p5);
  result[BCD_SIGN]=p0[BCD_SIGN];
}
