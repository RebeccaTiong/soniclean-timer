/*
 Note: for EMC EMI  reasons I turn on the common driver for 
    the displays and then sequence them  or stagger the indiviual segments,
  and not enable each segment and then turn them all on at once.
   This would cause a sudden rush of current which can cause more interference.



*/






/*******************************************************
This program was created by the CodeWizardAVR V3.27 
Automatic Program Generator
© Copyright 1998-2016 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 14/10/2016
Author  : 
Company : 
Comments: 


Chip type               : ATmega328P
Program type            : Application
AVR Core Clock frequency: 16.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/

#include <io.h>

#include <delay.h>
//#include "routines.inc"


/*

   ucThous  ucHuns     ucTens    ucUnits   ucThous   ucHuns
   1000      100       10        1         1000      100
   |----|    |----|    |----|    |----|    |----|    |----|
   |    |    |    |    |    |    |    |    |    |    |    |
---|    |----|    |----|    |----|    |----|    |----|    |----
  T0T2  T1    T0T2  T1   T0T2  T1  T0T2  T1  T0T2  T1  T0T2  T1
                                                               

 
 

*/



#define ROUNDROUTINE   250


//==========================
//PIN CONFIGURATION
//==========================
                                //Arduino UNO PinOut
#define portSegmentA PORTB.5    //13
#define portSegmentB PORTB.4    //12
#define portSegmentC PORTB.3    //11
#define portSegmentD PORTB.2    //10
#define portSegmentE PORTB.1    //9
#define portSegmentF PORTB.0    //8
#define portSegmentG PORTD.1    //1
#define portSegmentDP PORTD.6   //7

#define portDigit1 PORTD.5      //5
#define portDigit2 PORTD.4      //4
#define portDigit3 PORTD.3      //3
#define portDigit4 PORTD.2      //2

#define PINKeypad   PINC & 0b00111000

#define SHORTPRESS  100
#define LONGPRESS   1000

#define TEST_PIN2   PORTD.7     //6
#define TEST_PIN    PORTD.0     //0



//==========================
//CONSTANT VARIABLES
//==========================
#define Digit_1 1
#define Digit_2 2
#define Digit_3 3
#define Digit_4 4

#define DD_ON   0
#define DD_OFF  1

//fixme change to enum but must correspond with look up table
//enum correspond with look up table fucLookUpSegments
enum enLookUpSegments{
    DISP_0,
    DISP_1,
    DISP_2,
    DISP_3,
    DISP_4,
    DISP_5,
    DISP_6,
    DISP_7,
    DISP_8,
    DISP_9,
    DISP_dp,

    DISP_n,
    DISP_d,
    DISP_t,
    DISP_p,

    DISP_E,
    DISP_C,
    DISP_G,
    DISP_A,

    DISP_H,
    DISP_F,
    DISP_b,
    DISP_U,

    DISP_L,
    DISP_degree,
    DISP_CLEAR,
};

 //fixme change to enum
//enum for messages to be displayed
enum enMessages{
    //message type  //  display     param1      param2
    //------------      -------        ------      ------
    STANDBY,        //  ...
    POWERON,        //  P.On        -           -
    TIME,           //  MM.SS       minutes     second
    TEMP,           //  nn.C        temp        -
    PLEVEL,         //  nnn.P       power       -
    DGAS,           //  dGAS        -           -
    HeaterOn,       //  H.On        -           -
    HeaterOff,      //  H.OFF       -           -
    TCLEAN,         //  nn.ct       minutes     -
    TDGAS,          //  nn.dG       minutes     -
    TDUTYC,         //  nn.dc       minutes     -
    SWVER,          //  Snnn        number      -
    BOFF,           //  b.OFF       -           -
    BON,            //  b.On        -           -
    FSET,           //  F.SEt       -           -
    FON,            //  F.On        -           -
    FOFF,           //  F.OFF       -           -
    USET,           //  U.SEt       -           -
    UON,            //  U.On        -           -
    UOFF,           //  U.OFF       -           -
    LID,            //  Lid         -           -
    LON,            //  L.On        -           -
    LOFF,           //  L.OFF       -           -
    LIDO,           //  Lid.o       -           -
    PAUSE,          //  PAUSE       -           -
    PDC,            //  P.dc        -           -
    DON,            //  d.On        -           -
    DOFF,           //  d.Off       -           -
    CLEAR           //  0.0         -           -  
};

//-----------------------------------------------------------

//==========================
//FLASH                          
//==========================
//look up table for all available display segments
#define SEG_x  0b00000000
#define SEG_a  0b10000000
#define SEG_b  0b01000000
#define SEG_c  0b00100000
#define SEG_d  0b00010000
#define SEG_e  0b00001000
#define SEG_f  0b00000100
#define SEG_g  0b00000010
#define SEG_dp 0b00000001



flash unsigned char fucLookUpSegments[]=
{
    //abcdefg dp
 SEG_a | SEG_b | SEG_c | SEG_d | SEG_e | SEG_f                 , // 0 
         SEG_b | SEG_c                                         , // 1
 SEG_a | SEG_b         | SEG_d | SEG_e         | SEG_g         , // 2
 SEG_a | SEG_b | SEG_c | SEG_d                 | SEG_g         , // 3
         SEG_b | SEG_c                 | SEG_f | SEG_g         , // 4
 SEG_a         | SEG_c | SEG_d         | SEG_f | SEG_g         , // 5
 SEG_a         | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g         , // 6
 SEG_a | SEG_b | SEG_c                                         , // 7
 SEG_a | SEG_b | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g         , // 8
 SEG_a | SEG_b | SEG_c | SEG_d         | SEG_f | SEG_g         , // 9
                                                         SEG_dp, // dp

                 SEG_c         | SEG_e         | SEG_g         , // n
         SEG_b | SEG_c | SEG_d | SEG_e         | SEG_g         , // d
                         SEG_d | SEG_e | SEG_f | SEG_g         , // t
 SEG_a | SEG_b                 | SEG_e | SEG_f | SEG_g         , // P

 SEG_a                 | SEG_d | SEG_e | SEG_f | SEG_g         , // E
 SEG_a                 | SEG_d | SEG_e | SEG_f                 , // C
 SEG_a         | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g         , // G
 SEG_a | SEG_b | SEG_c         | SEG_e | SEG_f | SEG_g         , // A

         SEG_b | SEG_c         | SEG_e | SEG_f | SEG_g         , // H
 SEG_a                         | SEG_e | SEG_f | SEG_g         , // F
                 SEG_c | SEG_d | SEG_e | SEG_f | SEG_g         , // b
         SEG_b | SEG_c | SEG_d | SEG_e         | SEG_g         , // U

                         SEG_d | SEG_e | SEG_f                 , // L
 SEG_a | SEG_b                         | SEG_f | SEG_g         , // degree 
 SEG_x                                                           // CLEAR
};
//----------------------------------------------------
//----------------------------------------------------

//------------------------------------------------------
// P r o t o t y p e   m e t h o d s
//------------------------------------------------------


//void display_message(unsigned int message_type, int param1, int param2);

void InitializeAll(void);

void display_number(unsigned int uiNumber);

void DebounceKeys(void);
void StateMachinePrimary(void);
void TimingGenerator(void);


//------------------------------------------------------
// e n u m z
//------------------------------------------------------

enum STATE_PHASES
     {
     SM_INIT,
     SM_ZERO,
     SM_ADD_ONE,
     SM_MINUS_ONE,
     SM_COUNT_UP,
     SM_COUNT_DOWN, 
     SM_PAUSE,
     SM_END
     
     };

//attempt start
enum KEYPAD_NAME
     {
     KEY0,  //press nothing
     KEY1,
     KEY2,
     KEY3
     };

//attempt end

//------------------------------------------------------
// b i t z
//------------------------------------------------------


bit b10msFlag = 0;
bit b100msFlag = 0;
bit b1secFlag = 0;
bit bTimeT0T2;

bit bFirstFlag;

//attemp start
//Key flag
//bKeyFlag1 bKeyFlag2   Execution
//--------- ---------   ---------
//  0           X       Do nothing
//  1           0       Key long pressed
//  1           1       Key short pressed
bit bKeyFlag1 = 0;     
bit bKeyFlag2 = 0;     
//attempt end

//------------------------------------------------------
// u n s i g n e d   c h a r z
//------------------------------------------------------


char uc1msTimer;
char uc10msTimer;
char uc100msTimer;
char uc1secTimer;
char ucPhase,ucPhasePrev;

char ucThousand,   ucThousandSegments,  ucThousandSegmentsSynced;             
char ucHundred,    ucHundredSegments,   ucHundredSegmentsSynced;              
char ucTen,        ucTenSegments,       ucTenSegmentsSynced;                   
char ucUnit,       ucUnitSegments,      ucUnitSegmentsSynced;            

char ucTimeT0;
char ucTimeT1;
char ucTimeT2;
char ucDutyCycle = 100; //brightness effective between 3 - 249, other number will cause liasing
char ucNextInterrupt;

char ucDigitPtr;
char ucCurrentSegs;

char ucRoutineCounterSync;

//attempt start
unsigned char ucKeyPrevActive;
unsigned char ucKeyCurrActive;

flash unsigned char fucKeyMapLookUp[] = {
    0b00000000,
    0b00100000,
    0b00010000,
    0b00001000
};
//attempt end

//------------------------------------------------------
// u n s i g n e d   i n t z
//------------------------------------------------------

unsigned int uiPhaseTimer;
unsigned int  uiAdcResult;

unsigned int uiNumberDisplay;

unsigned int uiKeyPressCounter;
//----------------------------------------------------

//*********************PRIVATE FUNCTIONS**********************


//-----------------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
// Timer 0 output compare A interrupt service routine
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
interrupt [TIM0_COMPA] void timer0_compa_isr(void)
{
TEST_PIN2 = 1;
  if(bTimeT0T2 == 1)  //TURN ON SEGMENTS  12us at 16Mhz
  {

   TEST_PIN = 1;

   bTimeT0T2 = 0;

   ucTimeT0   = ucTimeT2;    // this is now
   ucTimeT2  += ROUNDROUTINE;   // next time T2
   ucTimeT1   = ucTimeT0 + ucDutyCycle;   // next T1
   ucNextInterrupt = ucTimeT1;
  //sophiepeppamolly 
 
// Sync the update to the thousands display update time  
  
   if(ucDigitPtr == 0)
                    {
                    ucThousandSegmentsSynced = ucThousandSegments;
                    ucHundredSegmentsSynced  = ucHundredSegments;
                    ucTenSegmentsSynced      = ucTenSegments;
                    ucUnitSegmentsSynced     = ucUnitSegments;
                    
                      
                    ucCurrentSegs = ucThousandSegmentsSynced;
                    portDigit1 = DD_ON;
                    }
   if(ucDigitPtr == 1){ucCurrentSegs = ucHundredSegmentsSynced; portDigit2 = DD_ON;}
   if(ucDigitPtr == 2){ucCurrentSegs = ucTenSegmentsSynced;     portDigit3 = DD_ON;}
   if(ucDigitPtr == 3){ucCurrentSegs = ucUnitSegmentsSynced;    portDigit4 = DD_ON;}
   
   
   
    ucDigitPtr++;
    if(ucDigitPtr > 3){ucDigitPtr = 0;}
                          //molly
    portSegmentA = ucCurrentSegs  & 0b10000000;// SegmentA;    
    portSegmentB = ucCurrentSegs  & 0b01000000;// SegmentB;    
    portSegmentC = ucCurrentSegs  & 0b00100000;// SegmentC;    
    portSegmentD = ucCurrentSegs  & 0b00010000;// SegmentD;    
    portSegmentE = ucCurrentSegs  & 0b00001000;// SegmentE;    
    portSegmentF = ucCurrentSegs  & 0b00000100;// SegmentF;    
    portSegmentG = ucCurrentSegs  & 0b00000010;// SegmentG;    
    portSegmentDP = ucCurrentSegs & 0b00000001;// SegmentDP;        
    
  }
  else // it is Time T1  TURN OFF SEGMENT   3 us at 16MHz
  {       
   TEST_PIN = 0;

   bTimeT0T2 = 1;
  ucNextInterrupt = ucTimeT2;
 

 portSegmentA  = 0;  //13
 portSegmentB  = 0;  //12
 portSegmentC  = 0;  //11
 portSegmentD  = 0;  //10
 portSegmentE  = 0;  //9
 portSegmentF  = 0;  //8
 portSegmentG  = 0;  //1
 portSegmentDP = 0;  //6

  portDigit1 = DD_OFF;
  portDigit2 = DD_OFF;
  portDigit3 = DD_OFF;
  portDigit4 = DD_OFF;


  }
  
    OCR0A = ucNextInterrupt;
    ucRoutineCounterSync++;
TEST_PIN2 = 0;

}
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------

// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR))

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCW;
}

//--------------------------------------------------------
//--------------------------------------------------------
//--------------------------------------------------------
//--------------------------------------------------------
//--------------------------------------------------------
void main(void)
{
// Declare your local variables here

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif


InitializeAll();


// Globally enable interrupts
#asm("sei")

while (1)
      {
      
      while(ucRoutineCounterSync == 0){}  ucRoutineCounterSync--;

      TimingGenerator();
       
      uiAdcResult = read_adc(0);

      
      DebounceKeys();
     
      StateMachinePrimary();
      
      }
}

//----------------------------------------------------
//----------------------------------------------------
//----------------------------------------------------
//----------------------------------------------------

//-----------------------------------------------------------

     //attempt start
//How it works
//1 Scan the key that is pressed
//2 If the key pressed duration pass the 'short press', activate a flag to indicate the short press is activated
//3 If detecting the exiting of the key pressing process, activate a flag to enable execution
//4 If the key pressed duration pass the 'long press', deactivate a flag to indicate the short press is activaed, activate a flag to enable execution
//Further information
//In this trial, use three keys, UP(KEY1), STOP(KEY2), and DOWN(KEY3) - see state machine to see how it respond.
void DebounceKeys(void)
{
    unsigned char uckeysize = sizeof(fucKeyMapLookUp);
    unsigned char i = 0;
    unsigned char ucKeyPressed = ~PINKeypad;
    ucKeyCurrActive = 0;
    
    
    for (i = 1; i < uckeysize; i++){
        if((fucKeyMapLookUp[i] & ucKeyPressed) == fucKeyMapLookUp[i]){
            ucKeyCurrActive = i; 
            break;
        } 
    }    
        
    if(ucKeyCurrActive == KEY0){
        if(ucKeyPrevActive != 0){ //exit from a key press, reset flags and counters
            bKeyFlag1 = 1;
        }
    }else if(ucKeyCurrActive == ucKeyPrevActive){ //no change in the keypad state
        uiKeyPressCounter ++;
    }else{
        ucKeyPrevActive = ucKeyCurrActive;  //enter into a key press
    }
    
    //set flags
    if(uiKeyPressCounter >= LONGPRESS){
        bKeyFlag1 = 1;
        bKeyFlag2 = 0;    
    }else if(uiKeyPressCounter >= SHORTPRESS){
        //bKeyFlag1 = 1;
        bKeyFlag2 = 1;
    }
    
    //response according to the flags
    if(bKeyFlag1){
        switch(ucKeyPrevActive){
            case KEY1:
                if(bKeyFlag2){
                    ucPhase = SM_ADD_ONE;
                }else{
                    ucPhase = SM_COUNT_UP;
                }
            break;
            case KEY2:
                if(bKeyFlag2){
                    ucPhase = SM_PAUSE;
                }else{
                    ucPhase = SM_ZERO;
                }
            break;
            case KEY3:
                if(bKeyFlag2){
                    ucPhase = SM_MINUS_ONE;
                }else{
                    ucPhase = SM_COUNT_DOWN;
                }
            break;
        } 
        bKeyFlag1 = 0;
        bKeyFlag2 = 0;
        uiKeyPressCounter = 0;
        ucKeyPrevActive = 0;            
    }else{
    //    display_number(0);
    }
}
//attempt end

//attempt start
//How it works
//1 If UP key is pressed once, increase the number display by 1 (maximum number 9999)
//2 If UP key is hold, keep increase the number until is reaches 9999, or pause button pressed
//3 If DOWN key is pressed, decrease the number display by 1 (minimum number 0)
//4 If DOWN key is hold, keep decreasing the number until is reaches 0, or pause
//5 If PAUSE key is pressed, pause the number increasing / decreasing process.
//6 If PAUSE key is hold, zero the number display.
void StateMachinePrimary(void)
{
   uiPhaseTimer++;
   bFirstFlag = 0;
  if(ucPhasePrev != ucPhase)
    {
     ucPhasePrev = ucPhase; 
     bFirstFlag = 1;
     uiPhaseTimer = 0;
     }

  switch(ucPhase)
  {
    case SM_INIT: 
        ucPhase = SM_ZERO;
    break;                            
  
  //------------
    case SM_ZERO:
        uiNumberDisplay = 0;
        display_number(uiNumberDisplay);
        ucPhase = SM_PAUSE;
    break;
  
  //------------
    case SM_PAUSE:
        //do nothing
    break;
  //------------
    case SM_ADD_ONE:
        if(uiNumberDisplay < 9999)
        {
            uiNumberDisplay++;
        }
        display_number(uiNumberDisplay);
        ucPhase = SM_PAUSE;
    break;    
  //------------
    case SM_MINUS_ONE:
        if(uiNumberDisplay > 0)
        {
            uiNumberDisplay--;
        }
        display_number(uiNumberDisplay);
        ucPhase = SM_PAUSE;
    break;    
  //------------
    case SM_COUNT_UP:
        if(uiNumberDisplay < 9999)
        {
            uiNumberDisplay++;
        }
        display_number(uiNumberDisplay);
        ucPhase = SM_COUNT_UP;
    break;    
  //------------
    case SM_COUNT_DOWN:
        if(uiNumberDisplay > 0)
        {
            uiNumberDisplay--;
        }
        display_number(uiNumberDisplay);
        ucPhase = SM_COUNT_DOWN;
    break;    
  //------------
    default:
        ucPhase = SM_INIT;
    break;
  }

}//attempt end

////--------original-------------------------------------------
//void StateMachinePrimary(void)
//{
//   uiPhaseTimer++;
//   bFirstFlag = 0;
//  if(ucPhasePrev != ucPhase)
//    {
//     ucPhasePrev = ucPhase; 
//     bFirstFlag = 1;
//     uiPhaseTimer = 0;
//     }
//
//  switch(ucPhase)
//  {
//  case SM_INIT:
//     ucPhase = SM_START;
//  break;
//  //------------
//  case SM_START:
//  if(bFirstFlag){uiPhaseTimer = 0;}
//
//  break;
//  //------------
//  case SM_RUN:
//  if(bFirstFlag){}
//  
//
//  break;
//  //------------
//  case SM_WHATEVER:
//  if(bFirstFlag){}
//  
//
//  break;
//  //------------
//  
//  default:
//   ucPhase = SM_INIT;
//  break;
//  }
//
//}
//-----------------------------------------------------------


// fixme  check if > should be >+

void TimingGenerator(void)
{

b10msFlag  = 0;
b100msFlag = 0;
b1secFlag  = 0;

uc1msTimer++;
if(uc1msTimer >= 10)
  {
   uc1msTimer  = 0;

   b10msFlag   = 1;

   uc10msTimer++;
   if(uc10msTimer >= 10)
   {
    b100msFlag = 1;
    uc100msTimer++;
    if(uc100msTimer >= 10)
    {              
     uc100msTimer = 0;
     b1secFlag = 1;
     uc1secTimer++;
    }
   }
  }



};

//-----------------------------------------------------------

//-------------------------------------------
// Only call this when needed - not every loop
//---------------------------------------------
void display_number(unsigned int uiNumber)
{    
          
   ucThousand = (char)(uiNumber / 1000);    uiNumber %= 1000;
   ucHundred  = (char)(uiNumber / 100);     uiNumber %= 100;
   ucTen      = (char)(uiNumber / 10);      uiNumber %= 10;
   ucUnit     = (char)uiNumber;
 #asm("cli")

//  ucThousand = 1;
//  ucHundred  = 2;
//  ucTen = 3;
//  ucUnit = 4;

  ucThousandSegments = fucLookUpSegments[ucThousand];
  ucHundredSegments  = fucLookUpSegments[ucHundred];
  ucTenSegments      = fucLookUpSegments[ucTen];
  ucUnitSegments     = fucLookUpSegments[ucUnit];

 #asm("sei")
}
//----------------------------------------------------------





void InitializeAll(void)
{

// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRB=(1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit6=P Bit5=P Bit4=P Bit3=P Bit2=T Bit1=T Bit0=T 
PORTC=(1<<PORTC6) | (1<<PORTC5) | (1<<PORTC4) | (1<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (1<<PORTD5) | (1<<PORTD4) | (1<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 250.000 kHz
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
// Timer Period: 1.024 ms
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2A output: Disconnected
// OC2B output: Disconnected
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: Off
EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EIMSK=(0<<INT1) | (0<<INT0);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

// USART initialization
// USART disabled
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
// Digital input buffer on AIN0: On
// Digital input buffer on AIN1: On
DIDR1=(0<<AIN0D) | (0<<AIN1D);

// ADC initialization
// ADC Clock frequency: 1000.000 kHz
// ADC Voltage Reference: AREF pin
// ADC Auto Trigger Source: ADC Stopped
// Digital input buffers on ADC0: Off, ADC1: Off, ADC2: On, ADC3: On
// ADC4: On, ADC5: On
DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

}
//------------------------------------------------------
