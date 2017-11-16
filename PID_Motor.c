//-------------------------------------------------------------------------
// DC Motor PID Controller
//
// This program was written to set a DC motor's speed and correct for any resistance from a load.
// Because of system limitations, this code operated between 200 - 1000rpm.
//
// by Adam Woo & Takuto Wada
//
// Status: program successfully tested December 2016
//-------------------------------------------------------------------------

#include <c8051F120.h>
#include <stdio.h>

//-------------------------------------------------------------------------
// Global Constants and variables
//--------------------------------------------------------------------------
#define SYSCLK 1225000
char indicator;
char LCD_init_flag = 0; //LCD initialization flag
char xdata LCD_display[32]; //hold the data to be shown on an LCD
int count;
int LCD_busy_flag (void);
int max_rpm = 1000;
int min_rpm = 250;
int newcount;
long actual_rpm, pulse_count, target_rpm, voltage_reading;
short k, msec_count; //each delay is about 1 msec;
short RTH0, RTL0, RTH1, RTL1; //reload values for Timer_0 and Timer_1
unsigned char LCD_pointer=0;

sbit DB7 = P5^7;
sbit LCD_en = P6^2;
sbit LED = P1^6;
sbit RS = P6^0;
sbit RW = P6^1;
sfr16 ADC0 = 0xBE;
sfr16 RCAP3 = 0xCA;
sfr16 TMR3 = 0xCC;

void ADC_Init(void);
void DISPLAY_ISR(void);
void Display_String(void);
void LCD_Init_ISR(void);
void Oscillator_Init(void);
void PCA_Init(void);
void Port_Init(void);
void Timer3_Init(void);
void Timer_Init(void);
void Timer_ReInit(void);

//--------------------------------------------------------------------------
// program starts here
//--------------------------------------------------------------------------
void main (void){ //calling functions here
  WDTCN = 0xde;
  WDTCN = 0xad;
  LCD_Init_ISR();
  Oscillator_Init();
  Timer3_Init();
  Port_Init();
  Timer_Init();
  PCA_Init();
  ADC_Init();
  DISPLAY_ISR();
  Display_String();
  LED = 0;
  while (1);
}

//--------------------------------------------------------------------------
// Oscillator_Init
//
// This routine initializes the system clock to use the precision internal
// oscillator as its clock source.
//--------------------------------------------------------------------------
void Oscillator_Init (void){
  char SFRPAGE_SAVE = SFRPAGE; // Save Current SFR page
  SFRPAGE = CONFIG_PAGE; // Set SFR page
  OSCICN = 0x82; //use internal oscillator as system clock 12.25 MHz
  CLKSEL = 0x00; // Select the internal oscillator as SYSCLK source
  SFRPAGE = SFRPAGE_SAVE; // Restore SFR page
}

//--------------------------------------------------------------------------
// Port_Init();
//--------------------------------------------------------------------------
void Port_Init(void){
  char SFRPAGE_SAVE = SFRPAGE;
  SFRPAGE = CONFIG_PAGE;
  P1MDOUT = 0x40; //P1.6(LED),
  P6MDOUT = 0x07; //LCD-en (P6.2) RS(P6.0) and RW(P6.1) are set
  P5MDOUT = 0xFF; //DB0-DB7 are set push-pull
  XBR0 = 0x08;
  XBR1  	= 0x06;
  XBR2 = 0x40; // Enable crossbar

  SFRPAGE = SFRPAGE_SAVE;
}

//--------------------------------------------------------------------------
// Initialization of programmable counter array
//--------------------------------------------------------------------------
void PCA_Init(){
  char SFRPAGE_SAVE = SFRPAGE;
  SFRPAGE   = PCA0_PAGE;
  PCA0CN	= 0x40;
  PCA0CPM0  = 0x42;
  PCA0CPH0  = 0xD0;
  SFRPAGE = SFRPAGE_SAVE;
}

//--------------------------------------------------------------------------
// Initialization of analog to digital conversion
//--------------------------------------------------------------------------
void ADC_Init(){
  char SFRPAGE_SAVE = SFRPAGE;
  SFRPAGE   = ADC0_PAGE;
  ADC0CN	= 0x85;
  REF0CN = 0x03;
  AMX0SL = 0x00;
  EIE2  	|= 0x02;
  SFRPAGE = SFRPAGE_SAVE;
}

//--------------------------------------------------------------------------
// Initialization of Timer_0 and Timer_1 as 16 bit
//--------------------------------------------------------------------------
void Timer_Init(void){
  char SFRPAGE_SAVE = SFRPAGE;
  SFRPAGE = TIMER01_PAGE;
  TMOD = 0x11; //Timer_0 and Timer_1 in 16-bit mode
  RTL0 = 0x17; //1000 cycles for about 1 msec
  RTH0 = 0xFC;
  TR0 = 1;
  ET0 = 1;
  RTL1 = 0xEF; //refresh rate of 100Hz (10 msec)
  RTH1 = 0xD8;
  TR1 = 1;
  ET1 = 1;
  EA = 1; //EA and TF0, TF1,and EA are enabled
  SFRPAGE = SFRPAGE_SAVE;
}

//--------------------------------------------------------------------------
// Reinitialization of Timer_0 as a counter
//--------------------------------------------------------------------------
void Timer_ReInit(void){
  char SFRPAGE_SAVE = SFRPAGE;
  SFRPAGE   = TIMER01_PAGE;
  ET0 = 0;
  TMOD = 0x15;
  EA=0;
  TH0 = 0x00;
  TL0 = 0x00;
  EA=1;
  TR0=1;
  SFRPAGE = SFRPAGE_SAVE;
}

//--------------------------------------------------------------------------
// Initialization of Timer_3 as a ADC overflow interrupt
//--------------------------------------------------------------------------
void Timer3_Init(void){
  char SFRPAGE_SAVE = SFRPAGE;
  SFRPAGE = TMR3_PAGE;

  TMR3CN = 0x00;  // Stop timer 3, clear TF3
  TMR3CF = 0x08;  // use SYSCLK as time base
  RCAP3H  = 0xEC;
  RCAP3L = 0x7C;
  TMR3   = RCAP3;
  EIE2 &= ~0x01;  //disable timer 3 Interrupts
  TR3 = 1;
  SFRPAGE = SFRPAGE_SAVE;
}

//--------------------------------------------------------------------------
// LCD_Init_ISR
//
// setup Timer_0 overflow interrupt service routine
// to perform LCD Initialization Function
//--------------------------------------------------------------------------
void LCD_Init_ISR(void) interrupt 1{
  char SFRPAGE_SAVE = SFRPAGE;
  EA = 0;
  TH0 = RTH0;
  TL0 = RTL0;
  EA = 1;
  if (LCD_init_flag == 0 ){
    msec_count++;
    SFRPAGE = CONFIG_PAGE;
    switch (msec_count){
      case 5: case 10: case 11: case 12:
      //function set four times
      LCD_en = 1;
      RS = 0;
      RW = 0;
      P5 = 0x3F;
      // pulse enable
      LCD_en = 0;
      //LED = ~LED;
      break;
      case 13:
      //send display off
      LCD_en = 1;
      RS = 0;
      RW = 0;
      P5 = 0x08;
      // pulse enable
      LCD_en = 0;
      break;
      case 14:
      // send display clear
      LCD_en = 1;
      RS = 0;
      RW = 0;
      P5 = 0x01;
      // pulse enable
      LCD_en = 0;
      break;
      case 16:
      // send entry mode set
      LCD_en = 1;
      RS = 0;
      RW = 0;
      P5 = 0x06;
      // pulse enable
      LCD_en = 0;
      break;
      case 18:
      // send display ON
      LCD_en = 1;
      RS = 0;
      RW = 0;
      P5 = 0x0F;
      // pulse enable
      LCD_en = 0;
      // set the LCD_init_flag
      //LED = 1; //lit the LED to indicate the end of LCD initialization
      LCD_init_flag = 1;
      break;
      default:
      break;
    } //end of switch-case
  }// end of if
  if (LCD_init_flag == 1 ){
    Timer_ReInit();
  }//end of if
  SFRPAGE = SFRPAGE_SAVE;
}


//--------------------------------------------------------------------------
// Display_String()
//
// set up a test display message
//--------------------------------------------------------------------------
void Display_String(void){
  // first line dislay data
  LCD_display[0] = 'T';
  LCD_display[1] = 'A';
  LCD_display[2] = 'R';
  LCD_display[3] = 'G';
  LCD_display[4] = 'E';
  LCD_display[5] = 'T';
  LCD_display[6] = ' ';
  LCD_display[7] = 'R';
  LCD_display[8] = 'P';
  LCD_display[9] = 'M';
  LCD_display[10] = '-';
  LCD_display[11] = ' ';
  //generate the rpm number
  LCD_display[12] = '0'+ target_rpm/1000;
  LCD_display[13] = '0'+ (target_rpm%1000)/100;
  LCD_display[14] = '0'+ ((target_rpm%1000)%100)/10;
  LCD_display[15] = '0'+ ((target_rpm%1000)%100)%10;
  //second line display data
  LCD_display[16] = 'A';
  LCD_display[17] = 'C';
  LCD_display[18] = 'T';
  LCD_display[19] = 'U';
  LCD_display[20] = 'A';
  LCD_display[21] = 'L';
  LCD_display[22] = ' ';
  LCD_display[23] = 'R';
  LCD_display[24] = 'P';
  LCD_display[25] = 'M';
  LCD_display[26] = '-';
  //genearte rpm reading for display
  LCD_display[27] = ' ';
  LCD_display[28] = '0'+ actual_rpm/1000;
  LCD_display[29] = '0'+ (actual_rpm%1000)/100;
  LCD_display[30] = '0'+ ((actual_rpm%1000)%100)/10;
  LCD_display[31] = '0'+ ((actual_rpm%1000)%100)%10;
}

//--------------------------------------------------------------------------
// LCD Display Refresh Function
//--------------------------------------------------------------------------
void DISPLAY_ISR(void) interrupt 3{
  char SFRPAGE_SAVE = SFRPAGE;
  SFRPAGE = CONFIG_PAGE;
  EA = 0;
  TL1 = RTL1; // refresh rate of 50Hz
  TH1 = RTH1; // about 20-msec interval
  EA = 1;

  if (LCD_init_flag == 1) //&& (LCD_busy_flag()==0)){
    switch(LCD_pointer){
      case 16:
      LCD_en = 1;
      RS = 0;
      RW = 0;
      P5 = 0xC0; //set DD RAM address to 40H
      //first char in the 2nd line
      LCD_en = 0;
      for(k=0; k<25; k++); //add delay to settle
      break;
      case 32: //return cursor home
      LCD_en = 1;
      RS = 0;
      RW = 0;
      P5 = 0x02; //move cursor to home position
      LCD_en = 0;
      LCD_pointer = 0;
      for(k=0; k<25; k++); //add delay to settle
      break;
      default:
      break;
    }//end of switch

    //display data on LCD
    LCD_en = 1;
    RS = 1;
    RW = 0;
    P5 = LCD_display[LCD_pointer];
    LCD_en = 0;
    LCD_pointer++;
  }

  SFRPAGE = SFRPAGE_SAVE;
}


//--------------------------------------------------------------------------
// Timer 3 Overflow Interrupt - controls ADC: outputs ADC0H and ADC0L based on 0-5 V reading to be used in target RPM calc
//--------------------------------------------------------------------------
void AD0_ISR(void) interrupt 15{
  if (newcount==500){
    SFRPAGE=ADC0_PAGE;
    AD0INT = 0;
    voltage_reading = ADC0;
    target_rpm = ((voltage_reading*max_rpm))/0xFFF0;
    if (target_rpm < min_rpm){
      target_rpm = min_rpm;
    }
    SFRPAGE = TIMER01_PAGE;

    pulse_count= TH0*0x100+TL0; //first conver TH0 and TL0 into one number (total pulse count)
    actual_rpm = pulse_count*60/80;

    EA=0;
    TH0=0;
    TL0=0;
    EA=1;

    Display_String();

    if (actual_rpm <= max_rpm && actual_rpm >= min_rpm){
      PCA0CPH0 = PCA0CPH0 - (((target_rpm - actual_rpm)/max_rpm)*255);
    }
    else if (actual_rpm <= min_rpm){
      PCA0CPH0--;
    }
    else if (actual_rpm >= max_rpm){
      PCA0CPH0++;
    }

  }
  newcount=newcount+1;
}
