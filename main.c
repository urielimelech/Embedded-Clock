/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM� FS USB Demo Board, 
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/


//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include <math.h>

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"

#include "OledGraphics.h"

//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
     #pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
   //PLL Prescaler Selection bits:
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
   //Stack Overflow/Underflow Reset Enable bit:
     #pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
   //Extended Instruction Set Enable bit:
     #pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
   //CPU System Clock Postscaler:
     #pragma config CPUDIV = OSC1        //No CPU system clock divide
   //Code Protection bit:
     #pragma config CP0 = OFF            //Program memory is not code-protected
   //Oscillator Selection bits:
     #pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
   //Secondary Clock Source T1OSCEN Enforcement:
     #pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
   //Low-Power Timer1 Oscillator Enable bit:
     #pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
   //Fail-Safe Clock Monitor Enable bit:
     #pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
   //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
     #pragma config IESO = OFF           //Two-Speed Start-up disabled
   //Watchdog Timer Postscaler Select bits:
     #pragma config WDTPS = 32768        //1:32768
   //DSWDT Reference Clock Select bit:
     #pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
   //RTCC Reference Clock Select bit:
     #pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
   //Deep Sleep BOR Enable bit:
     #pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
   //Deep Sleep Watchdog Timer Enable bit:
     #pragma config DSWDTEN = OFF        //Disabled
   //Deep Sleep Watchdog Timer Postscale Select bits:
     #pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
   //IOLOCK One-Way Set Enable bit:
     #pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
   //MSSP address mask:
     #pragma config MSSP7B_EN = MSK7     //7 Bit address masking
   //Write Protect Program Flash Pages:
     #pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
   //Write Protection End Page (valid when WPDIS = 0):
     #pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
   //Write/Erase Protect Last Page In User Flash bit:
     #pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
   //Write Protect Disable bit:
     #pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored
  
#else
    #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();

static char digits[9] = {'2','3',':','5', '0', ':', '0', '0', '\0'};
static char Date[6] = {'0', '1', '/', '0', '1', '\0'};
static BYTE MonthValue, DayValue;
static BYTE clock = 0, HourValue;
static BOOL changeClockFlag = 0;
static BOOL AM_PM = 0;
static char APM[3] = {'A', 'M', '\0'};

struct changeFlag{
  BYTE sec, min, hour;
};

static struct changeFlag changeFlags = {2, 2, 2};
static struct changeFlag AlarmFlag = {0, 0, 0};

BOOL CheckButtonPressed(void);
void DateUpdate();
void Alarm();

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
  //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
  //the reset, high priority interrupt, and low priority interrupt
  //vectors.  However, the current Microchip USB bootloader 
  //examples are intended to occupy addresses 0x00-0x7FF or
  //0x00-0xFFF depending on which bootloader is used.  Therefore,
  //the bootloader code remaps these vectors to new locations
  //as indicated below.  This remapping is only necessary if you
  //wish to program the hex file generated from this project with
  //the USB bootloader.  If no bootloader is used, edit the
  //usb_config.h file and comment out the following defines:
  //#define PROGRAMMABLE_WITH_SD_BOOTLOADER
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
  #else 
    #define REMAPPED_RESET_VECTOR_ADDRESS     0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
  #endif
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  extern void _startup (void);        // See c018i.c in your C18 compiler dir
  #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
  void _reset (void)
  {
      _asm goto _startup _endasm
  }
  #endif
  #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
  void Remapped_High_ISR (void)
  {
       _asm goto YourHighPriorityISRCode _endasm
  }
  #pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
  void Remapped_Low_ISR (void)
  {
       _asm goto YourLowPriorityISRCode _endasm
  }
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  //Note: If this project is built while one of the bootloaders has
  //been defined, but then the output hex file is not programmed with
  //the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
  //As a result, if an actual interrupt was enabled and occured, the PC would jump
  //to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
  //executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
  //(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
  //would effective reset the application.
  
  //To fix this situation, we should always deliberately place a 
  //"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
  //"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
  //hex file of this project is programmed with the bootloader, these sections do not
  //get bootloaded (as they overlap the bootloader space).  If the output hex file is not
  //programmed using the bootloader, then the below goto instructions do get programmed,
  //and the hex file still works like normal.  The below section is only required to fix this
  //scenario.
  #pragma code HIGH_INTERRUPT_VECTOR = 0x08
  void High_ISR (void)
  {
       _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #pragma code LOW_INTERRUPT_VECTOR = 0x18
  void Low_ISR (void)
  {
       _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

  #pragma code
  
//	========================	Application Interrupt Service Routines	========================
  //These are your actual interrupt handling routines.
  #pragma interrupt YourHighPriorityISRCode
  void YourHighPriorityISRCode()
  {
    BYTE temp;
    clock++;
    changeFlags.sec = 1;
    if (clock == 10){
      digits[7] = clock + 0x30;
      if (digits[7] > '9'){
        digits[7] = '0';
        digits[6]++;
        if (digits[6] >= '6'){
          digits[6] = '0';
          digits[4]++;
          changeFlags.min = 1;
          changeFlags.hour = 1;
          if (digits[4] > '9'){
            digits[4] = '0';
            digits[3]++;
            if (digits[3] >= '6'){
              digits[3] = '0';
              digits[1]++;
              HourValue++;
              if (digits[1] > '9'){
                digits[1] = '0';
                digits[0]++;
              }
            }
          }
        } 
      }
      clock = 0;
    }
    digits[7] = clock + 0x30;
    if (HourValue == 24){
      HourValue = 0;
      digits[0] = '0';
      digits[1] = '0';
      DayValue++;
    }
    if (HourValue >= 12){
      APM[0] = 'P';
    }
    else {
      APM[0] = 'A';
    }
    if (DayValue > 30){
      MonthValue++;
      DayValue = 1;
    }
    if (MonthValue > 12){
      MonthValue = 1;
    }
    if (MonthValue == 2){
      if (DayValue > 28){
        MonthValue++;
        DayValue = 1;
      }
    }
    DateUpdate();
    changeClockFlag = 1;

    if (AlarmFlag.sec >= 2){
      if (AlarmFlag.sec > 21){
        AlarmFlag.sec = 0;
      }
      else
        AlarmFlag.sec++;
    }
    if (AlarmFlag.sec == 1){
      temp = (digits[0] - 0x30) * 10 + digits[1] - 0x30;
      if (AlarmFlag.hour == temp){
        temp = (digits[3] - 0x30) * 10 + digits[4] - 0x30;
        if (AlarmFlag.min == temp){
          AlarmFlag.sec = 2;
        }
      }
    }
    TMR0H = 0x48;
    TMR0L = 0xe5;
    INTCONbits.TMR0IF = 0;
  }

    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  //This return will be a "retfie fast", since this is in a #pragma interrupt section 
  #pragma interruptlow YourLowPriorityISRCode
  void YourLowPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif




//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
  /* Initialize the mTouch library */
  mTouchInit();

  /* Call the mTouch callibration function */
  mTouchCalibrate();

  /* Initialize the accelerometer */
  InitBma150(); 

  /* Initialize the oLED Display */
   ResetDevice();  
   FillDisplay(0x00);
   //oledPutROMString((ROM_STRING)" PIC18F Starter Kit  ",0,0);
}//end UserInit


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
	// Soft Start the APP_VDD
   while(!AppPowerReady())
		;

    #if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
  //On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
  //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
  //This allows the device to power up at a lower initial operating frequency, which can be
  //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
  //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
  //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while(pll_startup_counter--);
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.
    #endif

    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
    
    UserInit();

}//end InitializeSystem

static void InitializeADCON0(void)
{
    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ADCON0 = 0x11;                  // Default all pins to digital
    #endif
}

int readTouch(BYTE channel){
	BYTE x;
	int a2d;
	x = 2;
	ADCON0 = channel;
	while (x){						//check the status of ADCON0 if get any input, else exit the loop
		x = x & ADCON0;
	}
	a2d = ADRESH;
	a2d = a2d << 8;
	a2d = a2d + ADRESL;	
	return a2d;
}

char CheckButtonPressed()
{
  static char buttonPressed = FALSE;
  static unsigned long buttonPressCounter = 0;
  unsigned char temp;
  unsigned long touch1;

  touch1 = 700000;

  while(PORTBbits.RB0 == 0)
  {
    buttonPressCounter++;
  }
  if (buttonPressCounter > 200){
    if (buttonPressCounter < touch1){
      buttonPressCounter = 0;
      buttonPressed = 2;
    }
    else if (buttonPressCounter > touch1){
      buttonPressCounter = 0;
      buttonPressed = 1;
      if (AlarmFlag.sec >= 2){
        AlarmFlag.sec = 0;
      }
    }
  }
  if(buttonPressed != 0)
  {
    if(buttonPressCounter == 0)
    {
      temp = buttonPressed;
      buttonPressed = 0;
      return temp;
    }
    else
    {
      buttonPressCounter--;
    }
  }
  return 0;
}

int readAcc(BYTE address){
	BYTE b;
	int result;
	b = BMA150_ReadByte(address);
	result = (int)b;
	result = result << 8;
	b = BMA150_ReadByte(address - 0x01);
	result += b;
	result = result >> 6;
	if (address == 0x07)
		{return result;}
	result = calcAcc(result);
	return result;
}

int calcAcc(int result){
	if(result > 511){
		result = result | 0xFC00;
		result = twosComp(result);
	}
	return result;
}

int twosComp(int value){
	value = ~value;
	value += 0x1;
	return -value;
}

//	========================	Application Code	========================


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/

// void checkedLine(char* unCheck, char* Check){
//   unCheck[0] = ' ';
//   Check[0] = '>';
// }

BOOL flag = TRUE, BP = TRUE;
int pressed(int selection){
  if (flag){
    flag = FALSE;
    BP = TRUE;
  }
  else if(!flag & !BP){
    flag = TRUE;
  }
  BP = FALSE;
  return selection;
}

#define	PI 3.141592654

static BYTE prevXSec = 0, prevYSec = 0, prevXMin = 0, prevYMin = 0, prevXHour = 0, prevYHour = 0;

void oledAnalogClock(char* time){
  BYTE centerX = 67, centerY = 32;
  float forwardX, forwardY;
  int sec, min, h;
  BYTE xSec, ySec, xMin, yMin, xHours, yHours;

  sec = ((time[6] - 0x30) * 10) + (time[7] - 0x30);
  min = ((time[3] - 0x30) * 10) + (time[4] - 0x30);
  h   = ((time[0] - 0x30) * 10) + (time[1] - 0x30);

  forwardX = sin(sec * 6 / 180.0 * PI);
  forwardY = cos(sec * 6 / 180.0 * PI);
  xSec = centerX + (27 * forwardX);
  ySec = centerY - (27 * forwardY);
  forwardX = sin(min * 6 / 180.0 * PI);
  forwardY = cos(min * 6 / 180.0 * PI);
  xMin = centerX + (23 * forwardX);
  yMin = centerY - (23 * forwardY);
  forwardX = sin((h * 30 + (min / 12) * 6) / 180.0 * PI);
  forwardY = cos((h * 30 + (min / 12) * 6) / 180.0 * PI);
  xHours = centerX + (19 * forwardX);
  yHours = centerY - (19 * forwardY);

  if (changeFlags.sec == 1){
    if (prevXSec == 0 || prevYSec == 0){
      drawLine(centerX, centerY, xSec, ySec, thin);
    }
    else{
      drawLine(centerX, centerY, prevXSec, prevYSec, thin);
      drawLine(centerX, centerY, xSec, ySec, thin);
    }
    prevXSec = xSec;
    prevYSec = ySec;
    changeFlags.sec = 0;
  }
  else if (changeFlags.sec == 2){
    drawLine(centerX, centerY, xSec, ySec, thin);
    prevXSec = xSec;
    prevYSec = ySec;
    changeFlags.sec = 0;
  }
  if (changeFlags.min == 1){
    if (prevXMin == 0 || prevYMin == 0){
      prevXMin = xMin;
      prevYMin = yMin;
      drawLine(centerX, centerY, xMin, yMin, thick);
    }
    else{
      drawLine(centerX, centerY, prevXMin, prevYMin, thick);
      drawLine(centerX, centerY, xMin, yMin, thick);
    }
    prevXMin = xMin;
    prevYMin = yMin;
    changeFlags.min = 0;
  }
  else if (changeFlags.min == 2){
      prevXMin = xMin;
      prevYMin = yMin;
      drawLine(centerX, centerY, xMin, yMin, thick);
      changeFlags.min = 0;
  }
  if (changeFlags.hour == 1){
    if (prevXHour == 0 || prevYHour == 0){
      drawLine(centerX, centerY, xHours, yHours, fat);
    }
    else{
      drawLine(centerX, centerY, prevXHour, prevYHour, fat);
      drawLine(centerX, centerY, xHours, yHours, fat);
    }
    prevXHour = xHours;
    prevYHour = yHours;
    changeFlags.hour = 0;
  }
  else if (changeFlags.hour == 2){
    prevXHour = xHours;
    prevYHour = yHours;
    drawLine(centerX, centerY, xHours, yHours, fat);
    changeFlags.hour = 0;
  }
}

void Alarm(void){
  FillDisplay(0xFF);
  DelayMs(50);
  FillDisplay(0x00);
  DelayMs(50);
}

void resetAnalog(void){
  prevXSec = 0;
  prevYSec = 0;
  prevXMin = 0;
  prevYMin = 0;
  prevXHour = 0;
  prevYHour = 0;
  changeFlags.sec = 1;
  changeFlags.min = 1;
  changeFlags.hour = 1;
}

void changeToAPM(void){
  if (HourValue > 12){
    digits[1] = (HourValue % 12) % 10 + 0x30;
    digits[0] = (HourValue % 12) / 10 + 0x30;
  }
}

void changeTo24H(void){
  digits[1] = HourValue % 10 + 0x30;
  digits[0] = HourValue / 10 + 0x30;
}

void DateUpdate(void){
  Date[1] = DayValue % 10 + 0x30;
  Date[0] = DayValue / 10 + 0x30;
  Date[4] = MonthValue % 10 + 0x30;
  Date[3] = MonthValue / 10 + 0x30;
}

void setTime(char* tempTime){
  BYTE i;
  for (i = 0; i < 9; i++){
    digits[i] = tempTime[i];
  }
  HourValue = (digits[0] - 0x30) * 10 + digits[1] - 0x30;
}

void setTempTime(BYTE H, BYTE M, BYTE S, char* tempTime){
  tempTime[0] = (H / 10) + 0x30;
  tempTime[1] = (H % 10) + 0x30;
  tempTime[3] = (M / 10) + 0x30;
  tempTime[4] = (M % 10) + 0x30;
  tempTime[6] = (S / 10) + 0x30;
  tempTime[7] = (S % 10) + 0x30;
}

void selection_Menu(void){                    //pot menu
  char option1[13] = {'D', 'i', 's', 'p', 'l', 'a', 'y', ' ', 'M', 'o', 'd', 'e', '\0'};
  char option2[19] = {'1', '2', 'H', ' ', '/', ' ', '2', '4', 'H', ' ', 'I', 'n', 't', 'e', 'r', 'v', 'a', 'l', '\0'};
  char option3[9] = {'S', 'e', 't', ' ', 'T', 'i', 'm', 'e', '\0'};
  char option4[9] = {'S', 'e', 't', ' ', 'D', 'a', 't', 'e', '\0'};
  char option5[6] = {'A', 'l', 'a', 'r', 'm', '\0'};
  char option6[8] = {'G', 'o', ' ', 'B', 'a', 'c', 'k', '\0'};

  oledPutString (digits, 0, 13*6);
  oledPutString (Date, 0, 0*6);
  oledPutString (option1, 2, 1*6);
  oledPutString (option2, 3, 1*6);
  oledPutString (option3, 4, 1*6);
  oledPutString (option4, 5, 1*6);
  oledPutString (option5, 6, 1*6);
  oledPutString (option6, 7, 7*6);
}

void mainMenu(void){                   // Main Menu
  oledPutString(Date, 0,0*6);
}

void setDate_Menu(void){
  oledPutBigChar(Date, 3, 4*6);
}

void analog_Menu (void){
  BYTE i;
  float x, y, temp;
  BYTE x1, x2, y1, y2;

  temp = PI / 6;
  oledPutString(Date, 0, 0*6);

  for (i = 0; i < 12; i++){
    x = sin(i * temp);
    y = cos(i * temp);
    x1 = 67 + (28 * x);
    x2 = 67 + (32 * x);
    y1 = 32 - (28 * y);
    y2 = 32 - (32 * y);
    drawLine (x1, y1, x2, y2, thin);
  }
}

void num2str(char* str, BYTE num){  
  str[0] = (num / 10) + 0x30;
  str[1] = (num % 10) + 0x30;
  str[2] = '\0';
}

void menuPrinter(int menuNum){
  FillDisplay(0x00);
  switch (menuNum){
    case 0:
      if (AlarmFlag.sec == 1){
        oledWriteAlarmIcon(0xB0, 16*6);
      }
      mainMenu();
      break;
    case 1:
      if (AlarmFlag.sec == 1){
        oledWriteAlarmIcon(0xB2, 19*6);
      }
      selection_Menu();
      break;
    case 2:
      if (AlarmFlag.sec == 1){
        oledWriteAlarmIcon(0xB0, 16*6);
      }
      analog_Menu();
      break;
    case 6:
      setDate_Menu();
      break;
  }
}


void main(void)
{
  BYTE selection_flag = 0;
  BYTE H, M, S;
  BYTE i;
  char time[9] = {'0', '0', ':', '0', '0', ':', '0', '0', '\0'};
  char str[3];
  unsigned int resultY;
  int potLastState = 0;
  BYTE functionallity = 0;
	int potValue, L, R, U, D;
  BYTE clickFunc, prevSelection, dateFlag = 0, selectMenu = 0, selection = 0;

 	InitializeADCON0();
	InitializeSystem();
  mTouchInit();
  mTouchCalibrate();

  T0CONbits.T08BIT = 0 ;			//Timer0 16BIT COUNTER
	T0CONbits.T0CS = 0 ;			//Clock Source -- Internal
	T0CONbits.PSA = 0 ;				//Use Pre-Scaler
	T0CONbits.T0PS = 7 ;			//Prescale 1:4
	T0CONbits.TMR0ON = 1 ;			//Set Timer to ON

	RCONbits.IPEN = 1 ;				//Use Priority Interrutps
	INTCON2bits.T0IP = 1 ;			//Timer0 High-Priority

	INTCONbits.GIE = 1 ;			//Enable Interrupts
	INTCONbits.PEIE = 1 ;
	INTCONbits.T0IE = 1 ;			//Timer0 Overflow Interrupt Enabled

  mainMenu();
  HourValue = ((digits[0] - 0x30) * 10 + digits[1] - 0x30);
  MonthValue = (Date[0] - 0x30) * 10 + Date[1] - 0x30;
  DayValue = (Date[3] - 0x30) * 10 + Date[4] - 0x30;
/*******************\
 */ 
  while(1){                                    //Main is Usualy an Endless Loop
    if (AlarmFlag.sec >= 2){
      Alarm();
      resetAnalog();
      menuPrinter(selectMenu);
    }
    clickFunc = CheckButtonPressed();          // 1 - long press, 2 - short press
    selectMenu = pressed(selection);
    ADCON0 = 0x13;
    switch (selectMenu){
      case 0:                                  // main menu
        if (selection_flag != 0){              // selection = 0
          menuPrinter(selectMenu);             // functionallity = 0
          functionallity = 0;
        }
        selection_flag = selection;
        oledPutString(Date, 0,0*6);
        if (changeClockFlag){
          if (AM_PM == 0){
            oledPutBigChar(digits, 4, 0);
          }
          else {
            changeToAPM();
            oledPutString(APM, 0, 19*6);
            oledPutBigChar(digits, 4, 0);
          }
          changeClockFlag = 0;
        }
        if (clickFunc == 1){
          prevSelection = selection;
          selection = 1;
        }
        break;
      case 1:
        if (selection_flag != 1){               // selection menu
          menuPrinter(selectMenu);              // selection = 1
          functionallity = 1;                   // functionallity = 1
        }
        selection_flag = selection;
        oledPutString(Date, 0,0*6);
        if (changeClockFlag){
          oledPutString(digits, 0, 13*6);
          changeClockFlag = 0;
        }
        if (changeClockFlag){
          changeClockFlag = 0;
        }
        break;
      case 2:                                   // analog mode selection = 2
        if (selection_flag != 2){               // functionallity = 0
          menuPrinter(selectMenu);
          functionallity = 0;
          AM_PM = 0;
        }
        selection_flag = selection;
        oledPutString(Date, 0,0*6);
        if (changeClockFlag){
          oledPutString(APM, 0, 19*6);
          oledAnalogClock(digits);
          changeClockFlag = 0;
        }
        if (clickFunc == 1){
          prevSelection = selection;
          selection = 1;
        }
        break;
      case 3:                                   // display mode selection = 3
        if (prevSelection == 0){
          selection = 2;
          menuPrinter(selectMenu);
          resetAnalog();
        }
        else{
          selection = 0;
          menuPrinter(selectMenu);
        }
        break;
      case 4:                                   // 12H/24H selection = 4
        if (AM_PM == 0){
          AM_PM = 1;
        }
        else {
          AM_PM = 0;
          changeTo24H();
        }
        selection = 0;
        break;
      case 5:                                   // set time selection = 5
        if (selection_flag != 5){               // set date menu
          menuPrinter(selectMenu);              // selection = 6
          functionallity = 5;                   // functionallity = 6
        }
        selection_flag = selection;
        break;
      case 6:
        if (selection_flag != 6){               // set date menu
          menuPrinter(selectMenu);              // selection = 6
          functionallity = 6;                   // functionallity = 6
          dateFlag = 0;
        }
        selection_flag = selection;
        break;
      case 7:
        if (selection_flag != 7){               // set alarm menu
          menuPrinter(selectMenu);              // selection = 7
          functionallity = 2;                   // functionallity = 2
        }
        selection_flag = selection;
        break;
    }
    switch (functionallity){
      case 1:
        potValue = readTouch(0x13);
        potValue /= 203;
        if (potLastState != potValue){
          potLastState = potValue;
        }
        switch (potLastState){
          case 0:   /* Display Mode */
            oledWriteChar1x('>', 0xB2, 0*6);
            oledWriteChar1x(' ', 0xB3, 0*6);
            if (clickFunc == 2)
              selection = 3;
            break;
          case 1:   /* 12H/24H Interval */
            oledWriteChar1x('>', 0xB3, 0*6);
            oledWriteChar1x(' ', 0xB2, 0*6);
            oledWriteChar1x(' ', 0xB4, 0*6);
            if (clickFunc == 2)
              selection = 4;
            break;
          case 2:   /* Set Time */
            oledWriteChar1x('>', 0xB4, 0*6);
            oledWriteChar1x(' ', 0xB3, 0*6);
            oledWriteChar1x(' ', 0xB5, 0*6);
            if (clickFunc == 2)
              selection = 5;
            break;
          case 3:   /* Set Date */
            oledWriteChar1x('>', 0xB5, 0*6);
            oledWriteChar1x(' ', 0xB4, 0*6);
            oledWriteChar1x(' ', 0xB6, 0*6);
            if (clickFunc == 2)
              selection = 6;
            break;
          case 4:   /* Alarm */
            oledWriteChar1x('>', 0xB6, 0*6);
            oledWriteChar1x(' ', 0xB5, 0*6);
            oledWriteChar1x(' ', 0xB7, 6*6);
            if (clickFunc == 2)
              selection = 7;
            break;
          case 5:   /* Go Back */
            oledWriteChar1x('>', 0xB7, 6*6);
            oledWriteChar1x(' ', 0xB6, 0*6);
            if (clickFunc == 2)
              selection = prevSelection;
              resetAnalog();
            break;
        }
        break;
      case 2:
        num2str(str, AlarmFlag.hour);
        oledPutBigChar(str, 3, 4*6);
        num2str(str, AlarmFlag.min);
        oledPutBigChar(str, 3, 10*6);
        resultY = readAcc(0x05);
        if (resultY > 200 && resultY < 500){
          selection = 0;
          AlarmFlag.min = 0;
          AlarmFlag.hour = 0;
          AlarmFlag.sec = 0;
        }
        ADCON0 = 0x11;
        L = mTouchReadButton(3);							//read if left is being touched
        R = mTouchReadButton(0);							//read if right is being touched
        U = mTouchReadButton(1);              //read if up is being touched
        D = mTouchReadButton(2);              //read if down is being touched
        for (i = 4; i < 9; i++){
          oledWriteChar1x('=', 0xB7, i*6);  
          oledWriteChar1x(' ', 0xB7, (i+6)*6);
        }
        while (U < 600) {
          AlarmFlag.hour++;
          if (AlarmFlag.hour > 23){
            AlarmFlag.hour = 0;
          }
          num2str(str, AlarmFlag.hour);
          oledPutBigChar(str, 3, 4*6);
          DelayMs(100);
          U = mTouchReadButton(1);
        }
        while (D < 600){
          AlarmFlag.hour--;
          if (AlarmFlag.hour > 23){
            AlarmFlag.hour = 23;
          }
          num2str(str, AlarmFlag.hour);
          oledPutBigChar(str, 3, 4*6);        
          DelayMs(100);
          D = mTouchReadButton(2);
        }
        if (L < 600){
          selection = prevSelection;
          break;
        }
        if (R < 600){
          DelayMs(100);
          for (i = 4; i < 9; i++){
            oledWriteChar1x(' ', 0xB7, i*6);  
            oledWriteChar1x('=', 0xB7, (i+6)*6);
          }
          while(1){
            L = mTouchReadButton(3);							//read if left is being touched
            R = mTouchReadButton(0);							//read if right is being touched
            U = mTouchReadButton(1);              //read if up is being touched
            D = mTouchReadButton(2);              //read if down is being touched
            if (L < 600){
              for (i = 10; i < 15; i++){
                oledWriteChar1x(' ', 0xB7, i*6);
              }
              DelayMs(100);
              L = mTouchReadButton(3);
              break;
            }
            while (U < 600){
              AlarmFlag.min++;
              if (AlarmFlag.min > 59){
                AlarmFlag.min = 0;
              }
              num2str(str, AlarmFlag.min);
              oledPutBigChar(str, 3, 10*6);
              DelayMs(100);
              U = mTouchReadButton(1);
            }
            while (D < 600){
              AlarmFlag.min--;
              if (AlarmFlag.min > 59){
                AlarmFlag.min = 59;
              }
              num2str(str, AlarmFlag.min);
              oledPutBigChar(str, 3, 10*6);
              DelayMs(100);
              D = mTouchReadButton(2);
            }
            if (R < 600){
              DelayMs(100);
              AlarmFlag.sec = 1;
              selection = 0;
              break;
            }
          }
        }
        break;
      case 5:
        H = 0;
        M = 0;
        S = 0;
        while (selection != 0){
          L = mTouchReadButton(3);							//read if left is being touched
          R = mTouchReadButton(0);							//read if right is being touched
          U = mTouchReadButton(1);              //read if up is being touched
          D = mTouchReadButton(2);              //read if down is being touched
          oledPutBigChar(time, 3, 0);
          for (i = 0; i < 5; i++){
            oledWriteChar1x('=', 0xB7, i*6);  
          }
          if (L < 600){
            for (i = 0; i < 5; i++){
              oledWriteChar1x(' ', 0xB7, i*6);
            }
            DelayMs(100);
            L = mTouchReadButton(3);
            selection = 0;
            break;
          }
          while (U < 600){
            H++;
            if (H > 23){
              H = 0;
            }
            setTempTime(H, M, S, time);
            oledPutBigChar(time, 3, 0);
            DelayMs(100);
            U = mTouchReadButton(1);
          }
          while (D < 600){
            H--;
            if (H > 23){
              H = 23;
            }
            setTempTime(H, M, S, time);
            oledPutBigChar(time, 3, 0);
            DelayMs(100);
            D = mTouchReadButton(2);
          }
          if (R < 600){
            DelayMs(100);
            for (i = 0; i < 5; i++){
              oledWriteChar1x(' ', 0xB7, i*6);  
            }
            while(1){
              L = mTouchReadButton(3);							//read if left is being touched
              R = mTouchReadButton(0);							//read if right is being touched
              U = mTouchReadButton(1);              //read if up is being touched
              D = mTouchReadButton(2);              //read if down is being touched
              for (i = 8; i < 13; i++){
                oledWriteChar1x('=', 0xB7, i*6);
              }
              if (L < 600){
                for (i = 8; i < 13; i++){
                  oledWriteChar1x(' ', 0xB7, i*6);
                }
                DelayMs(100);
                L = mTouchReadButton(3);
                break;
              }
              while (U < 600){
                M++;
                if (M > 59){
                  M = 0;
                }
                setTempTime(H, M, S, time);
                oledPutBigChar(time, 3, 0);
                DelayMs(100);
                U = mTouchReadButton(1);
              }
              while (D < 600){
                M--;
                if (M > 59){
                  M = 59;
                }
                setTempTime(H, M, S, time);
                oledPutBigChar(time, 3, 0);
                DelayMs(100);
                D = mTouchReadButton(2);
              }
              if (R < 600){
                DelayMs(100);
                for (i = 8; i < 13; i++){
                  oledWriteChar1x(' ', 0xB7, i*6);
                }
                while (1){
                  L = mTouchReadButton(3);							//read if left is being touched
                  R = mTouchReadButton(0);							//read if right is being touched
                  U = mTouchReadButton(1);              //read if up is being touched
                  D = mTouchReadButton(2);              //read if down is being touched                  
                  for (i = 16; i < 21; i++){
                    oledWriteChar1x('=', 0xB7, i*6);
                  }
                  if (L < 600){
                    for (i = 16; i < 21; i++){
                      oledWriteChar1x(' ', 0xB7, i*6);
                    }
                    DelayMs(100);
                    L = mTouchReadButton(3);
                    break;
                  }
                  while (U < 600){
                    S++;
                    if (S > 59){
                      S = 0;
                    }
                    setTempTime(H, M, S, time);
                    oledPutBigChar(time, 3, 0);
                    DelayMs(100);
                    U = mTouchReadButton(1);
                  }
                  while (D < 600){
                    S--;
                    if (S > 59){
                      S = 59;
                    }
                    setTempTime(H, M, S, time);
                    oledPutBigChar(time, 3, 0);
                    DelayMs(100);
                    D = mTouchReadButton(2);
                  }
                  if (R < 600){
                    selection = 0;
                    setTime(time);
                    for (i = 0; i < 7; i++){
                      time[i++] = '0';
                      time[i++] = '0';
                    }
                    DelayMs(100);
                    break;
                  }
                }
                break;
              }
            }
          }
        }
        selection = 0;
        break;
      case 6:       /* Set Date Screen */
        while (dateFlag == 0){
          if (changeClockFlag){
            oledPutString(digits, 0, 13*6);
            changeClockFlag = 0;
          }
          ADCON0 = 0x11;
          L = mTouchReadButton(3);							//read if left is being touched
          R = mTouchReadButton(0);							//read if right is being touched
          U = mTouchReadButton(1);              //read if up is being touched
          D = mTouchReadButton(2);              //read if down is being touched
          for (i = 4; i < 9; i++){
            oledWriteChar1x('=', 0xB7, i*6);
            oledWriteChar1x(' ', 0xB7, (i+8)*6);
          }
          while (U < 600) {
            DayValue++;
            if (DayValue > 30){
              DayValue = 1;
            }
            DateUpdate();
            setDate_Menu();
            DelayMs(100);
            U = mTouchReadButton(1);
          }
          while (D < 600){
            DayValue--;
            if (DayValue < 1){
              DayValue = 30;
            }
            DateUpdate();
            setDate_Menu();           
            DelayMs(100);
            D = mTouchReadButton(2);
          }
          if (L < 600){
            selection = prevSelection;
            break;
          }
          if (R < 600){
            DelayMs(100);
            while (1){
              if (changeClockFlag){
                oledPutString(digits, 0, 13*6);
                changeClockFlag = 0;
              }
              L = mTouchReadButton(3);							//read if left is being touched
              R = mTouchReadButton(0);							//read if right is being touched
              U = mTouchReadButton(1);              //read if up is being touched
              D = mTouchReadButton(2);              //read if down is being touched
              for (i = 4; i < 9; i++){
                oledWriteChar1x('=', 0xB7, (i+8)*6);
                oledWriteChar1x(' ', 0xB7, i*6);
              }
              while (U < 600){
                MonthValue++;
                if (MonthValue > 12){
                  MonthValue = 1;
                }
                DateUpdate();
                setDate_Menu();
                DelayMs(100);
                U = mTouchReadButton(1);
              }
              while (D < 600){
                MonthValue--;
                if (MonthValue < 1){
                  MonthValue = 12;
                }
                DateUpdate();
                setDate_Menu();
                DelayMs(100);
                D = mTouchReadButton(2);                
              }
              if (R < 600){
                dateFlag = 1;
                selection = 0;
                break;
              }
              if (L < 600){
                DelayMs(100);
                break;
              }
            }   
          }
        }
      break;
    }
  }
}//end main

/** EOF main.c *************************************************/