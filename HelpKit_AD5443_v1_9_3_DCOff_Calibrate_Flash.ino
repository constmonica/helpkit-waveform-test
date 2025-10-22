#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

#include "AD9833.h"
#include <Versatile_RotaryEncoder.h>  //Versatile_RotaryEncoder by Rui Seixas Monteiro, 1.3.1

#include "pwm.h"

#include <EEPROM.h>  //to store Offset value

//#include "ArduinoGraphics.h"
//#include "Arduino_LED_Matrix.h"
//ArduinoLEDMatrix matrix;


#include <Adafruit_SH110X.h>
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 128
#define SSD1306_WHITE SH110X_WHITE
#define SSD1306_BLACK SH110X_BLACK

#include "FireTimer.h"  //Albert: Pentru intreruperi periodice: bilioteca FireTimer by PowerBroker2, 1.05

/***************************************************************
    Definire Mesagerie Debug
***************************************************************/
//StackOverfolw hack:
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINT(x) \
  do { \
  } while (0)
#endif

/*************************************************************/

/***************************************************************
    Definire Configurare Program
***************************************************************/

/*************************************************************/


/*************************************************************/

/**********************************************************************
*       Rotary Encoder and Timer 
***********************************************************************/
// Pin definitions.
// - enc_a is ENC Signal A line (Arduino analog pin A0)
// - enc_b is ENC Signal B line (Arduino analog pin A1)

#define NUMTURN_NAV_GRANULARITY 1
#define NUMTURN_ADJ_GRANULARITY 1

// Encoder reading pins
#define clk A1  // (A1 = ROT-B)
#define dt A0   // (A0 = ROT-A)
#define sw 0    // (D0 = RXD = SW)

//Create a global pointer for the encoder object
Versatile_RotaryEncoder *versatile_encoder;

volatile uint8_t NumTurnRight = 0, NumTurnLeft = 0;

//events are: NoEvent=0, RightTurn, LeftTurn, Press
enum RotaryEvents { NoEvent = 0,
                    RightTurn,
                    LeftTurn,
                    Press };
volatile enum RotaryEvents RotaryEvent = NoEvent;

//RotaryEvent will act as a flag
extern volatile enum RotaryEvents RotaryEvent;

//timer
FireTimer msTimer;


void InitializeRotaryEncoder();

/***********************************************************************/


/***************************************************************
    Amplitude and DC Off LSB slopes, //updated to changed gain values after 23 Apr
***************************************************************/


const float SinLGainVperLsb = 0.0053311;
//Older value, Helpkit pre-release
//float SinLGainVperLsb = 0.004942;

const float SQWLGainVperLsb = 0.02914;
//Older value, Helpkit pre-release
//float SQWLGainVperLsb = 0.02946;

////updated to new gain values after 2025 Sep 16
const float DcOffperLSB = 0.0031640;  //

int LSBOFF = 0;

uint16_t CalibAddress = 0x10;  //starting to write the Offset value from CalibAddress

//Structure holding the Offset value in EEPROM
typedef struct {
  char ID = 'O';
  int CalLsbOff = 0;
} CalibrationValues;
CalibrationValues CVal;



/**********************************************************************
*   Generator Menu defs and functions
***********************************************************************/
#define MINVOLTAGE 0.1
#define MAXVOLTAGE 20.0
#define SIN_SWGAIN_VOLTAGE 4.1
#define MAXFREQ 2000000
#define MINFREQ 10
#define MAXDC 10.0
#define MINDC -10.0

enum FreqOrders { Hz = 0,
                  tHz,
                  hHz,
                  KHz,
                  tKHz,
                  hKHz,
                  MHz,
                  NumFreqOrders };



//Pentru Main Menu
enum MainMenuOption { BASIC_DEMO = 0,
                      SIGGEN,
                      PWMGEN,
                      CALIBRATE,
                      MMENU_SIZE };
MainMenuOption currentMainOption = BASIC_DEMO;  //se schimba la rotire in Main Menu

//Main menu or submenu must be displayed
enum MainOrSub { Main = 0,
                 Sub };
MainOrSub currentMenuOption = Main;

const char MainMenuText[MMENU_SIZE][25] = { "Basic Demo", "Signal Gen", "PWM Gen", "Calibrate Offset" };


const char FreqText[7][4] = { " Hz", " Hz", " Hz", "KHz", "KHz", "KHz", "MHz" };

enum IncOrDec { inc = 0,
                dec };


//Pentru Generator Menu
enum GenMenuOption { SIGNAL_TYPE = 0,
                     VOLTAGE,
                     FREQUENCY,
                     DC_VALUE,
                     GMENU_SIZE };
GenMenuOption currentGenOption = SIGNAL_TYPE;

enum Wavetypes { OFF = 0,
                 DC,
                 Sin,
                 Tri,
                 Sqw1,
                 Numwaves };
const char WaveText[Numwaves][5] = { "OFF", "DC", "Sine", "Tri", "Sqw" };

enum GenSelModes { GenMenuSel = 0,
                   ValueSel,
                   ValueOrdSel,
                   NumSelModes };
enum GenSelModes GenSelMode = GenMenuSel;

//Pentru PWM menu
enum PMenuOption { PSIGNAL_TYPE = 0,
                   PFREQUENCY,
                   PDUTY,
                   PMENU_SIZE };
PMenuOption currentPwmOption = PSIGNAL_TYPE;

enum PSignalTypes { POff = 0,
                    POn,
                    NUMPSIGNALS };
const char PsignalText[NUMPSIGNALS][5] = { "OFF", "ON" };

enum PwmSelModes { PwmMenuSel = 0,
                   PwmValueSel,
                   PwmValueOrdSel,
                   PNumSelModes };
enum PwmSelModes PwmSelMode = PwmMenuSel;


//Structure holding the Generator values
typedef struct {
  enum Wavetypes Wavetype = OFF;
  enum FreqOrders FreqOrder = KHz;
  float Freq = 1.0;
  float Amp = 1.0;
  float DC = 0.0;

} GMenuItems;

GMenuItems GMenu;

//Structure holding the PWM values
typedef struct {
  enum PSignalTypes PSignaltype = POff;
  enum FreqOrders FreqOrder = hHz;
  float Freq = 100;
  float Duty = 50.0;
} PMenuItems;
PMenuItems PMenu;


//RAW Potmeter and Offset DAC settings
unsigned int PotVal = 100;
int OffsetDACVal = 0;


enum FreqOrders FreqOrd_Prev = GMenu.FreqOrder;
enum FreqOrders PwmFreqOrd_Prev = PMenu.FreqOrder;

//Flags to which menu item to update on Generator display
uint8_t flGmenuUpd[6];

//Flags to which menu item to update on PWM display
uint8_t flPmenuUpd[4];

int signalTypeIndex = 0;

//extern enum MenuOption currentOption;

int signalType = 0;
//int currentSignalIndex = 0;
//const int numSignalTypes = sizeof(signalTypes) / sizeof(signalTypes[0]);

int i = 0;

void UpdFreqfromOrd(GMenuItems *Gm);
uint8_t IncDecGenFreq(GMenuItems *Gm, enum IncOrDec I_d);

/**********************************************************************
*    AD9833 AWG + Potmeters, DAC, PWM
***********************************************************************/
#define AD9833_CS 10
#define AD9833_MOSI 11
#define AD9833_SCK 13

#define PWM_PIN D9

PwmOut objPWMD9(PWM_PIN);

//variables for slow PWM
//unsigned int SlowPwmCount = 0;
uint32_t SlowPwmPeriod = 1000000;
uint32_t SlowPwmPulseWidth = SlowPwmPeriod / 2;
uint8_t EnableSlowPWM = 0;


/*********************Not needed in the swichless version Cut from here*************************************************/
// //signal showing the status of the Gain switch: High or Low Gain
// #define H_L_GAIN_SW 6
// uint8_t flChangeGainSwPos = 0;
/*********************Not needed in the swichless version Cut until here*************************************************/

//AD9833 can be used with Soft SPI too
//AD9833 AD(AD9833_CS, AD9833_MOSI, AD9833_SCK);

//But now we use HW_SPI
AD9833 AD(AD9833_CS);

//Generator function prototypes
void UpdateAD9833Frequency(GMenuItems *Gm);
void UpdateAD9833Wave(GMenuItems *Gm);
void updateGenAndPot(GMenuItems *Gm);
void UpdateGenAmp(GMenuItems *Gm);
uint16_t WritePot(uint16_t Data);
/**********************************************************************
*    AD5443 added
***********************************************************************/
uint16_t WriteAD5443(uint16_t Data);
uint16_t WriteOffDACData(uint16_t Data, uint8_t Channel);
void UpdateGenDC(GMenuItems *Gm);

uint8_t Chk_GSW_Needed_POS(GMenuItems *Gm);

// #define DIG_POT_SYNCN  8
// #define DIG_POT_DIN    11
// #define DIG_POT_SDO    12
// #define DIG_POT_SCLK   13
// #define DIG_POT_RESETN 7
// #define DIG_POT_RDY    2

/**********************************************************************
*    AD5443 definitions added
***********************************************************************/
#define AD5443_SYNCN 8
//AD5443 Definitions
#define DAC_5443_NOP 0b0000 << 12
#define DAC_5443_LOAD_UPDATE 0b0001 << 12
#define DAC_5443_INIT_READBACK 0b0010 << 12
#define DAC_5443_DIS_DAISYCHAIN 0b1001 << 12
#define DAC_5443_CLK_RISING_EDGE 0b1010 << 12
#define DAC_5443_CLEAR_DAC_ZERO 0b1011 << 12
#define DAC_5443_CLEAR_DAC_MID 0b1100 << 12


#define AD5293_NOP_CMD 0x00
#define AD5293_WRITE_RDAC_CMD 0x1 << 10
#define AD5293_READ_RDAC_CMD 0x2 << 10
#define AD5293_RESET_CMD 0x4 << 10
#define AD5293_WRITE_CTRL_CMD 0x6 << 10
#define AD5293_READ_CTRL_CMD 0x7 << 10
#define AD5293_PWRDOWN_CMD 0x8 << 10

//SPI transaction on the potmeter
uint16_t SendPotData = 0;
uint16_t RecvPotData = 0xFFFF;

/**********************************************************************
*    AD5625R (BRUZ-1) Offset DAC, 14-Lead TSSOP
***********************************************************************/
#define AD5625_I2C_ADDR 0b0011110  // ADDR = 0x1E! i.e. 0x3C write, 0x3D read

#define CMD_WRITE_REG_N 0b000
#define CMD_UPDATE_REG_N 0b001
#define CMD_WRITE_UPDATE_ALL 0b010
#define CMD_WRITE_UPDATE_N 0b011
#define CMD_PWR_REG 0b100
#define CMD_RESET 0b101
#define CMD_LDAC_REG 0b110
#define CMD_INTERNAL_REF 0b111

//LDAC: LDAC pin is inactive.
//For automatic LDAC i.e. update DAC when writing to reg, on Channels A and B,
//Comand to send: Byte 0: (CMD_LDAC_REG << 3); Byte 1: 0 (=Don't care, X); Byte2: 0x03
//(last four bits in Byte3: DAC_D DAC_C DAC_B DAC_A )

//Power Up/Down:
//Power down DAC C and D: Send Byte 0: (CMD_PWR_REG << 3); Byte 1: 0 (=Don't care, X); Byte2: 0x03C
//Byte 2 Bit 4, Bit 5: 00: Normal, 11: Hi-Z, Bit 3..0: DAC D..A
//Power up DAC A and B: Send Byte 0: (CMD_PWR_REG << 3); Byte 1: 0 (=Don't care, X); Byte2: 0x003

//Internal Reference: At power-up, the internal ref is stopped
//To turn on internal Ref: Send Byte 0: (CMD_INTERNAL_REF << 3); Byte 1: 0 (=Don't care, X); Byte2: 0x01

#define ADDR_DAC_A 0b000
#define ADDR_DAC_B 0b001
#define ADDR_DAC_C 0b010
#define ADDR_DAC_D 0b011
#define ADDR_DAC_ALL 0b111

//Data buffer to write DAC data
uint8_t OffDacData[3] = { 0, 0, 0 };

/**********************************************************************
*    SSD1306 display 
***********************************************************************/



#define ICON_SIZE 15

//Generator menu graphic alignments
#define ROW_SPACING 7
#define RECTANGLE_SEL_HEIGHT 16

//PWM menu graphic alignments
#define PWM_ROW_SPACING 7
#define PWM_RECTANGLE_SEL_HEIGHT 16

//Main menu graphic alignments
#define MAIN_ROW_SPACING 8
#define MAIN_RECTANGLE_SEL_HEIGHT 16


//Instantiate display
Adafruit_SH1107 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);



void drawPoint(uint8_t x, uint8_t y) {
  display.drawPixel(x, y, SH110X_WHITE);
  display.drawPixel(x, y + 1, SH110X_WHITE);
  display.drawPixel(x + 1, y, SH110X_WHITE);
  display.drawPixel(x + 1, y + 1, SH110X_WHITE);
}


// x should start at 10 since the width of an icon is 10 pixels
void drawText(uint8_t x, uint8_t y, const char text[]) {
  uint8_t current_position = x;
  for (uint8_t i = 0; text[i] != '\0'; i++) {
    if (text[i] == '.') {
      drawPoint(current_position, y + 12);
      current_position += 2;
    } else {
      display.drawChar(current_position, y, text[i], SSD1306_WHITE, SSD1306_BLACK, 2);
      current_position += 11;
    }
  }
}

//Function used to display the switch warning
//void StopandDispSwitchMsg(char* c, uint8_t ExpVal);

/***********************************************************************
*   Calibration Menu Messages
***********************************************************************/
const char CalMsg1[] = "Connect Voltmeter to";
const char CalMsg2[] = "Sig Out";
const char CalMsg3[] = "Rotate until Vout~0,";
const char CalMsg4[] = "then Press";
const char CalMsg5[] = "New Offset Value";
const char CalMsg6[] = "Stored";

//when entering the Calibration Menu, flag to display text with animation
uint8_t flCalMenuFirstRun = 1;

//At Rotary Press, display the Calibration Menu End
uint8_t fCalMenuEnd = 0;

/**********************************************************************
***********************************************************************
*   Function Prototypes
***********************************************************************/
void updateMainMenu();
void navigateMainMenu();

void updateDemoMenu();

void updateGenMenu(GMenuItems *Gm);
void adjustGenValue(GMenuItems *Gm);
void navigateGenMenu(GMenuItems *Gm);


void updatePwmMenu(PMenuItems *Pm);
void adjustPwmValue(PMenuItems *Pm);
void navigatePwmMenu(PMenuItems *Pm);
void updatePwmGen(PMenuItems *Pm);

float updatePwmFrequency(PMenuItems *Pm);

void updateCalMenu();
void adjustCalValue();
void navigateCalMenu();


/***********************************************************************
**********************************************************************
*       setup()
***********************************************************************
***********************************************************************/
void setup() {
  Serial.begin(57600);

  if (!display.begin(0x3C, 1)) {
    Serial.println("Display Error");
    for (;;)
      ;
  }
  DEBUG_PRINT("Display Initialized\r\n");
  display.display();
  delay(1500);
  display.setRotation(1);
  display.clearDisplay();
  display.setTextSize(1);
  //display.setTextSize(1,2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.display();


  InitializeRotaryEncoder();
  // Read the encoder every 1 miliseconds
  msTimer.begin(1);

  SPI.begin();
  /**********************************************************************
*  EEPROM init
***********************************************************************/
  EEPROM.get(CalibAddress, CVal);
  //DEBUG_PRINT("EEPROM Calib ID = "); DEBUG_PRINT(String(CVal.ID)); DEBUG_PRINT(" EEPROM Calib Off Val = "); DEBUG_PRINT(String(CVal.CalLsbOff));  DEBUG_PRINT("\r\n");
  if (CVal.ID != 'O') {
    //DEBUG_PRINT("No stored Offset Value found, setting LSBOFF to 0"); DEBUG_PRINT("\r\n");
    LSBOFF = 0;
  } else {
    LSBOFF = CVal.CalLsbOff;
    //DEBUG_PRINT("Loading LSBOFF from EEPROM, "); DEBUG_PRINT(String(LSBOFF)); DEBUG_PRINT("\r\n");
  }

  /**********************************************************************
*  PWM init
***********************************************************************/
  pinMode(PWM_PIN, OUTPUT);


  /**********************************************************************
*  For probing AD5443 MDAC: Write the maximum to the POT value
***********************************************************************/
  //WritePot(0x3FF);

  pinMode(AD5443_SYNCN, OUTPUT);
  digitalWrite(AD5443_SYNCN, HIGH);

  /**********************************************************************
*  Setup AD5625R Offset DAC
***********************************************************************/
  //Power Down DAC C, D
  OffDacData[0] = (CMD_PWR_REG << 3);
  OffDacData[1] = 0;
  OffDacData[2] = 0x3C;
  Wire.begin();
  Wire.beginTransmission(AD5625_I2C_ADDR);
  Wire.write(OffDacData, 3);
  i = Wire.endTransmission();
  //DEBUG_PRINT("AD562R Offset DAC response after I2C  transmission= ");
  //DEBUG_PRINT(i); DEBUG_PRINT("\r\n");

  //Power up DAC A, B
  OffDacData[0] = (CMD_PWR_REG << 3);
  OffDacData[1] = 0;
  OffDacData[2] = 0x03;
  Wire.beginTransmission(AD5625_I2C_ADDR);
  Wire.write(OffDacData, 3);
  i = Wire.endTransmission();

  //Enable automatic LDAC on DAC A and DAC B
  OffDacData[0] = (CMD_LDAC_REG << 3);
  OffDacData[1] = 0;
  OffDacData[2] = 0x03;
  Wire.beginTransmission(AD5625_I2C_ADDR);
  Wire.write(OffDacData, 3);
  i = Wire.endTransmission();

  //Turn on internal reference
  OffDacData[0] = (CMD_INTERNAL_REF << 3);
  OffDacData[1] = 0;
  OffDacData[2] = 0x01;
  Wire.beginTransmission(AD5625_I2C_ADDR);
  Wire.write(OffDacData, 3);
  i = Wire.endTransmission();
  //DEBUG_PRINT("AD562R Offset DAC response after I2C  transmission= ");
  //DEBUG_PRINT(i); DEBUG_PRINT("\r\n");

  //Set both AD5625R channels to half-scale
  WriteOffDACData(2048, ADDR_DAC_A);
  WriteOffDACData(2048, ADDR_DAC_B);

  /**********************************************************************
*  Setup Generators
***********************************************************************/
  //the quartz is 20MHz, not 25MHz
  AD.setCrystalFrequency(20000000.0);
  AD.begin();

  // DEBUG_PRINT("AD9833 SPI Speed="); DEBUG_PRINT(AD.getSPIspeed()); DEBUG_PRINT(" AD9833 uses HW SPI ="); DEBUG_PRINT(AD.usesHWSPI()); DEBUG_PRINT("\r\n");



  //float crystalFrecv = AD.getCrystalFrequency();
  //DEBUG_PRINT("Crystal Frequency = "); DEBUG_PRINT(crystalFrecv); DEBUG_PRINT("\r\n");
  //AD.setWave(AD9833_SQUARE2);

  //Force an update on the Wavegen
  for (i = 0; i < 6; i++) flGmenuUpd[i] = 1;
  updateGenAndPot(&GMenu);

  float PwmFrequency = updatePwmFrequency(&PMenu);
  objPWMD9.begin(PwmFrequency, PMenu.Duty);

  objPWMD9.suspend();

  DEBUG_PRINT("Initialized\r\n");

  //updateGenMenu(&GMenu);

  //acum pornim Main Menu
  updateMainMenu();
}
/***********************************************************************/


/**********************************************************************
**********************************************************************
*       loop()
**********************************************************************
**********************************************************************/
void loop() {

  //periodicallly read the encoder data
  if (msTimer.fire()) versatile_encoder->ReadEncoder();

  /**********************************************************************
*       Handle Rotate in Menus
**********************************************************************/
  if (((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) || ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY))) {
    // DEBUG_PRINT("Rot Event\r\n");
    if (currentMenuOption == Main) {
      navigateMainMenu();
    } else {
      switch (currentMainOption) {
        case BASIC_DEMO:
          break;
        case SIGGEN:
          if (GenSelMode != GenMenuSel) {
            adjustGenValue(&GMenu);
          } else {
            navigateGenMenu(&GMenu);
          }
          updateGenAndPot(&GMenu);
          break;
        case PWMGEN:
          if (PwmSelMode != PwmMenuSel) {
            adjustPwmValue(&PMenu);
          } else {
            navigatePwmMenu(&PMenu);
          }
          updatePwmGen(&PMenu);
          break;
        case CALIBRATE:
          navigateCalMenu();
          updateGenAndPot(&GMenu);
          break;
        default: break;
      }
    }
  }

  /**********************************************************************
*       Handle Press in Menus
**********************************************************************/
  if (RotaryEvent == Press) {
    if (currentMenuOption == Main) {  //we are in the main menu
      currentMenuOption = Sub;
      switch (currentMainOption) {
        case BASIC_DEMO:
          updateDemoMenu();
          break;
        case SIGGEN:
          updateGenMenu(&GMenu);
          break;
        case PWMGEN:
          updatePwmMenu(&PMenu);
          break;
        case CALIBRATE:
          //turn off the generator before launching calibration
          GMenu.DC = 0.0;
          flGmenuUpd[0] = 1;  //raise flag to update Generator Waveform
          flGmenuUpd[4] = 1;  //raise flag to update Generator DC Offset
          GMenu.Wavetype = DC;
          updateGenAndPot(&GMenu);  //Some bug: I have to call updateGenAndPot() twice
          //AD.setWave(AD9833_OFF);
          GMenu.Wavetype = DC;
          flGmenuUpd[0] = 1;  //raise flag to update Generator Waveform
          updateGenAndPot(&GMenu);
          updateCalMenu();
          break;
        default: break;
      }
    } else {  //we are in a sub-menu
      switch (currentMainOption) {
        case BASIC_DEMO:
          break;
        case SIGGEN:
          if (currentGenOption == FREQUENCY)
            GenSelMode = (GenSelMode == GenMenuSel) ? ValueOrdSel : (GenSelMode == ValueOrdSel) ? ValueSel
                                                                                                : GenMenuSel;  //La frecventa baleiem pe toate cele 3 moduri de selectie, dar dupa MenuSel urmeaza ValueOrdSel
          else
            GenSelMode = (GenSelMode == ValueOrdSel) ? GenMenuSel : (GenSelMode == GenMenuSel) ? ValueSel
                                                                                               : GenMenuSel;  //La celelalte optiuni baleiem doar intre Menu si valoare
          //DEBUG_PRINT("SelMode ="); DEBUG_PRINT(SelMode); DEBUG_PRINT("\r\n");
          updateGenMenu(&GMenu);
          updateGenAndPot(&GMenu);
          break;
        case PWMGEN:
          if (currentPwmOption == PFREQUENCY)
            PwmSelMode = (PwmSelMode == PwmMenuSel) ? PwmValueOrdSel : (PwmSelMode == PwmValueOrdSel) ? PwmValueSel
                                                                                                      : PwmMenuSel;  //La frecventa baleiem pe toate cele 3 moduri de selectie, dar dupa MenuSel urmeaza ValueOrdSel
          else
            PwmSelMode = (PwmSelMode == PwmValueOrdSel) ? PwmMenuSel : (PwmSelMode == PwmMenuSel) ? PwmValueSel
                                                                                                  : PwmMenuSel;  //La celelalte optiuni baleiem doar intre Menu si valoare
          //DEBUG_PRINT("PwmSelMode ="); DEBUG_PRINT(PwmSelMode); DEBUG_PRINT("\r\n");
          updatePwmMenu(&PMenu);
          updatePwmGen(&PMenu);
          break;
        case CALIBRATE:
          //Exit Calibration Menu by Peress
          navigateCalMenu();
          break;
        default: break;
      }
    }
    RotaryEvent = NoEvent;
  }




}  //loop
/***********************************************************************/


/**********************************************************************
*       navigateMainMenu(): Select Main menu operation
***********************************************************************/
void navigateMainMenu() {
  //la Right Turn "urcam" in meniu, deoarece prima optiune e cea "mai de sus", cu index mai mic
  if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {  //limitez cu if, deoarece constrain() nu limiteaza corect la enum
    //Constrangerea pentru uint si limita 0 nu mai functioneaza corect, din cauza asta aici folosesc atribuire ternara
    currentMainOption = (currentMainOption == BASIC_DEMO) ? BASIC_DEMO : (MainMenuOption)(currentMainOption - 1);
    //DEBUG_PRINT("Current Main Option = "); DEBUG_PRINT(currentMainOption); DEBUG_PRINT("\r\n");
    RotaryEvent = NoEvent;
    NumTurnRight = 0;
  }
  if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
    currentMainOption = constrain((MainMenuOption)(currentMainOption + 1), BASIC_DEMO, CALIBRATE);
    //DEBUG_PRINT("Current Option = "); DEBUG_PRINT(currentMainOption); DEBUG_PRINT("\r\n");
    RotaryEvent = NoEvent;
    NumTurnLeft = 0;
  }
  updateMainMenu();
}




/**********************************************************************
*       updateMainMenu(): Displays the Main Menu and updates the display
***********************************************************************/
void updateMainMenu() {
  String ValString = "";
  //Aprox. 21 caractere incap intr-un rand => 1 caracter = 128/21 = 6 pixeli
  //Text Begin Position for each text
  //const uint8_t Text_Begin[MMENU_SIZE]={15, }
  uint8_t TextBegin, RectangleBegin, RectangleEnd;

  display.clearDisplay();
  display.setCursor(5, 0);
  for (int i = 0; i < MMENU_SIZE; i++) {
    TextBegin = ((128 - (6 * strlen(MainMenuText[i]))) / 2) - 1;
    RectangleBegin = TextBegin - 5;
    RectangleEnd = 6 * strlen(MainMenuText[i]) + 10;
    if (i == currentMainOption) {
      display.drawRect(RectangleBegin, i * (8 + MAIN_ROW_SPACING), RectangleEnd, MAIN_RECTANGLE_SEL_HEIGHT - 1, SSD1306_WHITE);  //Albert: Mai bine inconjuram cu dreptunghi, se vede mai bine
    }


    display.setCursor(TextBegin, i * (8 + MAIN_ROW_SPACING) + MAIN_ROW_SPACING / 2);  //Albert
    ValString = String(MainMenuText[i]);
    display.print(ValString);
  }
  display.display();
}



/**********************************************************************
*       updateDemoMenu(): Displays the Demo and updates the display
***********************************************************************/
void updateDemoMenu() {
  String ValString = "";
  display.clearDisplay();

  // --- DEMO CONFIG: Sine 1 MHz, 10.0 Vpp, +5.0 V DC ---
  GMenu.Wavetype  = Sin;
  GMenu.FreqOrder = MHz;
  GMenu.Freq      = 1.0;
  GMenu.Amp       = 10.0;  // Vpp
  GMenu.DC        = 5.0;   // +5 V offset

  // push to hardware (same update pipeline/style)
  flGmenuUpd[0] = 1; // waveform
  flGmenuUpd[1] = 1; // frequency value
  flGmenuUpd[2] = 1; // frequency order
  flGmenuUpd[3] = 1; // amplitude
  flGmenuUpd[4] = 1; // DC offset
  updateGenAndPot(&GMenu);
  // -----------------------------------------------------

  // --- DEMO SCREEN ---
  display.setCursor(5, 6);
  ValString = "Waveform : " + String(WaveText[GMenu.Wavetype]);
  display.print(ValString);

  display.setCursor(5, 22);
  ValString = "Vampl-pp : " + String(GMenu.Amp, 1) + " V";
  display.print(ValString);

  display.setCursor(5, 38);
  switch (GMenu.FreqOrder) {
    case Hz:
    case KHz:
    case MHz:
      ValString = "Frequency: " + String(GMenu.Freq, 1) + " " + String(FreqText[GMenu.FreqOrder]);
      break;
    case tHz:
    case tKHz:
      ValString = "Frequency:  " + String(GMenu.Freq, 0) + " " + String(FreqText[GMenu.FreqOrder]);
      break;
    case hHz:
    case hKHz:
      ValString = "Frequency: " + String(GMenu.Freq, 0) + " " + String(FreqText[GMenu.FreqOrder]);
      break;
    default:
      ValString = "Frequency: " + String(GMenu.Freq, 1) + " " + String(FreqText[GMenu.FreqOrder]);
      break;
  }
  display.print(ValString);

  display.setCursor(5, 54);
  ValString = "DC Offset: " + String(GMenu.DC, 1) + " V";
  display.print(ValString);

  display.display();
}


/**********************************************************************
*       navigatePwmMenu(): Updates current PWM Generator item selection
***********************************************************************/
void navigatePwmMenu(PMenuItems *Pm) {
  //la Right Turn "urcam" in meniu, deoarece prima optiune e cea "mai de sus", cu index mai mic
  if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {  //limitez cu if, deoarece constrain() nu limiteaza corect la enum
    //Constrangerea pentru uint si limita 0 nu mai fucntioneaza corect, din cauza asta aici folosesc atribuire ternara
    currentPwmOption = (currentPwmOption == PSIGNAL_TYPE) ? PSIGNAL_TYPE : (PMenuOption)(currentPwmOption - 1);
    //DEBUG_PRINT("Current PWM Option = "); DEBUG_PRINT(currenPwmtOption); DEBUG_PRINT("\r\n");
    RotaryEvent = NoEvent;
    NumTurnRight = 0;
  }
  if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
    currentPwmOption = constrain((PMenuOption)(currentPwmOption + 1), PSIGNAL_TYPE, PDUTY);
    //DEBUG_PRINT("Current PWM Option = "); DEBUG_PRINT(currentPwmOption); DEBUG_PRINT("\r\n");
    RotaryEvent = NoEvent;
    NumTurnLeft = 0;
  }
  updatePwmMenu(&PMenu);
}


/**********************************************************************
*       updatePwmMenu(): Displays the PWM menu and updates the display
***********************************************************************/
void updatePwmMenu(PMenuItems *Pm) {
  String ValString = "";
  //int val_fract, val_int;
  display.clearDisplay();
  display.setCursor(5, 0);
  for (int i = 0; i < PMENU_SIZE; i++) {
    if (i == currentPwmOption) {
      switch (PwmSelMode) {
        case PwmMenuSel:
          display.drawRect(1, i * (8 + PWM_ROW_SPACING) + 3, 68, PWM_RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);  //Albert: Mai bine inconjuram cu dreptunghi, se vede mai bine
          break;
        case PwmValueSel:
          if (currentPwmOption == PFREQUENCY) display.drawRect(66, i * (8 + PWM_ROW_SPACING) + 3, 26, PWM_RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);  //Albert: La Frecv avem doar 3 cifre de inconjurat
          else display.drawRect(68, i * (8 + PWM_ROW_SPACING) + 3, 55, PWM_RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);                                 //Albert: Inconjuram optiunea aleasa
          break;
        case PwmValueOrdSel:  //asta poate fi doar la PwmFrequency
          display.drawRect(91, i * (8 + PWM_ROW_SPACING) + 3, 31, PWM_RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);
          break;
        default: break;
      }
    }
    display.setCursor(5, i * (8 + PWM_ROW_SPACING) + PWM_ROW_SPACING);  //Albert
    switch (i) {
      case PSIGNAL_TYPE:
        {
          ValString = "PWM  Out :  " + String(PsignalText[Pm->PSignaltype]);
          display.print(ValString);  // "OFF", "ON"
        }
        break;
      case PFREQUENCY:
        //enum FreqOrders{Hz=0, tHz, hHz, KHz, tKHz, hKHz, MHz, NumFreqOrders};
        switch (Pm->FreqOrder) {
          case Hz:
            ValString = "Frequency: " + String(Pm->Freq, 1) + String(" ") + String(FreqText[Pm->FreqOrder]);
            break;
          case KHz:
          case MHz:
            ValString = "Frequency: " + String(Pm->Freq, 1) + String(" ") + String(FreqText[Pm->FreqOrder]);
            break;
          case tHz:
          case tKHz:
            ValString = "Frequency: " + String(" ") + String(Pm->Freq, 0) + String(" ") + String(FreqText[Pm->FreqOrder]);
            break;
          case hHz:
          case hKHz:
            ValString = "Frequency: " + String(Pm->Freq, 0) + String(" ") + String(FreqText[Pm->FreqOrder]);
            break;
          default: break;
        }
        display.print(ValString);
        break;
      case PDUTY:
        ValString = "Duty Cyc.:  " + String(Pm->Duty, 1) + " %";
        display.print(ValString);
        break;
    }
  }
  display.display();
}

/**********************************************************************
*       navigateCalMenu(): adjusts the calibration value
***********************************************************************/
void navigateCalMenu() {
  //la Right Turn "urcam" in meniu, deoarece prima optiune e cea "mai de sus", cu index mai mic
  if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
    LSBOFF++;
    flGmenuUpd[4] = 1;
    RotaryEvent = NoEvent;
    NumTurnRight = 0;
  }
  if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
    LSBOFF--;
    flGmenuUpd[4] = 1;
    RotaryEvent = NoEvent;
    NumTurnLeft = 0;
  }

  if (RotaryEvent == Press) {
    fCalMenuEnd = 1;
    //DEBUG_PRINT("Saving LSBOFF to EEPROM, "); DEBUG_PRINT(String(LSBOFF)); DEBUG_PRINT("\r\n");
    //Store the new offset value in the EEPROM
    CVal.ID = 'O';
    CVal.CalLsbOff = LSBOFF;
    EEPROM.put(CalibAddress, CVal);

    RotaryEvent = NoEvent;
  }

  updateCalMenu();
}

/**********************************************************************
*       updateCalMenu(): Displays the Calibration and updates the display
***********************************************************************/
void updateCalMenu() {
  String ValString = "";
  int i = 0;
  uint16_t deltime = 25;
  display.clearDisplay();

  if (flCalMenuFirstRun) {
    display.setCursor(1, 5);
    for (i = 0; i < strlen(CalMsg1); i++) {
      display.print(CalMsg1[i]);
      display.display();
      delay(deltime);
    }
    //delay(500);
    display.setCursor(1, 15);
    for (i = 0; i < strlen(CalMsg2); i++) {
      display.print(CalMsg2[i]);
      display.display();
      delay(deltime);
    }
    delay(500);
    display.setCursor(1, 30);
    for (i = 0; i < strlen(CalMsg3); i++) {
      display.print(CalMsg3[i]);
      display.display();
      delay(deltime);
    }
    //delay(500);
    display.setCursor(1, 40);
    for (i = 0; i < strlen(CalMsg4); i++) {
      display.print(CalMsg4[i]);
      display.display();
      delay(deltime);
    }
    delay(500);
    flCalMenuFirstRun = 0;
  } else {
    display.setCursor(1, 5);
    ValString = String(CalMsg1);
    display.print(ValString);

    display.setCursor(1, 15);
    ValString = String(CalMsg2);
    display.print(ValString);

    display.setCursor(1, 30);
    ValString = String(CalMsg3);
    display.print(ValString);

    display.setCursor(1, 40);
    ValString = String(CalMsg4);
    display.print(ValString);
  }
  display.setCursor(25, 55);
  ValString = String("Offset Lsb = ") + String(LSBOFF);
  display.print(ValString);

  if (fCalMenuEnd == 1) {
    display.clearDisplay();
    display.setCursor(15, 5);
    for (i = 0; i < strlen(CalMsg5); i++) {
      display.print(CalMsg5[i]);
      display.display();
      delay(deltime);
    }
    delay(200);

    display.setCursor(25, 20);
    ValString = String("Offset Lsb = ") + String(LSBOFF);
    display.print(ValString);
    display.display();
    delay(200);

    display.setCursor(50, 35);
    display.print(CalMsg6);
    display.display();
    delay(200);

    for (i = 2; i >= 0; i--) {
      //redisplay menu end
      display.clearDisplay();
      display.setCursor(15, 5);
      display.print(CalMsg5);
      display.setCursor(25, 20);
      ValString = String("Offset Lsb = ") + String(LSBOFF);
      display.print(ValString);
      display.setCursor(50, 35);
      display.print(CalMsg6);
      //now disply down counter
      display.setCursor(63, 45);
      display.print(String(i));
      display.display();
      delay(500);
    }
    fCalMenuEnd = 0;
    currentMenuOption = Main;
    currentMainOption = SIGGEN;
    updateMainMenu();
  }

  display.display();
}


/**********************************************************************
*       navigateGenMenu(): Updates current Generator item selection
***********************************************************************/
void navigateGenMenu(GMenuItems *Gm) {
  //la Right Turn "urcam" in meniu, deoarece prima optiune e cea "mai de sus", cu index mai mic
  if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {  //limitez cu if, deoarece constrain() nu limiteaza corect la enum
    if (Gm->Wavetype == DC) {
      currentGenOption = SIGNAL_TYPE;
    } else {
      //Constrangerea pentru uint si limita 0 nu mai fucntioneaza corect, din cauza asta aici folosesc atribuire ternara
      currentGenOption = (currentGenOption == SIGNAL_TYPE) ? SIGNAL_TYPE : (GenMenuOption)(currentGenOption - 1);
      //DEBUG_PRINT("Current Option = "); DEBUG_PRINT(currentOption); DEBUG_PRINT("\r\n");
    }
    RotaryEvent = NoEvent;
    NumTurnRight = 0;
  }
  if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
    if (Gm->Wavetype == DC) {
      currentGenOption = DC_VALUE;
    } else {
      currentGenOption = constrain((GenMenuOption)(currentGenOption + 1), SIGNAL_TYPE, DC_VALUE);
      //if (currentOption == DC_VALUE) currentOption = DC_VALUE;
      //else currentOption = (GenMenuOption)(currentOption + 1);
      //DEBUG_PRINT("Current Option = "); DEBUG_PRINT(currentOption); DEBUG_PRINT("\r\n");
    }
    RotaryEvent = NoEvent;
    NumTurnLeft = 0;
  }
  updateGenMenu(&GMenu);
}
/***********************************************************************/


/**********************************************************************
*       updateGenMenu(): Displays the Generator menu and updates the display
***********************************************************************/
void updateGenMenu(GMenuItems *Gm) {
  String ValString = "";
  //int val_fract, val_int;
  display.clearDisplay();
  display.setCursor(5, 0);
  for (int i = 0; i < GMENU_SIZE; i++) {
    if (i == currentGenOption) {
      switch (GenSelMode) {
        case GenMenuSel:
          display.drawRect(1, i * (8 + ROW_SPACING) + 3, 66, RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);  //Albert: Mai bine inconjuram cu dreptunghi, se vede mai bine
          break;
        case ValueSel:
          if (currentGenOption == FREQUENCY) display.drawRect(66, i * (8 + ROW_SPACING) + 3, 28, RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);  //Albert: La Frecv avem doar 3 cifre de inconjurat
          else display.drawRect(66, i * (8 + ROW_SPACING) + 3, 61 /*50*/, RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);                         //Albert: Inconjuram optiunea aleasa
          break;
        case ValueOrdSel:  //asta poate fi doar la Frequency
          display.drawRect(92, i * (8 + ROW_SPACING) + 3, 31, RECTANGLE_SEL_HEIGHT, SSD1306_WHITE);
          break;
        default: break;
      }
    }
    display.setCursor(5, i * (8 + ROW_SPACING) + ROW_SPACING);  //Albert
    switch (i) {
      case SIGNAL_TYPE:
        {
          ValString = "WaveForm : " + String(WaveText[Gm->Wavetype]);
          display.print(ValString);  // "OFF", "DC", "Sine", "Square1", "Square2","Triangle"
        }
        break;
      case VOLTAGE:
        ValString = "Vampl-pp : " + String(Gm->Amp, 1) + " V";
        display.print(ValString);
        if (Gm->Wavetype == DC) {
          for (int j = 0; j < 50; j = j + 7) {
            display.drawLine(66 + j, i * (8 + ROW_SPACING) + RECTANGLE_SEL_HEIGHT, 66 + j + 10, i * (8 + ROW_SPACING) + 5, SSD1306_WHITE);
          }
        }
        break;
      case FREQUENCY:
        //enum FreqOrders{Hz=0, tHz, hHz, KHz, tKHz, hKHz, MHz, NumFreqOrders};
        switch (Gm->FreqOrder) {
          case Hz:
          case KHz:
          case MHz:
            ValString = "Frequency: " + String(Gm->Freq, 1) + String(" ") + String(FreqText[Gm->FreqOrder]);
            break;
          case tHz:
          case tKHz:
            ValString = "Frequency: " + String(" ") + String(Gm->Freq, 0) + String(" ") + String(FreqText[Gm->FreqOrder]);
            break;
          case hHz:
          case hKHz:
            ValString = "Frequency: " + String(Gm->Freq, 0) + String(" ") + String(FreqText[Gm->FreqOrder]);
            break;
          default: break;
        }
        if (Gm->Wavetype == DC) {
          for (int j = 0; j < 50; j = j + 7) {
            display.drawLine(66 + j, i * (8 + ROW_SPACING) + RECTANGLE_SEL_HEIGHT, 66 + j + 10, i * (8 + ROW_SPACING) + 5, SSD1306_WHITE);
          }
        }

        display.print(ValString);
        break;
      case DC_VALUE:
        ValString = "DC Offset: " + String(Gm->DC, 1) + " V";
        display.print(ValString);
        break;
    }
  }
  display.display();
}

/**********************************************************************
*       adjustPwmValue(): Performs PWM generator parameter adjustment
***********************************************************************/
void adjustPwmValue(PMenuItems *Pm) {
  switch (currentPwmOption) {
    case PSIGNAL_TYPE:
      {
        if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
          Pm->PSignaltype = constrain((PSignalTypes)(Pm->PSignaltype + 1), POff, POn);
          //DEBUG_PRINT("Pwm Wavetype "); DEBUG_PRINT(Pm->PSignaltype);
          flPmenuUpd[0] = 1;  //Turn On/Off PWM gen
          RotaryEvent = NoEvent;
          NumTurnRight = 0;
        }

        if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
          Pm->PSignaltype = (Pm->PSignaltype == POff) ? POff : (PSignalTypes)(Pm->PSignaltype - 1);
          //DEBUG_PRINT("Wavetype "); DEBUG_PRINT(Gm->Wavetype);
          flPmenuUpd[0] = 1;  //update waveform

          RotaryEvent = NoEvent;
          NumTurnLeft = 0;
        }
        updatePwmGen(Pm);
        //oldPosition = newPosition;
        break;
      }
    case PFREQUENCY:
      {
        if (PwmSelMode == PwmValueSel) {  //Adjust frequency value
          if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
            if (IncDecPwmFreq(Pm, inc)) flPmenuUpd[2] = 1;  //update frequency order too
            RotaryEvent = NoEvent;
            NumTurnRight = 0;
            flPmenuUpd[1] = 1;
          }

          if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
            if (IncDecPwmFreq(Pm, dec)) flPmenuUpd[2] = 1;  //update frequency order too
            RotaryEvent = NoEvent;
            NumTurnLeft = 0;
            flPmenuUpd[1] = 1;
          }
        } else if (PwmSelMode == PwmValueOrdSel) {  //Adjust frequency order
          if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
            Pm->FreqOrder = (Pm->FreqOrder == MHz) ? MHz : (FreqOrders)(Pm->FreqOrder + 1);
            UpdPwmFreqfromOrd(Pm);
            RotaryEvent = NoEvent;
            NumTurnRight = 0;
            flPmenuUpd[2] = 1;
            //update the frequency value
            flPmenuUpd[1] = 1;
          }
          if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
            /*Deocamdata scot codul de protectie: Oare la SQW nu e acum frecventa minima de 100 HZ?
        if ( (GMenu.Wavetype == Sqw) && (GMenu.FreqOrder == hHz)){
            GMenu.Freq = 100.0;
            GMenu.FreqOrder = hHz;
        } 
        else*/
            if (Pm->FreqOrder == Hz) {
              //Pm->Freq = 10.0;
              Pm->FreqOrder = Hz;
            } else Pm->FreqOrder = (FreqOrders)(Pm->FreqOrder - 1);
            UpdPwmFreqfromOrd(Pm);
            RotaryEvent = NoEvent;
            NumTurnLeft = 0;
            flPmenuUpd[2] = 1;
            //update the frequency display too
            flPmenuUpd[1] = 1;
          }
        }
        break;
      }
    case PDUTY:
      {
        if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
          Pm->Duty = (Pm->Duty >= 100.0) ? 100.0 : Pm->Duty + 1.0;
          flPmenuUpd[3] = 1;  //update Duty
          RotaryEvent = NoEvent;
          NumTurnRight = 0;
        }

        if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
          Pm->Duty = (Pm->Duty <= 0.0) ? 0.0 : Pm->Duty - 1.0;
          flPmenuUpd[3] = 1;  //update duty
          RotaryEvent = NoEvent;
          NumTurnLeft = 0;
        }
        break;
      }
  }
  updatePwmMenu(Pm);
}

/**********************************************************************
*       adjustGenValue(): Performs Signal generator parameter adjustment
***********************************************************************/
void adjustGenValue(GMenuItems *Gm) {
  switch (currentGenOption) {
    case SIGNAL_TYPE:
      {
        if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
          Gm->Wavetype = constrain((Wavetypes)(Gm->Wavetype + 1), OFF, Sqw1);
          //DEBUG_PRINT("Wavetype "); DEBUG_PRINT(Gm->Wavetype);
          flGmenuUpd[0] = 1;  //update waveform
          flGmenuUpd[3] = 1;  //because it is possible to switch to/from SQW, update amplitude too
          RotaryEvent = NoEvent;
          NumTurnRight = 0;
        }

        if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
          Gm->Wavetype = (Gm->Wavetype == OFF) ? OFF : (Wavetypes)(Gm->Wavetype - 1);
          //DEBUG_PRINT("Wavetype "); DEBUG_PRINT(Gm->Wavetype);
          flGmenuUpd[0] = 1;  //update waveform
          flGmenuUpd[3] = 1;  //because it is possible to switch to/from SQW, update amplitude too
          RotaryEvent = NoEvent;
          NumTurnLeft = 0;
        }
        AD.setWave(signalTypeIndex);
        //oldPosition = newPosition;
        break;
      }
    case VOLTAGE:
      {
        /**********************************************************************
*       AD5443: Changed constraint to 2^12-1 = 4095
***********************************************************************/
        if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {

          Gm->Amp = constrain(Gm->Amp + 0.1, MINVOLTAGE, MAXVOLTAGE);
          PotVal = constrain(PotVal + 1, 0, 4095);

          flGmenuUpd[3] = 1;  //update amplitude
          RotaryEvent = NoEvent;
          NumTurnRight = 0;
        }

        if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
          Gm->Amp = Gm->Amp - 0.1;
          if (Gm->Amp <= MINVOLTAGE) Gm->Amp = MINVOLTAGE;  //deoarece Constrain merge circular

          flGmenuUpd[3] = 1;  //update amplitude
          RotaryEvent = NoEvent;
          NumTurnLeft = 0;
        }
        break;
      }
    case FREQUENCY:
      {
        if (GenSelMode == ValueSel) {  //Adjust frequency value
          if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
            if (IncDecGenFreq(Gm, inc)) flGmenuUpd[2] = 1;  //update frequency order too
            RotaryEvent = NoEvent;
            NumTurnRight = 0;
            //de fapt, flGmenuUpd[1] (=update Frequency val) e redundant,
            //deoarece aacum tot meniul se redeseneaza
            // Il las acolo penru comanda AD9833 - formarea frecventei
            flGmenuUpd[1] = 1;
          }

          if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
            if (IncDecGenFreq(Gm, dec)) flGmenuUpd[2] = 1;  //update frequency order too
            RotaryEvent = NoEvent;
            NumTurnLeft = 0;
            flGmenuUpd[1] = 1;
          }
        } else if (GenSelMode == ValueOrdSel) {  //Adjust frequency order
          if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
            Gm->FreqOrder = (Gm->FreqOrder == MHz) ? MHz : (FreqOrders)(Gm->FreqOrder + 1);
            UpdFreqfromOrd(Gm);
            RotaryEvent = NoEvent;
            NumTurnRight = 0;
            flGmenuUpd[2] = 1;
            //update the frequency value
            flGmenuUpd[1] = 1;
          }
          if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
            /*Deocamdata scot codul de protectie: Oare la SQW nu e acum frecventa minima de 100 HZ?
        if ( (GMenu.Wavetype == Sqw) && (GMenu.FreqOrder == hHz)){
            GMenu.Freq = 100.0;
            GMenu.FreqOrder = hHz;
        } 
        else*/
            if (Gm->FreqOrder == tHz) {
              Gm->Freq = 10.0;
              Gm->FreqOrder = tHz;
            } else Gm->FreqOrder = (FreqOrders)(Gm->FreqOrder - 1);
            UpdFreqfromOrd(Gm);
            RotaryEvent = NoEvent;
            NumTurnLeft = 0;
            flGmenuUpd[2] = 1;
            //update the frequency display too
            flGmenuUpd[1] = 1;
          }
        }
        break;
      }
    case DC_VALUE:
      {
        if ((RotaryEvent == RightTurn) && (NumTurnRight >= NUMTURN_NAV_GRANULARITY)) {
          Gm->DC = constrain(Gm->DC + 0.1, MINDC, MAXDC);
          flGmenuUpd[4] = 1;  //update DC Offset
          RotaryEvent = NoEvent;
          NumTurnRight = 0;
        }

        if ((RotaryEvent == LeftTurn) && (NumTurnLeft >= NUMTURN_NAV_GRANULARITY)) {
          Gm->DC = Gm->DC - 0.1;
          if (Gm->DC <= MINDC) Gm->DC = MINDC;  //deoarece Constrain merge circular
          flGmenuUpd[4] = 1;                    //update DC Offset
          RotaryEvent = NoEvent;
          NumTurnLeft = 0;
        }
        break;
      }
  }
  updateGenMenu(&GMenu);
}

/********************************************************************************
*       UpdPwmFreqfromOrd(): updates PWM frequency value when frequency order changes
********************************************************************************/
void UpdPwmFreqfromOrd(PMenuItems *Pm) {
  if (PwmFreqOrd_Prev < Pm->FreqOrder) {  //Order increased
    switch (Pm->FreqOrder) {
      case tHz:
      case hHz:
      case tKHz:
      case hKHz:
        Pm->Freq = Pm->Freq * 10.0;
        break;
      case KHz:
        Pm->Freq = Pm->Freq / 100.0;
        break;
      case MHz:
        if (Pm->Freq >= 200.0) Pm->Freq = 2.0;
        else Pm->Freq = Pm->Freq / 100.0;
        break;
      default: break;
    }
  } else if (PwmFreqOrd_Prev > Pm->FreqOrder) {  //Order Decreased
    switch (Pm->FreqOrder) {
      case Hz:
      case tHz:
      case KHz:
      case tKHz:
        Pm->Freq = Pm->Freq / 10.0;
        break;
      case hHz:
      case hKHz:
        Pm->Freq = Pm->Freq * 100.0;
      default: break;
    }
  }
  PwmFreqOrd_Prev = Pm->FreqOrder;
}

/*******************************************************************************************************
*       IncDecPwmFreq(): Increments/decrements generator frequency value, taking accound the order and the limits
*********************************************************************************************************/
uint8_t IncDecPwmFreq(PMenuItems *Pm, enum IncOrDec I_d) {
  float fFreq = Pm->Freq;
  enum FreqOrders fFreqOrder = Pm->FreqOrder;
  uint8_t UpdFreqOrd = 0;
  if (I_d == inc) {  //Increment
    if (fFreqOrder == MHz) {
      if (fFreq >= 2.0) fFreq = 2.0;  //upper frequency limit
      else fFreq = fFreq + 0.1;
    } else {
      if (fFreq >= 990.0) {  //this can be only at hHz or hKHz
        fFreq = 1.0;
        fFreqOrder = (FreqOrders)(fFreqOrder + 1);
        UpdFreqOrd = 1;
      } else if ((fFreq >= 100.0) && (fFreq < 990.0)) fFreq = fFreq + 10.0;
      else if (fFreq >= 99.0) {  //this can be only at tHz or tKHz
        fFreq = 100.0;
        fFreqOrder = (FreqOrders)(fFreqOrder + 1);
        UpdFreqOrd = 1;
      } else if ((fFreq >= 10.0) && (fFreq < 99.0)) fFreq = fFreq + 1.0;
      else if (fFreq >= 9.9) {  //this can be only at Hz or KHz
        fFreq = 10.0;
        fFreqOrder = (FreqOrders)(fFreqOrder + 1);
        UpdFreqOrd = 1;
      } else fFreq = fFreq + 0.1;
    }
  } else {  //Decrement
    if (fFreqOrder == Hz) {
      if (fFreq <= 1.0) fFreq = 1.0;  //lower frequency limit
      else fFreq = fFreq - 0.1;
      //}
    } else {
      if (fFreq <= 1.0) {  //this can be only at KHz or MHz
        fFreq = 990.0;
        fFreqOrder = (FreqOrders)(fFreqOrder - 1);
        UpdFreqOrd = 1;
      } else if ((fFreq >= 1.0) && (fFreq <= 9.9)) fFreq = fFreq - 0.1;
      else if (fFreq <= 10.0) {  //this can be only at tHz or tKHz
        fFreq = 9.9;
        fFreqOrder = (FreqOrders)(fFreqOrder - 1);
        UpdFreqOrd = 1;
      } else if ((fFreq >= 10.0) && (fFreq <= 99.0)) fFreq = fFreq - 1.0;
      else if (fFreq <= 100.0) {  //this can be only at hHz or hKHz
        fFreq = 99.0;
        fFreqOrder = (FreqOrders)(fFreqOrder - 1);
        UpdFreqOrd = 1;
      } else fFreq = fFreq - 10.0;
    }
  }

  if (UpdFreqOrd) UpdPwmFreqfromOrd(Pm);

  Pm->FreqOrder = fFreqOrder;
  PwmFreqOrd_Prev = fFreqOrder;
  Pm->Freq = fFreq;
  return UpdFreqOrd;
}


/********************************************************************************
*       UpdFreqfromOrd(): updates frequency value when frequency order changes
********************************************************************************/
void UpdFreqfromOrd(GMenuItems *Gm) {
  if (FreqOrd_Prev < Gm->FreqOrder) {  //Order increased
    switch (Gm->FreqOrder) {
      case hHz:
      case tKHz:
      case hKHz:
        Gm->Freq = Gm->Freq * 10.0;
        break;
      case KHz:
        Gm->Freq = Gm->Freq / 100.0;
        break;
      case MHz:
        if (Gm->Freq >= 200.0) Gm->Freq = 2.0;
        else Gm->Freq = Gm->Freq / 100.0;
        break;
      default: break;
    }
  } else if (FreqOrd_Prev > Gm->FreqOrder) {  //Order Decreased
    switch (Gm->FreqOrder) {
      case tHz:
      case KHz:
      case tKHz:
        Gm->Freq = Gm->Freq / 10.0;
        break;
      case hHz:
      case hKHz:
        Gm->Freq = Gm->Freq * 100.0;
      default: break;
    }
  }
  FreqOrd_Prev = Gm->FreqOrder;
}





/*******************************************************************************************************
*       IncDecGenFreq(): Increments/decrements generator frequency value, taking accound the order and the limits
*********************************************************************************************************/
uint8_t IncDecGenFreq(GMenuItems *Gm, enum IncOrDec I_d) {
  float fFreq = Gm->Freq;
  enum FreqOrders fFreqOrder = Gm->FreqOrder;
  uint8_t UpdFreqOrd = 0;
  if (I_d == inc) {  //Increment
    if (fFreqOrder == MHz) {
      if (fFreq >= 2.0) fFreq = 2.0;  //upper frequency limit
      else fFreq = fFreq + 0.1;
    } else {
      if (fFreq >= 990.0) {  //this can be only at hHz or hKHz
        fFreq = 1.0;
        fFreqOrder = (FreqOrders)(fFreqOrder + 1);
        UpdFreqOrd = 1;
      } else if ((fFreq >= 100.0) && (fFreq < 990.0)) fFreq = fFreq + 10.0;
      else if (fFreq >= 99.0) {  //this can be only at tHz or tKHz
        fFreq = 100.0;
        fFreqOrder = (FreqOrders)(fFreqOrder + 1);
        UpdFreqOrd = 1;
      } else if ((fFreq >= 10.0) && (fFreq < 99.0)) fFreq = fFreq + 1.0;
      else if (fFreq >= 9.9) {  //this can be only at Hz or KHz
        fFreq = 10.0;
        fFreqOrder = (FreqOrders)(fFreqOrder + 1);
        UpdFreqOrd = 1;
      } else fFreq = fFreq + 0.1;
    }
  } else {  //Decrement
    if (fFreqOrder == tHz) {
      //if ((Gm->Wavetype == Sin) || (Gm->Wavetype == Tri) ){//scot codul de protectie: Parca la SQW limita e 10 HZ?
      if (fFreq <= 10.0) fFreq = 10.0;  //lower frequency limit
      else fFreq = fFreq - 1.0;
      //}
    }
    /*else if ( (fFreqOrder == hHz) && (Gm->Wavetype == Sqw) ){ //Cod de protectie scos, vezi mai sus
          if (fFreq <=100.0) fFreq = 100.0; //lower frequency limit
          else fFreq = fFreq - 10.0;      
      }*/
    else {
      if (fFreq <= 1.0) {  //this can be only at KHz or MHz
        fFreq = 990.0;
        fFreqOrder = (FreqOrders)(fFreqOrder - 1);
        UpdFreqOrd = 1;
      } else if ((fFreq > 1.0) && (fFreq <= 9.9)) fFreq = fFreq - 0.1;
      else if (fFreq <= 10.0) {  //this can be only at tHz or tKHz
        fFreq = 9.9;
        fFreqOrder = (FreqOrders)(fFreqOrder - 1);
        UpdFreqOrd = 1;
      } else if ((fFreq > 10.0) && (fFreq <= 99.0)) fFreq = fFreq - 1.0;
      else if (fFreq <= 100.0) {  //this can be only at hHz or hKHz
        fFreq = 99.0;
        fFreqOrder = (FreqOrders)(fFreqOrder - 1);
        UpdFreqOrd = 1;
      } else fFreq = fFreq - 10.0;
    }
  }

  if (UpdFreqOrd) UpdFreqfromOrd(Gm);

  Gm->FreqOrder = fFreqOrder;
  FreqOrd_Prev = fFreqOrder;
  Gm->Freq = fFreq;
  return UpdFreqOrd;
}

/********************************************
* PWM Generator Functions
********************************************/


void updatePwmGen(PMenuItems *Pm) {
  uint8_t index = 0;
  float Frequency = 0.0;
  float slPeriod = 0.0;
  float slDutyVal = 50.0;
  float Duty = 0.0;
  for (index = 0; index < 4; index++) {
    if (flPmenuUpd[index]) {
      switch (index) {
        case 0:  //UpdateOn/Off
          if (Pm->PSignaltype == POn) objPWMD9.resume();
          else objPWMD9.suspend();
          break;
        case 1:
        case 2:  //Update frequency value from frequency or freq order change
          Frequency = updatePwmFrequency(Pm);
          objPWMD9.end();
          objPWMD9.begin(Frequency, Pm->Duty);
          break;
        case 3:  //update duty
          Duty = Pm->Duty;
          objPWMD9.pulse_perc(Duty);
          break;

        default: break;
      }
      //Clear flag
      //DEBUG_PRINT("clearing flGmenuUpd[");DEBUG_PRINT(index); DEBUG_PRINT("]="); DEBUG_PRINT(flGmenuUpd[index]); DEBUG_PRINT("\r\n");
      flPmenuUpd[index] = 0;
    }
  }

  /*
    if(Pm->PSignaltype == POn){
      objPWMD9.begin(Frequency, Duty);
    }
    else
      objPWMD9.end();
*/
}


float updatePwmFrequency(PMenuItems *Pm) {
  float Frequency = 0.0;
  uint8_t FreqMagnOrder;

  switch (Pm->FreqOrder) {
    case Hz: FreqMagnOrder = 0; break;
    case tHz: FreqMagnOrder = 0; break;
    case hHz: FreqMagnOrder = 0; break;
    case KHz: FreqMagnOrder = 3; break;
    case tKHz: FreqMagnOrder = 3; break;
    case hKHz: FreqMagnOrder = 3; break;
    case MHz: FreqMagnOrder = 6; break;
    default: FreqMagnOrder = 0; break;
  }

  Frequency = Pm->Freq * (powf(10, FreqMagnOrder));

  return Frequency;
}

/********************************************
* AD9833 Generator Functions
********************************************/
void updateGenAndPot(GMenuItems *Gm) {
  uint8_t index = 0;
  for (index = 0; index < 6; index++) {
    if (flGmenuUpd[index]) {
      switch (index) {
        case 0:  //Update wave type
                 //DEBUG_PRINT("Updating Wavegen"); DEBUG_PRINT("\r\n");
          UpdateAD9833Wave(Gm);
          break;
        case 1:  //Update frequency value
          UpdateAD9833Frequency(Gm);
          break;
        case 2:  //update frequency order
          UpdateAD9833Frequency(Gm);
          break;
        case 3:  //update amplitude
          UpdateGenAmp(Gm);
          break;
        case 4:  //update DC offset
          //DEBUG_PRINT("Updating DC"); DEBUG_PRINT("\r\n");
          UpdateGenDC(Gm);
          break;
        //in V 3.0 is not used:
        /*
        case 5://update On/Off
              UpdateAD9833Wave(Gm);
               break;
        */
        default: break;
      }
      //Clear flag
      //DEBUG_PRINT("clearing flGmenuUpd[");DEBUG_PRINT(index); DEBUG_PRINT("]="); DEBUG_PRINT(flGmenuUpd[index]); DEBUG_PRINT("\r\n");
      flGmenuUpd[index] = 0;
    }
  }
}

void UpdateAD9833Frequency(GMenuItems *Gm) {
  float Frequency = 0.0;
  uint8_t FreqMagnOrder;

  switch (Gm->FreqOrder) {
    case Hz: FreqMagnOrder = 0; break;
    case tHz: FreqMagnOrder = 0; break;
    case hHz: FreqMagnOrder = 0; break;
    case KHz: FreqMagnOrder = 3; break;
    case tKHz: FreqMagnOrder = 3; break;
    case hKHz: FreqMagnOrder = 3; break;
    case MHz: FreqMagnOrder = 6; break;
    default: FreqMagnOrder = 0; break;
  }

  Frequency = Gm->Freq * (powf(10, FreqMagnOrder));

  AD.setFrequency(Frequency, 0);
}

void UpdateAD9833Wave(GMenuItems *Gm) {
  uint8_t Update_Wave = 1, Wavegen_Mode = AD9833_OFF;

  if ((Gm->Wavetype == OFF) || (Gm->Wavetype == DC)) Wavegen_Mode = AD9833_OFF;
  else if (Gm->Wavetype == Sin) Wavegen_Mode = AD9833_SINE;
  else if (Gm->Wavetype == Tri) Wavegen_Mode = AD9833_TRIANGLE;
  else if (Gm->Wavetype == Sqw1) Wavegen_Mode = AD9833_SQUARE1;

  //Comment this to leave the generator running  if OnOff = 1
  //else if (Gm->Wavetype == PWM) Wavegen_Mode = MD_AD9833::MODE_OFF;
  //else if (Gm->Wavetype == Ext) Wavegen_Mode = MD_AD9833::MODE_OFF;
  else Update_Wave = 0;

  //DEBUG_PRINT("Update Wawe:");  DEBUG_PRINT(String(Update_Wave)); DEBUG_PRINT("\r\n");
  //DEBUG_PRINT("Wavegen Mode:");  DEBUG_PRINT(String(Wavegen_Mode)); DEBUG_PRINT("\r\n");

  if (Update_Wave) AD.setWave(Wavegen_Mode);
}


uint16_t WriteAD5443(uint16_t Data) {

  uint16_t WData = Data & 0xFFF;  //can be max 4095

  uint16_t WDacData = 0, RDacData = 0;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  WDacData = DAC_5443_LOAD_UPDATE | WData;
  //  DEBUG_PRINT("First Command to send: Load and Update ");  DEBUG_PRINT(String(WDacData, HEX)); DEBUG_PRINT("\r\n");
  digitalWrite(AD5443_SYNCN, LOW);
  RDacData = SPI.transfer16(WDacData);
  digitalWrite(AD5443_SYNCN, HIGH);
  //  DEBUG_PRINT("First Data received: ");  DEBUG_PRINT(String(RDacData, HEX)); DEBUG_PRINT("\r\n");
  //delayMicroseconds(100);

  WDacData = DAC_5443_INIT_READBACK | 0x00;
  //DEBUG_PRINT("Second Command to send: read RDAC: ");  DEBUG_PRINT(String(WDacData, HEX)); DEBUG_PRINT("\r\n");
  digitalWrite(AD5443_SYNCN, LOW);
  RDacData = SPI.transfer16(WDacData);
  digitalWrite(AD5443_SYNCN, HIGH);
  //DEBUG_PRINT("Second  data received: ");  DEBUG_PRINT(String(RDacData, HEX)); DEBUG_PRINT("\r\n");
  //delayMicroseconds(100);

  WDacData = DAC_5443_NOP;
  // DEBUG_PRINT("Sending NOP (0x0000) - read RDAC :  ");  DEBUG_PRINT(String(WDacData, HEX)); DEBUG_PRINT("\r\n");
  digitalWrite(AD5443_SYNCN, LOW);
  RDacData = SPI.transfer16(WDacData);
  digitalWrite(AD5443_SYNCN, HIGH);
  // DEBUG_PRINT("Third Data Received - read RDAC :  ");  DEBUG_PRINT(String(RDacData, HEX)); DEBUG_PRINT("\r\n");

  return RDacData;
}

/********************************************
* Update Amplitude function
********************************************/
void UpdateGenAmp(GMenuItems *Gm) {
  float fLSBval = 0.0;
  uint16_t LSBVal;

  uint16_t AmplData;
  uint16_t ReadData;

  if ((Gm->Wavetype == Sin) || (Gm->Wavetype == Tri)) {
    fLSBval = (Gm->Amp / SinLGainVperLsb);
    LSBVal = round(double(fLSBval));
  } else if (Gm->Wavetype == Sqw1) {
    fLSBval = (Gm->Amp / SQWLGainVperLsb);
    LSBVal = round(double(fLSBval));
  }


  //ReadData = WritePot(LSBVal);
  ReadData = WriteAD5443(LSBVal);

  //DEBUG_PRINT("RDAC data received (0x100): ");  DEBUG_PRINT(String(RecvPotData, HEX)); DEBUG_PRINT("\r\n");
}

/*********************************************************************
* Offset DAC access function
* Assumed: DAC is set up with internal reference and automatic LDAC
* (see setup() )
*********************************************************************/
uint16_t WriteOffDACData(uint16_t Data, uint8_t Channel) {
  uint8_t i;

  //Turn on internal reference
  OffDacData[0] = (CMD_WRITE_UPDATE_N << 3) | (Channel & 0x07);
  OffDacData[1] = (Data & 0x0FF0) >> 4;  //bitii 11..4
  OffDacData[2] = (Data & 0x0F) << 4;    //bitii 3..0, aliniat la stanga, ultimii 4 biti:0
  Wire.beginTransmission(AD5625_I2C_ADDR);
  Wire.write(OffDacData, 3);
  i = Wire.endTransmission();
  //Check DAC communication
  //DEBUG_PRINT("AD562R Offset DAC response after setting DAC value= ");
  //DEBUG_PRINT(i); DEBUG_PRINT("\r\n");
}



void UpdateGenDC(GMenuItems *Gm) {
  //Output is (DAC_A-DACB) * 0.36 * 7.2
  //For positive outputs, 2048+OffDACVal/2 goes to DAC A, 2048-OffDACVal/2 goes to DAC_B
  //For negative outputs, 2048+OffDACVal/2 goes to DAC B, 2048-OffDACVal/2 goes to DAC_A
  //We send LSBDC_DAC_A and LSBDC_DAC_B values to the DAC channels, respectively
  //LSBOFF is the offset correction LSB

  int DC_LSB = 0;

  uint16_t LSBDC_DAC_A = 0, LSBDC_DAC_B = 0;

  float fLSBval = 0.0;


  fLSBval = (Gm->DC / DcOffperLSB) + LSBOFF;
  DC_LSB = round(double(fLSBval));

  //DEBUG_PRINT("DC_LSB = ");  DEBUG_PRINT(DC_LSB); DEBUG_PRINT("\r\n");

  if ((DC_LSB % 2) == 0) {
    LSBDC_DAC_A = 2048 + DC_LSB / 2;
    LSBDC_DAC_B = 2048 - DC_LSB / 2;
  } else {  //even: Only one channel will be increased/decreased
    if (DC_LSB >= 0) {
      LSBDC_DAC_A = 2048 + DC_LSB / 2 + 1;
      LSBDC_DAC_B = 2048 - DC_LSB / 2;
    } else {
      LSBDC_DAC_A = 2048 + DC_LSB / 2;
      LSBDC_DAC_B = 2048 - DC_LSB / 2 + 1;
    }
  }


  /*
    if (fLSBval >= 0){
        LSBDC_DAC_A = 2048 + trunc(double(fLSBval/2));
        LSBDC_DAC_B = 2048 - trunc(double(fLSBval/2));
    }
    else{
        LSBDC_DAC_A = 2048 - abs(trunc(double(fLSBval/2)));
        LSBDC_DAC_B = 2048 + abs(trunc(double(fLSBval/2)));
    }
*/

  //DEBUG_PRINT("Writing OFF DAC A:");  DEBUG_PRINT(LSBDC_DAC_A); DEBUG_PRINT("\r\n");
  //DEBUG_PRINT("Writing OFF DAC B:");  DEBUG_PRINT(LSBDC_DAC_B); DEBUG_PRINT("\r\n");

  WriteOffDACData(LSBDC_DAC_A, ADDR_DAC_A);
  WriteOffDACData(LSBDC_DAC_B, ADDR_DAC_B);
}


/********************************************************************************
*       Rotary Encoder initialization
********************************************************************************/
void InitializeRotaryEncoder() {

  versatile_encoder = new Versatile_RotaryEncoder(clk, dt, sw);
  // Load to the encoder all nedded handle functions here (up to 9 functions)
  versatile_encoder->setHandleRotate(handleRotate);
  versatile_encoder->setHandlePressRotate(handlePressRotate);
  //versatile_encoder->setHandleHeldRotate(handleHeldRotate);
  versatile_encoder->setHandlePress(handlePress);
  versatile_encoder->setHandlePressRelease(handlePressRelease);
  versatile_encoder->setHandleLongPress(handleLongPress);
  // versatile_encoder->setHandleLongPressRelease(handleLongPressRelease);
  versatile_encoder->setHandlePressRotateRelease(handlePressRotateRelease);
  versatile_encoder->setHandleHeldRotateRelease(handleHeldRotateRelease);

  RotaryEvent = NoEvent;
  NumTurnRight = 0;
  NumTurnLeft = 0;
}

/********************************************************************************
*       These are callback functions for Rotary Encoder events
********************************************************************************/

void handleRotate(int8_t rotation) {
  if (rotation > 0) {
    NumTurnRight++;
    RotaryEvent = RightTurn;
    // Serial.print("Rot R"); Serial.println(NumTurnRight);
  } else {
    NumTurnLeft++;
    RotaryEvent = LeftTurn;
    //  Serial.print("Rot R"); Serial.println(NumTurnRight);
  }
}

void handlePressRotate(int8_t rotation) {
  handleRotate(rotation);
}

void handleHeldRotate(int8_t rotation) {
  handleRotate(rotation);
}

void handlePress() {
  RotaryEvent = Press;
}

void handlePressRelease() {
  //Serial.println("#5 Press released");
}

void handleLongPress() {
  //Serial.println("#6 Long pressed");
  currentMenuOption = Main;
  flCalMenuFirstRun = 1;  //the calibration menu will be animated again
  updateMainMenu();
}

void handleLongPressRelease() {
  //Serial.println("#7 Long press released");
}

void handlePressRotateRelease() {
  //Serial.println("#8 Press rotate released");
}

void handleHeldRotateRelease() {
  //Serial.println("#9 Held rotate released");
}
