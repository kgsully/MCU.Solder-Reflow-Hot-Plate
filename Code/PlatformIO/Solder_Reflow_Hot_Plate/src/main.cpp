/* 
Solder Reflow Hot Plate Controller
by Ken S. (03/2023)

Code for an ATMEGA328P-AU based design for a solder reflow hot plate to be used for soldering SMD components onto custom PCBs. 
The microcontroller uses 2 PID loops to individually control the temperature for 2 aluminum heating plates. 
An OLED display and rotary encoder with push button allow the user to configure various parameters 
(reflow profile, PID tuning constants for the hot plate PID loops, etc), save the configuration to the microcontroller EEPROM memory, 
and start the unit in either constant temperature mode, or reflow profile mode.

THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
*/

#include <Arduino.h> 
#include <Wire.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <U8g2lib.h>

// Definitions for the rotary encoder
#define encCLK_inp 2
#define encDT_inp 3
#define encSW_inp 4
#define readCLK bitRead(PIND, 2)  //faster than digitalRead()
#define readDT bitRead(PIND, 3)   //faster than digitalRead()

// Definitions & Variables for the Thermistors
#define THERMISTORPIN1 A0          // which analog pin to connect
#define THERMISTORPIN2 A1          // which analog pin to connect
#define THERMISTORNOMINAL1 120000  // resistance at 25 degrees C
#define THERMISTORNOMINAL2 120000  // resistance at 25 degrees C
#define TEMPERATURENOMINAL1 25     // temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL2 25     // temp. for nominal resistance (almost always 25 C)
#define BCOEFFICIENT1 3950         // The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT2 3950         // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR1 100000     // the value of the 'other' resistor
#define SERIESRESISTOR2 100000     // the value of the 'other' resistor
#define Numsamples 5               // how many samples1 to take and average, more takes longer but is more 'smooth'

#define pwmPin1 5  // PWM Output Pin for Hotplate 1 Control
#define pwmPin2 6  // PWM Output Pin for Hotplate 1 Control

int samples1[Numsamples];
int samples2[Numsamples];
double steinhart1 = 0.0;    // Thermistor Temperature Converted Value (deg C)
double steinhart2 = 0.0;    // Thermistor Temperature Converted Value (deg C)
double T1Disp = 0.0;        // Running T1 Temperature Display (update at running timer interval)
double T2Disp = 0.0;        // Running T2 Temperature Display (update at running timer interval)
double thermistor1Buffer = 0.0;   // Thermistor 1 temperature value buffer
double thermistor2Buffer = 0.0;   // Thermistor 2 temperature value buffer
bool thermistor1Fail = 0;   // Thermistor 1 Failure Flag
bool thermistor2Fail = 0;   // Thermistor 2 Failure Flag

// Rotary Encoder Operation Variables
volatile int tempCounter = 0;
volatile int menuCounter = 1;
int protectedMenuCounter = 1;
int previousMenuCounter = 0;
volatile int selectCounter = 0;
int protectedSelectCounter = 0;
int previousSelectCounter = 0;
bool encSW;

// Definitions for Menu Structure
uint8_t menuIndex = 0;         // Initialize to 0, or Main menu
uint8_t selectIndex = 0;       // Initialize to 0, or 1st option
uint8_t selectIndexMax = 1;    // Create variable for max selection index and initialize to 1 item
bool running = 0;              // Running state flag
bool reflowParamSelected = 0;  // Reflow menu parameter selected for edit flag
bool pidParamSelected = 0;     // PID menu parameter selected for edit flag
bool selectFlag = 0;
bool startConfirm = 0;
uint8_t curPos[2] = { 0, 0 };

// Process Variables & default values - Based upon MG Chemicals 4902P Sn42Bi57Ag1 Low Temperature Solder Paste T3
uint8_t wrkInt = 0;
double wrkDouble = 0.0;
uint8_t parametersReflow[7] = { 115, 100, 145, 155, 185, 180, 35 };  // T1, t1, T2, t2, T3, t3, Reflow Duration
double parametersPID[6] = { 3.30, 0.02, 3.45, 3.30, 0.02, 3.45 };  // Kp1, Ki1, Kd1, Kp2, Ki2, Kd2

// EEPROM Intermediate Variables
uint8_t parametersReflowREAD[7] = {0, 0, 0, 0, 0, 0, 0};
int parametersPIDREAD[6] = {0, 0, 0, 0, 0, 0};
int parametersPIDint[6] = {0, 0, 0, 0, 0, 0};

// PID Variables
double pid_Setpoint;
double pid1_Input, pid1_Output;  // Define PID Variables Loop 1
double pid2_Input, pid2_Output;  // Define PID Variables Loop 2

// Running Execution Variables
bool runningBuffer = 0;
uint8_t runningState = 0;        // States: 1 = RAMP, 2 = SOAK, 3 = REFLOW RAMP, 4 = REFLOW, 5 = COOLING / COMPLETE
unsigned long time_now = 0;
int runningSecondCounter = 0;
double initTempSnapshot = 25.0;
uint8_t constTempSP = 35;       // Constant Temp Mode Temperature SP
bool runningMode = 0;           // Run Mode variable: 0 = CONSTANT TEMP MODE, 1 = REFLOW PROFILE MODE, 


// Create PID Object(s)
PID hotPlate1PID(&pid1_Input, &pid1_Output, &pid_Setpoint, parametersPID[0], parametersPID[1], parametersPID[2], DIRECT);  // Set Proportional on Measurement (P on Error is default), and Direct acting
PID hotPlate2PID(&pid2_Input, &pid2_Output, &pid_Setpoint, parametersPID[3], parametersPID[4], parametersPID[5], DIRECT);  // Set Proportional on Measurement (P on Error is default), and Direct acting

// Create u8g2 object
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// -----------------------------------------------------------
// Interrupt handling routines for rotary encoder
// -----------------------------------------------------------
void isrEncCLK() {
  if (readDT != readCLK) {
    tempCounter++;
  } else {
    tempCounter--;
  }

  if (selectFlag == 1) {
    selectCounter += tempCounter;
  } else {
    if ((tempCounter > 0) && (menuCounter < selectIndexMax)) {
      menuCounter++;
    } else if ((tempCounter < 0) && (menuCounter > 1)) {
      menuCounter--;
    }
  }
  tempCounter = 0;
}

void isrEncDT() {
  if (readCLK == readDT) {
    tempCounter++;
  } else {
    tempCounter--;
  }

  if (selectFlag == 1) {
    selectCounter += tempCounter;
  } else {
    if ((tempCounter > 0) && (menuCounter < selectIndexMax)) {
      menuCounter++;
    } else if ((tempCounter < 0) && (menuCounter > 1)) {
      menuCounter--;
    }
  }
  tempCounter = 0;
}

// -----------------------------------------------------------
// EEPROM Read / Write handling routines
// -----------------------------------------------------------
void writeUInt8TArrayIntoEEPROM(int address, uint8_t numbers[], int arraySize) {
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++) 
  {
    EEPROM.update(addressIndex, numbers[i]);   // Does not need to be bit shifted as the uint8_t data type takes only 1 byte
    addressIndex ++;
    delay(10); // delay for EEPROM
  }
}

void writeIntArrayIntoEEPROM(int address, int numbers[], int arraySize) {
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++) 
  {
    EEPROM.update(addressIndex, numbers[i] >> 8);
    EEPROM.update(addressIndex + 1, numbers[i] & 0xFF);
    addressIndex += 2;
    delay(10); // delay for EEPROM
  }
}

void readUInt8TArrayFromEEPROM(int address, uint8_t numbers[], int arraySize) {
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++)
  {
    numbers[i] = (EEPROM.read(addressIndex));  // Does not need to be bit shifted as the uint8_t data type takes only 1 byte
    addressIndex ++;
    delay(10);
  }
}

void readIntArrayFromEEPROM(int address, int numbers[], int arraySize) {
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++)
  {
    numbers[i] = (EEPROM.read(addressIndex) << 8) + EEPROM.read(addressIndex + 1);
    addressIndex += 2;
    delay(10);
  }
}

// -----------------------------------------------------------
// Parameter Calculations
// -----------------------------------------------------------
void calcParameters() {

  if (menuIndex == 3) {     // Reflow Profile
    wrkInt = selectCounter + parametersReflow[menuCounter - 1];

    if (encSW) {
      parametersReflow[menuCounter - 1] = wrkInt;
      selectCounter = 0;
      wrkInt = 0;
      wrkDouble = 0.0;
    }
  }

  if (menuIndex == 4) {     // PID Parameters
    wrkDouble = ((float)selectCounter * 0.01) + parametersPID[menuCounter - 1];
    if (wrkDouble < 0 ) {
      wrkDouble = 0;
    }

    if (encSW) {
      parametersPID[menuCounter - 1] = wrkDouble;
      selectCounter = 0;
      wrkInt = 0;
      wrkDouble = 0.0;
    }
  }

  if (menuIndex == 98) {     // Running - Const Temp SP
    wrkInt = selectCounter + constTempSP;

    if (encSW) {
      constTempSP = wrkInt;
      selectCounter = 0;
      wrkInt = 0;
      wrkDouble = 0.0;
    }
  }
}

void pidLoop1() {
  pid1_Input = steinhart1;
  hotPlate1PID.Compute();
  analogWrite(pwmPin1, pid1_Output);
} 

void pidLoop2() {
  pid2_Input = steinhart2;
  hotPlate2PID.Compute();
  analogWrite(pwmPin2, pid2_Output);
} 

// -----------------------------------------------------------
// Display, Menu Navigation, & Encoder Selection Handling
// -----------------------------------------------------------
void updateCursorPosition() {
  int i = 0;

  switch (menuIndex) {
    curPos[0] = 0;
    curPos[1] = 0;
    case 0:  // MAIN MENU
      curPos[0] = 0;
      if (menuCounter == 1) {
        curPos[1] = 19;
      } else if (menuCounter == 2) {
        curPos[1] = 27;        
      } else {
        curPos[1] = 50; 
      }
      if (encSW) {
        if (menuCounter == 1) {           // Start Reflow Selected
          runningMode = 1;
          startConfirm = 1;
          menuIndex = 1;
        } else if (menuCounter == 2) {    // Start Const. Temp Selected
          runningMode = 0;
          startConfirm = 1;
          menuIndex = 1;
        } else if (menuCounter == 3) {
          menuIndex = 2;
        }
        menuCounter = 1;
      }
      break;
    case 1:  // CONFIRM DIALOG
      if (menuCounter == 1) {
        curPos[0] = 29;  curPos[1] = 45;
      } else {
        curPos[0] = 69;  curPos[1] = 45;
      }
      if (encSW) {
        if (menuCounter == 1) {     // ----- NO Selection -----
          if (startConfirm == 1) {  // If Start Confirm True, profile is NOT running, selecing 'NO' would fall back to main menu
            running = 0;
            menuIndex = 0;
            menuCounter = 1;
            startConfirm = 0;
          } else {                  // If Start Confirm False, profile IS running, selecing 'NO' would fall back to running screen to continue running
            running = 1;            
            if (runningMode == 1) {
              menuIndex = 99;
            } else {
              menuIndex = 98;
            }
            menuCounter = 1;
          }
        }
        if (menuCounter == 2) {     // ----- YES SELECTION -----
          if (startConfirm == 1) {  // if Start Confirm True, selecting 'Yes' would START running the profile
            running = 1; 
            if (runningMode == 1) {
              menuIndex = 99;
            } else {
              menuIndex = 98;
            }
            menuCounter = 1;
            startConfirm = 0;
          } else {                  // if Start Confirm False, profile is running. Selecting 'Yes' would STOP running the profile
            running = 0;            
            menuIndex = 0;
            menuCounter = 1;
          }
        }
      }
      break;
    case 2:  // CONFIGURATION MENU
      curPos[0] = 0;
      switch (menuCounter) {
        case 1:
          curPos[1] = 19;
          break;
        case 2:
          curPos[1] = 27;
          break;
        case 3:
          curPos[1] = 43;
          break;
        case 4:
          curPos[1] = 64;
          break;
      }
      if (encSW) {
        if (menuCounter == 4) {           // Back selection - Return to Main Menu
          menuIndex = 0;
          menuCounter = 1;
        } else {
          menuIndex = menuCounter + 2;    // Offset selection by 2
          menuCounter = 1;
        }
      }
      break;
    case 3:  // REFLOW PROFILE
      if ((menuCounter < 8 && menuCounter % 2 == 1) || menuCounter == 8) {
        curPos[0] = 0;
      } else if (menuCounter < 8 && menuCounter % 2 == 0) {
        curPos[0] = 66;
      }
      switch (menuCounter) {
        case 1: curPos[1] = 19; break;  // T1
        case 2: curPos[1] = 19; break;  // t1
        case 3: curPos[1] = 29; break;  // T2
        case 4: curPos[1] = 29; break;  // t2
        case 5: curPos[1] = 39; break;  // T3
        case 6: curPos[1] = 39; break;  // t3
        case 7: curPos[1] = 49; break;  // Reflow Duration
        case 8: curPos[1] = 64; break;  // Back
      }
      if (encSW) {
        if (menuCounter == 8) {
          menuIndex = 2;                // Return to Config Menu
          menuCounter = 1;
        } else {
          selectFlag = !selectFlag;
        }
      }
      break;
    case 4:  //  PID Loop Tuning
      switch (menuCounter) {
          case 1:  curPos[0] = 0;  curPos[1] = 19; break;  // Kp1
          case 2:  curPos[0] = 0;  curPos[1] = 29; break;  // Ki1
          case 3:  curPos[0] = 0;  curPos[1] = 39; break;  // Kd1
          case 4:  curPos[0] = 66; curPos[1] = 19; break;  // Kp2
          case 5:  curPos[0] = 66; curPos[1] = 29; break;  // Ki2
          case 6:  curPos[0] = 66; curPos[1] = 39; break;  // Kd2
          case 7:  curPos[0] = 0;  curPos[1] = 64; break;  // Back
      }
      if (encSW) {
        if (menuCounter == 7) {
          menuIndex = 2;              // Return to Config Menu
          menuCounter = 1;
        } else {
          selectFlag = !selectFlag;
        }
      }
      break;
    case 5:   //  Save Configuration
      writeUInt8TArrayIntoEEPROM(1, parametersReflow, 7);   // Write Reflow Paramter Data to EEPROM
      for (i = 0; i < 6; i++) {                             // Convert PID paramters to INT for storage
        parametersPIDint[i] = parametersPID[i] * 100;
      }
      writeIntArrayIntoEEPROM(8, parametersPIDint, 6);      // Write PID Parameter Data to EEPROM
      delay(3000);
      menuIndex = 2;                  // Return to Config Menu
      menuCounter = 1;
      break;
    case 98:  //  Running - Constant Temp Mode
      curPos[0] = 0; 
      if (menuCounter == 1) {
        curPos[1] = 48;
      } else {
        curPos[1] = 64;
      }
      if (encSW) {
        if (menuCounter == 2) {
          startConfirm = 0;
          menuIndex = 1;
          menuCounter = 1;
        } else {
          selectFlag = !selectFlag;
        }
      }   
      break;
    case 99:  //  Running - Reflow Profile Mode
      curPos[0] = 0;
      curPos[1] = 56;
      if (encSW) {
        startConfirm = 0;
        menuIndex = 1;
        menuCounter = 1;
      }    
      break;
  }
}

void updateDisplay() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont11_tr);
    u8g2.setFontMode(0);  // Opaque background, 1 = transparent
    if (thermistor1Fail == 0 && thermistor2Fail == 0) {
    // Define Menu Structure
    switch (menuIndex) {
      // ----------------------------------------
      // 0) MAIN MENU
      // ----------------------------------------
      case 0:
        selectIndexMax = 3;
        u8g2.setCursor(0, 8);
        if (steinhart1 > 40.0 || steinhart2 > 40.00) {
          u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, " CAUTION - PLATES HOT ");
        } else {
          u8g2.print(F("      MAIN MENU      "));
          u8g2.drawHLine(0, 9, 128);
        }
        u8g2.setCursor(6, 19);
        u8g2.print(F(" Start Reflow"));
        u8g2.setCursor(6, 27);
        u8g2.print(F(" Start Const Temp"));
        u8g2.setCursor(6, 50);
        u8g2.print(F(" Configuration"));
        u8g2.drawHLine(0, 54, 128);
        u8g2.setCursor(0, 64);
        u8g2.print(F("T1: "));  u8g2.print(steinhart1);
        u8g2.setCursor(64, 64);
        u8g2.print(F("T2: "));  u8g2.print(steinhart2);
        u8g2.setCursor(curPos[0], curPos[1]);
        u8g2.print(F(">"));
        break;

      // ----------------------------------------
      // 1) CONFIRM
      // ----------------------------------------
      case 1:
        selectIndexMax = 2;
        if (steinhart1 > 40.0 || steinhart2 > 40.00) {
          u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, " CAUTION - PLATES HOT ");
        }
        u8g2.setCursor(16, 20);
        u8g2.print(F("Confirm to "));
        if (startConfirm == 1) {
          u8g2.print(F("START"));
        } else {
          u8g2.print(F("STOP"));
        }
        if (runningMode == 0) {
          u8g2.setCursor(26, 30);
          u8g2.print(F("Constant Temp"));
        } else if (runningMode == 1) {
          u8g2.setCursor(22, 30);
          u8g2.print(F("Reflow Profile"));
        }
        u8g2.setCursor(35, 45);
        u8g2.print(F("NO"));
        u8g2.setCursor(75, 45);
        u8g2.print(F("YES"));
        u8g2.setCursor(curPos[0], curPos[1]);
        u8g2.print(F(">"));

        break;

      // ----------------------------------------
      // 2) CONFIGURATION MENU
      // ----------------------------------------
      case 2:
        selectIndexMax = 4;

        ////////////////// Header
        u8g2.setCursor(0, 8);
        if (steinhart1 > 40.0 || steinhart2 > 40.00) {
          u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, " CAUTION - PLATES HOT ");
        } else {
          u8g2.print(F("     CONFIG MENU     "));
          u8g2.drawHLine(0, 9, 128);
        }
        u8g2.drawHLine(0, 9, 128);

        u8g2.setCursor(6, 19);
        u8g2.print(F(" Reflow Profile"));
        u8g2.setCursor(6, 27);
        u8g2.print(F(" PID Parameters"));
        u8g2.setCursor(6, 43);
        u8g2.print(F(" Save Configuration"));
        u8g2.setCursor(6, 64);
        u8g2.print(F("BACK"));

        u8g2.setCursor(curPos[0], curPos[1]);
        u8g2.print(F(">"));
        break;

      // ----------------------------------------
      // 3) SET REFLOW PROFILE
      // ----------------------------------------
      case 3:  // Reflow Profile
        selectIndexMax = 8;

        ////////////////// Header
        u8g2.setCursor(0, 8);
        if (steinhart1 > 40.0 || steinhart2 > 40.00) {
          u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, " CAUTION - PLATES HOT ");
        } else {
          u8g2.println("   Reflow  Profile    ");
          u8g2.drawHLine(0, 9, 128);
        }
        u8g2.drawHLine(0, 9, 128);

        ////////////////// T1
        u8g2.setCursor(6, 19);
        u8g2.print(F("T1: "));
        if (selectFlag == 1 && menuCounter == 1) {
          u8g2.drawFrame(28, 10, 34, 11);
          u8g2.print(wrkInt);
        } else {
          u8g2.print(parametersReflow[0]);
        }
        u8g2.print(F(" C"));

        ////////////////// t1
        u8g2.setCursor(72, 19);
        u8g2.print(F("t1: "));
        if (selectFlag == 1 && menuCounter == 2) {
          u8g2.drawFrame(94, 10, 34, 11);
          u8g2.print(wrkInt);
      } else {
        u8g2.print(parametersReflow[1]);
        }
        u8g2.print(F(" s"));

        ////////////////// T2
        u8g2.setCursor(6, 29);
        u8g2.print(F("T2: "));
        if (selectFlag == 1 && menuCounter == 3) {
          u8g2.drawFrame(28, 20, 34, 11);
          u8g2.print(wrkInt);
      } else {
        u8g2.print(parametersReflow[2]);
        }
        u8g2.print(F(" C"));

        ////////////////// t2
        u8g2.setCursor(72, 29);
        u8g2.print(F("t2: "));
        if (selectFlag == 1 && menuCounter == 4) {
          u8g2.drawFrame(94, 20, 34, 11);
          u8g2.print(wrkInt);
      } else {
        u8g2.print(parametersReflow[3]);
        }
        u8g2.print(F(" s"));

        ////////////////// T3
        u8g2.setCursor(6, 39);
        u8g2.print(F("T3: "));
        if (selectFlag == 1 && menuCounter == 5) {
          u8g2.drawFrame(28, 30, 34, 11);
          u8g2.print(wrkInt);
      } else {
        u8g2.print(parametersReflow[4]);
        }
        u8g2.print(F(" C"));

        ////////////////// t3
        u8g2.setCursor(72, 39);
        u8g2.print(F("t3: "));
        if (selectFlag == 1 && menuCounter == 6) {
          u8g2.drawFrame(94, 30, 34, 11);
          u8g2.print(wrkInt);
      } else {
        u8g2.print(parametersReflow[5]);
        }
        u8g2.print(F(" s"));

        ////////////////// Reflow Hold
        u8g2.setCursor(6, 49);
        u8g2.print(F("Reflow Hold: "));
        if (selectFlag == 1 && menuCounter == 7) {
          u8g2.drawFrame(82, 40, 34, 11);
          u8g2.print(wrkInt);
      } else {
        u8g2.print(parametersReflow[6]);
        }
        u8g2.print(F(" s"));

        ////////////////// Back Selection
        u8g2.setCursor(6, 64);
        u8g2.print(F("BACK"));

        u8g2.setCursor(curPos[0], curPos[1]);
        u8g2.print(F(">"));
        break;

      // ----------------------------------------
      // 4) PID TUNING
      // ----------------------------------------
      case 4:  // PID Tuning
        selectIndexMax = 7;

        ////////////////// Header
        u8g2.setCursor(0, 8);
        if (steinhart1 > 40.0 || steinhart2 > 40.00) {
          u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, " CAUTION - PLATES HOT ");
        } else {
          u8g2.print(F("      PID Tuning     "));
          u8g2.drawHLine(0, 9, 128);
        }
        u8g2.drawHLine(0, 9, 128);

        ////////////////// Kp1
        u8g2.setCursor(6, 19);
        u8g2.print(F("Kp1: "));
        if (selectFlag == 1 && menuCounter == 1) {
          u8g2.drawFrame(34, 10, 28, 11);
          u8g2.print(wrkDouble);
      } else {
        u8g2.print(parametersPID[0]);
        }

        ////////////////// Ki1
        u8g2.setCursor(6, 29);
        u8g2.print(F("Ki1: "));
        if (selectFlag == 1 && menuCounter == 2) {
          u8g2.drawFrame(34, 20, 28, 11);
          u8g2.print(wrkDouble);
      } else {
        u8g2.print(parametersPID[1]);
        }

        ////////////////// Kd1
        u8g2.setCursor(6, 39);
        u8g2.print(F("Kd1: "));
        if (selectFlag == 1 && menuCounter == 3) {
          u8g2.drawFrame(34, 30, 28, 11);
          u8g2.print(wrkDouble);
      } else {
        u8g2.print(parametersPID[2]);
        }

        ////////////////// Kp2
        u8g2.setCursor(72, 19);
        u8g2.print(F("Kp2: "));
        if (selectFlag == 1 && menuCounter == 4) {
          u8g2.drawFrame(100, 10, 28, 11);
          u8g2.print(wrkDouble);
      } else {
        u8g2.print(parametersPID[3]);
        }

        ////////////////// Ki2
        u8g2.setCursor(72, 29);
        u8g2.print(F("Ki2: "));
        if (selectFlag == 1 && menuCounter == 5) {
          u8g2.drawFrame(100, 20, 28, 11);
          u8g2.print(wrkDouble);
      } else {
        u8g2.print(parametersPID[4]);
        }

        ////////////////// Kd2
        u8g2.setCursor(72, 39);
        u8g2.print(F("Kd2: "));
        if (selectFlag == 1 && menuCounter == 6) {
          u8g2.drawFrame(100, 30, 28, 11);
          u8g2.print(wrkDouble);
      } else {
        u8g2.print(parametersPID[5]);
        }

        ////////////////// Back Selection
        u8g2.setCursor(6, 64);
        u8g2.print(F("BACK"));

        u8g2.setCursor(curPos[0], curPos[1]);
        u8g2.print(F(">"));
        break;
      
      // ----------------------------------------
      // 5) SAVE CONFIGURATION
      // ----------------------------------------
      case 5:
        if (steinhart1 > 40.0 || steinhart2 > 40.00) {
          u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, " CAUTION - PLATES HOT ");
        }
        u8g2.setCursor(30, 30);
        u8g2.print(F("Saving Data"));
        u8g2.setCursor(36, 40);
        u8g2.print(F("to EEPROM"));
        break;

      // ----------------------------------------
      // 98) RUNNING - CONSTANT TEMP
      // ----------------------------------------
      case 98:  // Running - Constant Temp
        selectIndexMax = 2;
        u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, " CONST TEMP RUNNING ");
        u8g2.setCursor(6, 24);
        u8g2.print(F("T1: "));
        u8g2.print(T1Disp);
        u8g2.print(F(" C"));
        u8g2.setCursor(6, 32);
        u8g2.print(F("T2: "));
        u8g2.print(T2Disp);
        u8g2.print(F(" C"));
        u8g2.setCursor(6, 64);
        u8g2.setCursor(6, 48);
        u8g2.print(F("SP: "));
        if (selectFlag == 1 && menuCounter == 1) {
          u8g2.drawFrame(28, 39, 34, 11);
          u8g2.print(wrkInt);
        } else {
          u8g2.print(constTempSP);
        }
        u8g2.print(F(" C"));
        u8g2.setCursor(6, 64);
        u8g2.print(F("STOP"));

        u8g2.setCursor(curPos[0], curPos[1]);
        u8g2.print(F(">"));
        break;
      
      // ----------------------------------------
      // 99) RUNNING - REFLOW PROFILE
      // ----------------------------------------
      case 99:  // Running - Reflow Profile
        selectIndexMax = 1;
        u8g2.setCursor(0, 8);
        if (runningState == 5) {
          u8g2.print(F("       COMPLETE      "));
          u8g2.drawHLine(0, 9, 128);
        } else {
          u8g2.drawButtonUTF8(0, 8, U8G2_BTN_INV, 0, 0, 1, "    REFLOW RUNNING    ");
        }
        u8g2.setCursor(0, 24);
        switch (runningState) {
          case 1:
            u8g2.print(F("RAMP"));
            break;
          case 2:
            u8g2.print(F("SOAK"));
            break;
          case 3:
            u8g2.print(F("RFLW RAMP"));
            break;
          case 4:
            u8g2.print(F("REFLOW"));
            break;
          case 5:
            u8g2.print(F("COOLING"));
            break;
        }
        u8g2.setCursor(59, 24);
        u8g2.print(F("Time: "));
        u8g2.print(runningSecondCounter);
        u8g2.print(F(" s"));
        u8g2.setCursor(0, 40);
        u8g2.print(F("T1: "));
        u8g2.print(T1Disp);
        u8g2.setCursor(66, 40);
        u8g2.print(F("SP: "));
        u8g2.print(pid_Setpoint);
        u8g2.setCursor(0, 48);
        u8g2.print(F("T2: "));
        u8g2.print(T2Disp);
        u8g2.setCursor(0, 64);
        u8g2.print(F("> STOP"));
        break;
    }
    } else {
      if (thermistor1Fail == 1 && thermistor2Fail == 0) {
      u8g2.setCursor(12, 18);
      u8g2.print(F("Thermistor 1 Fail"));
      } else if (thermistor1Fail == 0 && thermistor2Fail == 1) {
        u8g2.setCursor(12, 18);
        u8g2.print(F("Thermistor 2 Fail"));
      } else if (thermistor1Fail == 1 && thermistor2Fail == 1) {
        u8g2.setCursor(10, 18);
        u8g2.print(F("Thermistors 1 & 2"));
        u8g2.setCursor(46, 26);
        u8g2.print(F("Failure"));

      }
    }
  } while (u8g2.nextPage());
}

// -----------------------------------------------------------
// Temperature Readings and Calculations
// -----------------------------------------------------------
void readThermistor() {
  uint8_t i;
  float average1;
  float average2;
  uint8_t thermistor1FailCounter;
  uint8_t thermistor2FailCounter;

  // Buffer thermistor readings prior to performing read operation
  thermistor1Buffer = steinhart1;
  thermistor2Buffer = steinhart2;

  // take N samples1 in a row, with a slight delay
  for (i = 0; i < Numsamples; i++) {
    samples1[i] = analogRead(THERMISTORPIN1);
    samples2[i] = analogRead(THERMISTORPIN2);
    delay(5);
  }
  // average all the samples1 out
  average1 = 0;
  average2 = 0;
  for (i = 0; i < Numsamples; i++) {
    average1 += samples1[i];
    average2 += samples2[i];
  }
  average1 /= Numsamples;
  average2 /= Numsamples;
  average1 = ((1023 * SERIESRESISTOR1) - (average1 * SERIESRESISTOR1)) / average1;
  average2 = ((1023 * SERIESRESISTOR2) - (average2 * SERIESRESISTOR2)) / average2;

  // Conversion for Thermistor 1
  steinhart1 = average1 / THERMISTORNOMINAL1;          // (R/Ro)
  steinhart1 = log(steinhart1);                        // ln(R/Ro)
  steinhart1 /= BCOEFFICIENT1;                         // 1/B * ln(R/Ro)
  steinhart1 += 1.0 / (TEMPERATURENOMINAL1 + 273.15);  // + (1/To)
  steinhart1 = 1.0 / steinhart1;                       // Invert
  steinhart1 -= 273.15;                                // convert absolute temp to C

  // Conversion for Thermistor 2
  steinhart2 = average2 / THERMISTORNOMINAL2;          // (R/Ro)
  steinhart2 = log(steinhart2);                        // ln(R/Ro)
  steinhart2 /= BCOEFFICIENT2;                         // 1/B * ln(R/Ro)
  steinhart2 += 1.0 / (TEMPERATURENOMINAL2 + 273.15);  // + (1/To)
  steinhart2 = 1.0 / steinhart2;                       // Invert
  steinhart2 -= 273.15;                                // convert absolute temp to C

  // Thermistor failure conditions - Prevent thermal runaway
  // 1. If thermistor readings are < 20 deg C, consider thermistor as failed
  // 2. Increment thermistor failure counter if thermistor reading is identical to buffered value
  //    If failure counter >= 3 (buffer = reading 3 or more times) consider thermistor as failed
  // If thermistor failure flag is set, reflow profile will stop and failure message will be displayed

  if (thermistor1Buffer = steinhart1) {                   // Increment failure counters if reading = buffered value
    thermistor1FailCounter = thermistor1FailCounter++;
  } else {
    thermistor1FailCounter = 0;
  }
  if (thermistor2Buffer = steinhart2) {
    thermistor2FailCounter = thermistor2FailCounter++;
  } else {
    thermistor2FailCounter = 0;
  }
  
  if (steinhart1 < -20 || thermistor1FailCounter >= 3) {  // Set thermistor fail flag(s) if measured temperature is < -20 deg C OR Fail Counter >= 3
    thermistor1Fail = 1;
  } else {
    thermistor1Fail = 0;
  }

  if (steinhart2 < -20 || thermistor2FailCounter >= 3) {
    thermistor2Fail = 1;
  } else {
    thermistor2Fail = 0;
  }
}

// -----------------------------------------------------------
// Running State Logic
// -----------------------------------------------------------
void reflowRunning() {
    
  // Execute Thermistor Reads
  readThermistor();

  // Ensure PID Loops are in AUTO 
  hotPlate1PID.SetMode(AUTOMATIC);
  hotPlate2PID.SetMode(AUTOMATIC);
  
  // Second Counter
  if (runningState < 5) {
    if(millis() - time_now > 1000){   // 1 Second timer - increment running second counter each time timer elapses. Update temperature display so it is more stable
        time_now = millis();
        runningSecondCounter ++;
        T1Disp = steinhart1;
        T2Disp = steinhart2;
    }
  }

  switch (runningState) {
    case 1:   // RAMP
      pid_Setpoint = (((double)parametersReflow[0] - initTempSnapshot) / ((double)parametersReflow[1]) - 0) * (double)runningSecondCounter + initTempSnapshot;
      if (runningSecondCounter >= parametersReflow[1]) {
        runningState = 2;
      }
      break;
    case 2:   // SOAK
      pid_Setpoint = (((double)parametersReflow[2] - (double)parametersReflow[0]) / ((double)parametersReflow[3] - (double)parametersReflow[1])) * ((double)runningSecondCounter - (double)parametersReflow[1]) + (double)parametersReflow[0];
      if (runningSecondCounter >= parametersReflow[3]) {
        runningState = 3;
      }
      break;
    case 3:   // REFLOW RAMP
      pid_Setpoint = (((double)parametersReflow[4] - (double)parametersReflow[2]) / ((double)parametersReflow[5] - (double)parametersReflow[3])) * ((double)runningSecondCounter - (double)parametersReflow[3]) + (double)parametersReflow[2];
      if (runningSecondCounter >= parametersReflow[5]) {
        runningState = 4;
      }
      break;
    case 4:   // REFLOW
      pid_Setpoint = (double)parametersReflow[4];
      if (runningSecondCounter >= (parametersReflow[5] + parametersReflow[6])) {
        runningState = 5;
      }
      break;
    case 5:   // COOLING / COMPLETE
      pid_Setpoint = 0;
      hotPlate1PID.SetMode(MANUAL);
      hotPlate2PID.SetMode(MANUAL);
      pid1_Output = 0;
      pid2_Output = 0;
      break;
  }

  // Execute PID Loops
  pidLoop1();
  pidLoop2();
}

void constTempRunning() {
  
  // Execute Thermistor Reads
  readThermistor();
  
  // Set PID SP to Constant Temperature SP Value
  pid_Setpoint = constTempSP;

  // Ensure PID Loops are in AUTO 
  hotPlate1PID.SetMode(AUTOMATIC);
  hotPlate2PID.SetMode(AUTOMATIC);

  if(millis() - time_now > 1000){   // 1 Second timer - Update temperature display so it is more stable
        time_now = millis();
        T1Disp = steinhart1;
        T2Disp = steinhart2;
    }

  // Execute PID Loops
  pidLoop1();
  pidLoop2();
}

// -----------------------------------------------------------
// Setup & Loop
// -----------------------------------------------------------
void setup() {
  int i = 0;

  // ----------------------------------------
  // Set up funcitons for the rotary encoder
  // ----------------------------------------

  // Set encoder pins as inputs
  pinMode(encCLK_inp, INPUT_PULLUP);
  pinMode(encDT_inp, INPUT_PULLUP);
  pinMode(encSW_inp, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encCLK_inp), isrEncCLK, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encDT_inp), isrEncDT, CHANGE);

  // ----------------------------------------
  // Read saved darameter data from EEPROM
  // ----------------------------------------
  
  readUInt8TArrayFromEEPROM(1, parametersReflowREAD, 7);
  for (i = 0; i < 7; i++) {
    parametersReflow[i] = parametersReflowREAD[i];  // Need to cast the read INT values to double to retaing decimal places
  }
  readIntArrayFromEEPROM(8, parametersPIDREAD, 6);
  // Convert INT PID Parameters from EEPROM memory to double and store them in variables used in program logic
  for (i = 0; i < 6; i++) {
    parametersPID[i] = (double)parametersPIDREAD[i] / 100;  // Need to cast the read INT values to double to retaing decimal places
  }
  
  // ----------------------------------------
  // Set up funcitons for the u8g2
  // ----------------------------------------
  delay(250);  // wait for the OLED to power up
  u8g2.begin();

  // ----------------------------------------
  // Initialization for PID Loops
  // ----------------------------------------
  // Initialize PID Loop Sample Times
  hotPlate1PID.SetSampleTime(200); // Sets PID loop sampling time. Default is 200ms
  hotPlate2PID.SetSampleTime(200); // Sets PID loop sampling time. Default is 200ms
  // Set temporary low value setpoints for PID loops
  pid_Setpoint = 50;
  // Set PID Loops to manual so they don't start until ready
  hotPlate1PID.SetMode(MANUAL);
  hotPlate2PID.SetMode(MANUAL);

  // Get initial temperature reading for display
  readThermistor();
}

void loop() {
  // ----------------------------------------
  // Rotary encoder handling
  // ----------------------------------------

  // Poll the encoder pushbutton switch. Delay for minor debounce effect.
  if (!digitalRead(encSW_inp)) {
    encSW = 1;
    delay(100);
  } else {
    encSW = 0;
  }

  // Handle rotary encoder rotation
  noInterrupts();
  protectedMenuCounter = menuCounter;
  interrupts();
  previousMenuCounter = protectedMenuCounter;

  // Handle parameter modifications
  if (selectFlag == 1) {
    calcParameters();
  }

  // Display Handling
  updateCursorPosition();
  updateDisplay();

  // Initialize Running State to 1 (RAMP) when profile run is started
  if (running == 1 && runningBuffer == 0) {   
    runningSecondCounter = 0;
    readThermistor();
    initTempSnapshot = (steinhart1 + steinhart2) / 2; // Capture initial temperature as average between the 2 thermistors
    runningState = 1;
  }

  // Additional logic when not running - Force PID loops to Manual mode, read thermistor every 10 sec for 'Hot' menu display and thermistor fail check
  if (!running) {
    
    pid_Setpoint = 0;                 // Force PID values to 0 / Manual & force a 0 output on PWM output pins
    hotPlate1PID.SetMode(MANUAL);
    hotPlate2PID.SetMode(MANUAL);
    pid1_Output = 0;
    pid2_Output = 0;
    analogWrite(pwmPin1, 0);
    analogWrite(pwmPin2, 0);

    if(millis() - time_now > 10000){   // 10 Second timer - Read thermistors every 10 sec when not running for 'HOT' menu display.
        time_now = millis();           // Also increment thermistor fail counters and set fail flag(s) if 3 identical readings are encountered
        readThermistor();
    }
    
    initTempSnapshot = 0;             // Clear / reset intial temp snapshot value
  } 
  
  // Call running state logic to execute SP calculation and PID loop execution
  if (running == 1 && runningMode == 1) {
    reflowRunning();
  } else if (running == 1 && runningMode == 0) {
    constTempRunning();
  }

  // Buffer running flag
  runningBuffer = running;  
}
