#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <LiquidCrystal.h>
#include <stdio.h>

// Purpose: provide handy reference code as a template for driving the Sparkfun LCD-10862 w- RGB Backlight
// It is largely derived from https://github.com/cottjr/DMS-Laser-Cutter-Usage-Timer, commit af40041 from 2016 March 28
// That source was originally conceived to track usage hours on a consumable laser tube at the Dallas Makerspace

/*
 This sketch shows two timers. One is user resettable and one is not. 
 The timers keep track of how long an analog input goes past a threshold.
 The smallest increment is one minute. If the threshold is exceeded for 
 any point in a minute (even less than a second) that minute is counted 
 towards the time.
 
  The circuit:          nano pins   -> Mega2560 pin map, targeting the Digital Expansion connector J38 of the MoebiusTech 4x motor driver board
 * LCD RS pin to digital pin 12     -> 25
 * LCD Enable pin to digital pin 11 -> 28
 * LCD D4 pin to digital pin 5      -> 29
 * LCD D5 pin to digital pin 4      -> 30
 * LCD D6 pin to digital pin 3      -> 32
 * LCD D7 pin to digital pin 2      -> 33
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 *
 * Analog input for thresholding analog in pin 0
 * button input for reset -> make on Nano D6, "pin 6"
 */

/*
// These items more relevant to driving the LCD module
*/

#define MAX_OUT_CHARS 18 //max nbr of characters to be sent on any one serial command,
// plus the terminating character sprintf needs to prevent it from overwriting following memory locations, yet still
//    send the complete 16 character string to the LCD
// sprintf: what a terrible construct...
char buffer[MAX_OUT_CHARS];  //buffer used to format a line (+1 is for trailing 0)
char buffer2[MAX_OUT_CHARS]; //buffer used to format a line (+1 is for trailing 0)

// initialize the library with the numbers of the interface pins
// LiquidCrystal lcd(12, 11, 5, 4, 3, 2);   // Original Arduino Nano pin map
LiquidCrystal lcd(25, 28, 29, 30, 32, 33); // Arduino Mega2560 pin map, targeting the Digital Expansion connector J38 of the MoebiusTech 4x motor driver board

int RedPin = 41;   //6;			// pin for driving the Red backlight LED  (Nano D6)
int GreenPin = 40; //9;			// pin for driving the Green backlight LED  (Nano D9)
int BluePin = 47;  //10;			// pin for driving the Blue backlight LED  (Nano D10)

// version number for this program.  simply counting releases
const unsigned int ThisCurrentVersion = 3;

/*
// Remaining variables mostly for original Laser Cutter Timer
*/
const unsigned long second = 1000;
const unsigned long minute = 60000; // number of millis in a minute
const unsigned long hour = 3600000; // number of millis in an hour

int userResetPin = 7; // pin used for resetting user laser time  (Nano D7)

boolean TestOutToggle = true;
int TestOutPin = 13; // pin for toggling once per every inner loop operation, just to keep tabs on software (Nano D13)

boolean TestOutToggle2 = true;
int TestOutPin2 = 8; // pin for toggling once per every outer loop operation, just to keep tabs on software (Nano D8)

int analogPin = 0;          // light sensor input (or voltsge) connected to analog pin 0
int analogVal = 0;          // variable to store the value read
int anaLowThreshold = 500;  // if analog value rises above this value its considered ON
int anaHighThreshold = 524; // if analog value falls below this value its considered OFF
int cursorPos = 0;
unsigned long millisOnLast = 0;
unsigned long millisOffLast = 0;
unsigned long millisTemp = 0;
unsigned long millisDiff = 0;
boolean lastLaserOn = false;
unsigned long userMillis = 0;
int userHours = 0;
int userMinutes = 0; // number of minutes user has used the laser (resettable when button pressed)
int userSeconds = 0;
int tubeHours = 0;
int tubeMinutes = 0; // number of minutes tube has been used (not resettable)
int tubeSeconds = 0;
unsigned long tubeMillis = 0;
unsigned long lastWriteToEEPROMMillis = 0; // number of millis that the EEPROM was laser written to

char buffer1[MAX_OUT_CHARS]; //buffer used to format a line (+1 is for trailing 0)

struct config_t
{
  unsigned long seconds;          // tube seconds
  unsigned long uSeconds;         // user seconds
  unsigned long EEPROMwriteCount; // EEPROM write cycle count
                                  // Arduino EEPROM good for ~ 100,000 writes
                                  // only lasts ~ 1 year if write every 5 min, 24 x 7
                                  // and have ~ 12 people hitting reset every day
  unsigned int thisVersion;       // version number of this software
} laserTime;

// Purpose
//  drive RGB backlight LED's with PWM
//  reference Branch "backlight8colorDigital"
// Reference also
//  comments in loop() below // Another handy reference for using sprintf to the LCD
void backlightColorPWM(int R, int G, int B)
{
  // accept standard RGB color specs, with values from 0..255
  // for a nice RGB value selection chart, see this source
  //  http://blogs.msdn.com/blogfiles/davidlean/WindowsLiveWriter/SQLReportingHowtoConditionalColor24Funct_B98C/image_8.png
  //  http://blogs.msdn.com/b/davidlean/archive/2009/02/17/sql-reporting-how-to-conditional-color-2-4-functions-for-tables-charts.aspx
  // render that color by writing appropriate values to PWM based pins
  // This assumes that dropping resistors are roughly matched to provide reasonable color balancing

  analogWrite(RedPin, R);
  analogWrite(GreenPin, G);
  analogWrite(BluePin, B);
}

int clip(int whatValue, int whatLimit)
{
  if (whatValue > abs(whatLimit))
  {
    return abs(whatLimit);
  }
  else if (whatValue < -abs(whatLimit))
  {
    return -abs(whatLimit);
  }
  else
  {
    return whatValue;
  }
}

// Purpose
//  Write 4 wheel integer values to the 16 Char by 2 line LCD display
//  Primarily intended for encoder related position or velocity values
//  Attempt to clip integer values to allowed ranges
//  HOWEVER- Note that this fails for values somewhat larger than 3,000
//  => seems to be some issue with sprintf() on arduino
// Format
//  Front Left          Front Right
//  Rear Left           Rear Right
//  F	L	+/-	1	2	3	4	5	_	R	'+/-	1	2	3	4	5
//  R	L	'+/-	1	2	3	4	5	_	R	'+/-	1	2	3	4	5
// Encoder Delta Range 29999 (values up to this limit DO display correctly)
//  230.8	20ms periods at max 130ticks/period velocity
//  4.6	seconds at max velocity
//  => must stay under ~ 4.5 sec at max velocity to keep encoder deltas w/in allowed range
// Encoder Delta Range 99999 (but 99999 doesn't display correctly)
//  769.2	20ms periods at max 130ticks/period velocity
//  15.4	seconds at max velocity
void write4wheelIntegersToLCD(int FL, int FR, int RL, int RR)
{
  const int clipMax = 29999;

  // sprintf(buffer, "FL%+6d R%+6d", clip(FL, clipMax), clip(FR, clipMax));
  // sprintf(buffer2, "RL%+6d R%+6d", clip(RL, clipMax), clip(RR, clipMax));

  // consider replacign sprintf() with a better function, e.g. fmt::format_int, per https://www.zverovich.net/2013/09/07/integer-to-string-conversion-in-cplusplus.html
  sprintf(buffer, "FL%+6d R%+6d", -12345, +23456);
  sprintf(buffer2, "RL%d ",45678);

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);
}

void setupLCD()
{

  lcd.begin(16, 2);

  // // set display backlight to Purple
  // backlightColorPWM(255, 0, 255);

  // // Briefly show Arduino status (ie. from legacy Laser Cutter Usage Timer, non-volatile values stored in EEPROM)
  // sprintf(buffer, "Version: %02d", laserTime.thisVersion);
  // sprintf(buffer2, "Writes: %06d", laserTime.EEPROMwriteCount);

  // lcd.setCursor(0, 0);
  // lcd.print(buffer);

  // lcd.setCursor(0, 1);
  // lcd.print(buffer2);

  // delay(1500);

  // set display backlight to White
  backlightColorPWM(255, 255, 255);
  // writeEncoderPosToLCD(-12345,23456,12345,-23456);
  // writeEncoderPosToLCD(34567,45678,12345,-23456);
  // writeEncoderPosToLCD(-18765, -17654, 12345, -23456);
  // writeEncoderPosToLCD(-18765, -17654, 12345, 30000);
  write4wheelIntegersToLCD(-18, -17, 5, 30000);

  delay(8000);

  // set display backlight to Red
  backlightColorPWM(255, 0, 0);

  // show random text
  sprintf(buffer, "Yup-random stuff");
  sprintf(buffer2, "   Hello Red    ");

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);

  delay(1500);

  // set display backlight to Green
  backlightColorPWM(0, 255, 0);

  // show random text
  sprintf(buffer, "And Green       ");
  sprintf(buffer2, "   Hi there     ");

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);

  delay(1500);

  // set display backlight to Blue
  backlightColorPWM(0, 0, 255);

  // show random text
  sprintf(buffer, "And Blue        ");
  sprintf(buffer2, "                ");

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);

  delay(1500);

  // set display backlight to blue-green
  backlightColorPWM(0, 255, 255);

  // show random text
  sprintf(buffer, "    sort of     ");
  sprintf(buffer2, "   blue-green   ");

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);

  delay(1500);

  // set display backlight to white-ish
  backlightColorPWM(255, 255, 255);

  // show random text
  sprintf(buffer, "    White       ");
  sprintf(buffer2, "     ish        ");

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);

  delay(1500);

  // set display backlight to Yellow
  backlightColorPWM(255, 255, 0);

  // show random text
  sprintf(buffer, "last not least  ");
  sprintf(buffer2, "   Yellow       ");

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);

  delay(1500);

  // set display backlight to white-ish
  backlightColorPWM(255, 255, 255);
  write4wheelIntegersToLCD(12345, -23456, -34567, 45678);
  delay(15000);
}

// Setup mostly Legacy stuff from Laser Cutter Usage Timer.
//  nice reference for certain tricks, but otherwise not relevant to LCD interface
void setup()
{
  pinMode(userResetPin, INPUT);
  pinMode(TestOutPin, OUTPUT);
  pinMode(TestOutPin2, OUTPUT);

  Serial.begin(250000);
  //EEPROM_readAnything(0, laserTime);
  //ROUND_ROBIN_EEPROM_ZeroOutWindow();  //use this if you need to clean all the eeprom possitions withing the window defined in the header
  int addr = ROUND_ROBIN_EEPROM_read(laserTime);
  tubeMillis = laserTime.seconds * 1000;
  userMillis = laserTime.uSeconds * 1000;

  // Initialize the version number in EEPROM if this is the first load after a reflash
  if (laserTime.thisVersion == 0)
  {
    laserTime.thisVersion = ThisCurrentVersion;
    laserTime.EEPROMwriteCount = laserTime.EEPROMwriteCount + 1;
    //EEPROM_writeAnything(0, laserTime);
    addr = ROUND_ROBIN_EEPROM_write(laserTime);
  }

  // Setup those items particular to driving the LCD
  setupLCD();

  // rest is legacy stuff from Laser Cutter Usage Timer
  delay(5000); // cheap debouncing trick

  Serial.print("Values stored in EEPROM address ");
  Serial.println(addr);

  Serial.print("  laserTime.seconds ie tube: ");
  Serial.println(laserTime.seconds);

  Serial.print("  laserTime.uSeconds ie user: ");
  Serial.println(laserTime.uSeconds);

  Serial.print("  laserTime.EEPROMwriteCount: ");
  Serial.println(laserTime.EEPROMwriteCount);

  Serial.print("  laserTime.thisVersion: ");
  Serial.println(laserTime.thisVersion);

  Serial.println("setup Complete");
  Serial.println("");
}

// Loop mostly Legacy stuff from Laser Cutter Usage Timer.
//  nice reference for certain tricks, but otherwise not relevant to LCD interface
void loop()
{

  //  toggle a pin during every outer loop execution - just watch on a 'scope for a sense of timing
  //  baseline shows that this loop takes about 37 ms to execute
  if (TestOutToggle2)
  {
    // nominally keep disabled, since Nano D13 annoyingly toggles an LED
    //		digitalWrite(TestOutPin2, HIGH);
  }
  else
  {
    digitalWrite(TestOutPin2, LOW);
  }
  TestOutToggle2 = !TestOutToggle2;

  // debug EEPROM protection timing
  //	Serial.print  ("lastWriteToEEPROMMillis: ");
  //	Serial.println(lastWriteToEEPROMMillis);

  // do a tight loop on checking the laser and keeping track of on/off times
  for (int i = 0; i <= 100; i++)
  {

    //  toggle a pin during every inner loop execution - just watch on a 'scope for a sense of timing
    //  baseline shows that this loop takes about 178 us to execute
    if (TestOutToggle)
    {
      // nominally keep disabled, since Nano D13 annoyingly toggles an LED
      //		digitalWrite(TestOutPin, HIGH);
    }
    else
    {
      digitalWrite(TestOutPin, LOW);
    }
    TestOutToggle = !TestOutToggle;

    analogVal = analogRead(analogPin); // read the input pin
                                       //    Serial.print("anaVal:");
                                       //    Serial.println(analogVal);

    // set backlight to Red while laser is firing
    // set backlight to Yellow while laser NOT firing and user time is accumulated
    // set backlight back to Blue when laser not firing and no time accumulated
    if (analogVal < anaLowThreshold)
    { // go red
      backlightColorPWM(255, 0, 0);
    }
    else if (userMillis > 0)
    { // go yellowish
      backlightColorPWM(255, 245, 0);
    }
    else
    { // go Blue
      backlightColorPWM(0, 0, 255);
    }

    // consider checking hysteresis logic - it appears that anaLowThreshold alone determines laser on/off state
    if ((analogVal < anaLowThreshold) && !lastLaserOn)
    { // laser has been off, laser turning on here
      lastLaserOn = true;
      millisOnLast = (unsigned long)millis();
      millisDiff = millisOnLast - millisOffLast;
    }
    else if ((analogVal < anaLowThreshold) && lastLaserOn)
    { // laser has been on here, continuing on
      lastLaserOn = true;

      millisTemp = (unsigned long)millis();
      millisDiff = millisTemp - millisOnLast;
      millisOnLast = millisTemp;
    }
    else if ((analogVal > anaHighThreshold) && lastLaserOn)
    { // laser has been on, turning off
      lastLaserOn = false;
      millisOffLast = (unsigned long)millis();
    }
    else
    { // laser has been off, staying off
      lastLaserOn = false;
      millisOffLast = (unsigned long)millis();
    }
    int userReset = digitalRead(userResetPin);
    if (userReset == LOW)
    {

      //    allow reset and writing once every 10 seconds, but no faster
      //    write values to EPROM every time user hits reset
      if (millis() > (lastWriteToEEPROMMillis + 10000))
      {
        userMillis = 0;
        laserTime.seconds = tubeMillis / 1000;
        laserTime.uSeconds = userMillis / 1000;
        laserTime.EEPROMwriteCount = laserTime.EEPROMwriteCount + 1;
        laserTime.thisVersion = ThisCurrentVersion;
        //EEPROM_writeAnything(0, laserTime);
        int addr = ROUND_ROBIN_EEPROM_write(laserTime);

        lastWriteToEEPROMMillis = millis();

        Serial.println("User hit reset & Wrote to EEPROM");

        Serial.print("  EEPROM address: ");
        Serial.println(addr);

        Serial.print("  laserTime.seconds ie tube: ");
        Serial.println(laserTime.seconds);

        Serial.print("  laserTime.uSeconds ie user: ");
        Serial.println(laserTime.uSeconds);

        Serial.print("  laserTime.EEPROMwriteCount: ");
        Serial.println(laserTime.EEPROMwriteCount);

        Serial.print("  laserTime.thisVersion: ");
        Serial.println(laserTime.thisVersion);
      }
    }
    userMillis = userMillis + millisDiff;
    tubeMillis = tubeMillis + millisDiff;
    millisDiff = 0;
  }

  // set the cursor to column 12, line 1    (is this really working this way? or does it rewrite the entire display?)
  // (note: line 1 is the second row, since counting begins with 0):
  tubeHours = tubeMillis / hour;
  tubeMinutes = (tubeMillis - tubeHours * hour) / minute;
  tubeSeconds = (tubeMillis - tubeHours * hour - tubeMinutes * minute) / second;
  userHours = userMillis / hour;
  userMinutes = (userMillis - userHours * hour) / minute;
  userSeconds = (userMillis - userHours * hour - userMinutes * minute) / second;

  // Another handy reference for using sprintf to the LCD
  sprintf(buffer, "User    %02d:%02d:%02d", userHours, userMinutes, userSeconds);
  sprintf(buffer2, "Tube %05d:%02d:%02d", tubeHours, tubeMinutes, tubeSeconds);

  lcd.setCursor(0, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(buffer2);

  // Only write to EEPROM if the current value is more than 5 minutes from the previous EEPROM value
  // to reduce the number of writes to EEPROM, since any one location is only good for ~ 100,000 writes
  //EEPROM_readAnything(0, laserTime);
  int addr = ROUND_ROBIN_EEPROM_read(laserTime);
  unsigned long laserSeconds = laserTime.seconds;

  // note - it appears that only one of the following If statements is required
  if ((laserSeconds + 300) < (tubeMillis / 1000))
  {
    Serial.print("LaserSeconds:");
    Serial.print(laserSeconds);
    Serial.print("adjTubeSecs:");
    Serial.println(((tubeMillis / 1000) + 300));
    laserTime.seconds = tubeMillis / 1000;
    laserTime.uSeconds = userMillis / 1000;
    laserTime.EEPROMwriteCount = laserTime.EEPROMwriteCount + 1;
    laserTime.thisVersion = ThisCurrentVersion;
    //EEPROM_writeAnything(0, laserTime);
    addr = ROUND_ROBIN_EEPROM_write(laserTime);
    lastWriteToEEPROMMillis = millis();
    Serial.println("Wrote to EEPROM - tube has another 5 minutes of use");

    Serial.print("  EEPROM address: ");
    Serial.println(addr);

    Serial.print("  laserTime.seconds ie tube: ");
    Serial.println(laserTime.seconds);

    Serial.print("  laserTime.uSeconds ie user: ");
    Serial.println(laserTime.uSeconds);

    Serial.print("  laserTime.EEPROMwriteCount: ");
    Serial.println(laserTime.EEPROMwriteCount);

    Serial.print("  laserTime.thisVersion: ");
    Serial.println(laserTime.thisVersion);
  }
  if ((millis() > (lastWriteToEEPROMMillis + 300000)) && ((laserSeconds + 1) * 1000 < tubeMillis))
  {
    // ie. if it has been 5 mins since last write and the value has changed, write now
    laserTime.seconds = tubeMillis / 1000;
    laserTime.uSeconds = userMillis / 1000;
    laserTime.EEPROMwriteCount = laserTime.EEPROMwriteCount + 1;
    laserTime.thisVersion = ThisCurrentVersion;
    //this method has been replaced to distribute the eeprom writing accross a wider address space
    //EEPROM_writeAnything(0, laserTime);
    addr = ROUND_ROBIN_EEPROM_write(laserTime);
    lastWriteToEEPROMMillis = millis();
    Serial.println("Wrote to EEPROM - value has changed in last 5 minutes");

    Serial.print("  EEPROM address: ");
    Serial.println(addr);

    Serial.print("  laserTime.seconds ie tube: ");
    Serial.println(laserTime.seconds);

    Serial.print("  laserTime.uSeconds ie user: ");
    Serial.println(laserTime.uSeconds);

    Serial.print("  laserTime.EEPROMwriteCount: ");
    Serial.println(laserTime.EEPROMwriteCount);

    Serial.print("  laserTime.thisVersion: ");
    Serial.println(laserTime.thisVersion);
  }
}
