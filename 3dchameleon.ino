// November 28, 2024 - Adapted for Arduino Uno + CNC Shield

/* 3DChameleon Mk4.1 Firmware - Arduino Uno CNC Shield Version

Adapted for Arduino Uno with CNC Shield v3
- Z-axis: Extruder motor (filament feeding)
- X-axis: Selector motor (tool selection)
- Trigger: A3 with PC817C optocoupler
- OLED: I2C (SDA=A4, SCL=A5)
- Servo: Pin 11 (filament cutter)

Original Copyright 2024 William J. Steele

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

Single Button Press Commands (count pulses of selector)

#1 - 1
#2 - 2 (T0)
#3 - 3 (T1)
#4 - 4 (T2)
#5 - 5 (T3)
#6 - 6 (Home and Load T0)
#7 - 7 (Unload Last and Home)
#8 - 8 (Home)
#9 - 9 (Next Filament)
#10 - 10 (Random Filament)
#11 - 11 (Clear EEPROM - testing only)

*/

#include <SPI.h>
#include <Servo.h>
#include <EEPROM.h>


// OLED removed - using Serial output instead

// CNC Shield doesn't have SX1509 IO expander

// defines pins numbers - Arduino Uno with CNC Shield
// Eixo Z para motor da extrusora (filament feeding)
#define extEnable 12   // Z_ENABLE
#define extStep 4      // Z_STEP
#define extDir 7       // Z_DIR

// Eixo X para motor do seletor (tool selector)
#define selEnable 8    // X_ENABLE
#define selStep 2      // X_STEP
#define selDir 5       // X_DIR

// Trigger button with PC817C optocoupler
#define trigger A3     // Trigger button (optoacoupler)

// Servo pin (compatible with Arduino Uno)
#define SERVO_PIN 11

const int counterclockwise = HIGH;
const int clockwise = !counterclockwise;

const int stepsPerRev = 200;
const int microSteps = 8;
const int speedDelay = 170;

const int defaultBackoff = 10;

Servo filamentCutter;  // create servo object to control a servo
int cutterPos = 0;    // variable to store the servo position
bool reverseServo = true;

int currentExtruder = -1;
int nextExtruder = 0;
int lastExtruder = -1;
int tempExtruder = -1;

int seenCommand = 0;
int prevCommand = 0;

int loaderMode = 2;  //(0= direct drive, 1=loader/unloader, 2=loader/unloader with press)

long triggerTime = 300;
long pulseTime = (triggerTime / 2);

long distance = 10;

long unloadDistance = stepsPerRev * microSteps * distance;  // this is 10 revs - about 10"
long loadDistance   = unloadDistance * 1.1;           // this is 11 revs - about 11"

int address = 0;
byte value;

long idleCount = 0;
bool logoActive = false;
bool T0Loaded = false;
bool T1Loaded = false;
bool T2Loaded = false;
bool T3Loaded = false;


bool displayEnabled = false;
bool ioEnabled = false;

long randomNumber = 0;

// EEPROM addresses
#define EEPROM_EXTRUDER_ADDR 0  // Address to store current extruder

// Functions to save/load extruder from EEPROM
void saveCurrentExtruder(int extruder) {
  EEPROM.write(EEPROM_EXTRUDER_ADDR, extruder + 1); // +1 because EEPROM can't store -1
}

int loadSavedExtruder() {
  int saved = EEPROM.read(EEPROM_EXTRUDER_ADDR);
  return saved - 1; // -1 to restore original value (-1 means no extruder)
}

void setup()
{

  // CNC Shield doesn't have IO expander - filament sensors not available
  ioEnabled = false;

  // Serial output instead of OLED
  Serial.begin(9600);

  // Load saved extruder from EEPROM
  lastExtruder = loadSavedExtruder();
  if(lastExtruder >= 0 && lastExtruder <= 3) {
    Serial.print("Extrusor salvo na EEPROM: T");
    Serial.println(lastExtruder);
  } else {
    Serial.println("Nenhum extrusor salvo na EEPROM");
    lastExtruder = -1;
  }

  Serial.println("3DChameleon Mk4 - Arduino Uno CNC Shield");
  Serial.println("Ready!");

  seenCommand = 0;

  // Sets the two pins as Outputs
  pinMode(extEnable, OUTPUT);
  pinMode(extStep, OUTPUT);
  pinMode(extDir, OUTPUT);

  pinMode(selEnable, OUTPUT);
  pinMode(selStep, OUTPUT);
  pinMode(selDir, OUTPUT);

  // set up the button
  pinMode(trigger, INPUT_PULLUP);  // selector

  // CNC Shield doesn't need special pin configurations for I2C

  // lock the selector by energizing it
  digitalWrite(selEnable, HIGH);

  // make sure filament isn't blocked by gillotine
  connectGillotine();
  cutFilament();
  disconnectGillotine();

  prevCommand = 0;

}

int lastLoop = 0;

void loop()
{
  static uint32_t lastTime = 0;

  seenCommand = 0;
  idleCount++;

  // process button press
  if (digitalRead(trigger) == 0)
  {
    // Pequeno delay para compensar tempo de resposta do optoacoplador LED (~10-20ms)
    delay(15);

    Serial.println("TRIGGER DETECTADO!");
    idleCount = 0;
    logoActive = false;
    unsigned long nextPulse;
    unsigned long pulseCount = 0;
    unsigned long commandCount = 0;

    // keep counting (and pulsing) until button is released
    while (digitalRead(trigger) == 0)
    {
      if(pulseCount<pulseTime)
      {
        pulseCount++;
        displayCommand(pulseCount);
        if(pulseCount>1) vibrateMotor();
      }
      delay(400);  // each pulse is 400+ milliseconds apart
    }
    Serial.print("Total de pulsos detectados: ");
    Serial.println(pulseCount);
    processCommand(pulseCount); // ok... execute whatever command was caught (by pulse count)
    pulseCount = 0;
  }

  // updates IO block, duh!  No really, grabs the state of the sparkfun gpio expansion
  updateIOBlock();

  // each loop adds 50ms delay, so that gets added AFTER the command is processed before the next one can start
  delay(50);
}

// CNC Shield doesn't have filament sensors - simulate all as loaded
void updateIOBlock()
{
    // CNC Shield doesn't have individual filament sensors
    // Assume all filaments are loaded for compatibility
    T0Loaded = true;
    T1Loaded = true;
    T2Loaded = true;
    T3Loaded = true;
}

// display command for each button pulse
void displayCommand(long commandCount)
{

  switch(commandCount)
  {
  case 2:
    displayText(25, "    Switch to T0");
    break;
  case 3:
    displayText(25, "    Switch to T1");
    break;
  case 4:
    displayText(25, "    Switch to T2");
    break;
  case 5:
    displayText(25, "    Switch to T3");
    break;
  case 6:
    displayText(25, "    Home/Load T0");
    break;
  case 7:
    displayText(28, "    Unload/Home");
    break;
  case 8:
    displayText(50, "        Home");
    break;
  case 9:
    displayText(50, "        Next");
    break;
  case 10:
    displayText(40, "       Random");
    break;
  default:
    displayText(30, "     No Command");
    break;
  }
}

// execute the pulse count command
void processCommand(long commandCount)
{

  // select case for commands
  switch (commandCount)
  {
  case 2: // unload current, switch to #0, load
    displayText(30, "     T0 Selected");
    currentExtruder = 0;
    processMoves();
    displayText(35, "      Idle - T0");
    break;

  case 3: // unload current, switch to #1, load
    displayText(30, "     T1 Selected");
    currentExtruder = 1;
    processMoves();
    displayText(35, "      Idle - T1");
    break;

  case 4: // unload current, switch to #3, load
    displayText(30, "     T2 Selected");
    currentExtruder = 2;
    processMoves();
    displayText(35, "      Idle - T2");
    break;

  case 5: // unload current, switch to #4, load
    displayText(30, "     T3 Selected");
    currentExtruder = 3;
    processMoves();
    displayText(35, "      Idle - T3");
    break;

  case 6: //home and reload #1
    displayText(40, "      Homing...");
    homeSelector();
    displayText(15, "   Press to Load T0");
    gotoExtruder(0, 0);
    if(loaderMode>0)rotateExtruder(clockwise, loadDistance);
    if(loaderMode>0)gotoExtruder(0, 1);
    currentExtruder = 0;
    lastExtruder = 0;
    displayText(35, "      Idle - T0");
    break;

  case 7: // unload current and rehome selector
    displayText(30, "     Cutting...");
    connectGillotine();
    cutFilament();
    switch(lastExtruder)
    {
      case 0:
        displayText(10, "  Press to Unload T0");
        break;
      case 1:
        displayText(10, "  Press to Unload T1");
        break;
      case 2:
        displayText(10, "  Press to Unload T2");
        break;
      case 3:
        displayText(10, "  Press to Unload T3");
        break;
    }
    if(loaderMode>0)gotoExtruder((lastExtruder==3?2:lastExtruder+1),lastExtruder);
    if(lastExtruder<2)
    {
      if(loaderMode>0)rotateExtruder(counterclockwise, unloadDistance);
    }
    else
    {
      if(loaderMode>0)rotateExtruder(clockwise, unloadDistance);
    }
    disconnectGillotine();
    displayText(50, "        Idle");
    break;

  case 8:
    displayText(40, "     Homing...");
    homeSelector();
    displayText(50, "        Idle");
    break;

  case 9: // move to next available filament
    displayText(30, "     Cutting...");
    connectGillotine();
    cutFilament();
    displayText(30, "     Next Tool");
    currentExtruder++;
    if(currentExtruder==4)currentExtruder=0;
    processMoves();
    displayText(35, "        Idle");
    break;

  case 10: // move to a random filament
    displayText(30, "     Cutting...");
    connectGillotine();
    cutFilament();
    displayText(30, "    Random Tool");

    // select a random number
    randomNumber = random(0,2) + 1;

    // skip ahead that many tools
    for(long i=0; i<randomNumber; i++)
    {
      currentExtruder++;
      if(currentExtruder==4)currentExtruder=0;
    }
    processMoves();
    displayText(50, "        Idle");
    break;

  case 11: // clear EEPROM (special command for testing)
    displayText(30, "   Clearing EEPROM...");
    saveCurrentExtruder(-1);
    lastExtruder = -1;
    displayText(30, "   EEPROM Cleared");
    delay(1000);
    displayText(50, "        Idle");
    break;

  default:
    displayText(47, "       Clear");
    delay(200);

    displayText(50, "        Idle");
    break;
  }
}

// Serial output instead of OLED display
void displayText(int offset, String str)
{
  Serial.println("3DChameleon Mk4");
  Serial.println(str);
  Serial.println("Status: -  -  -  -");
  Serial.println();
}

// Debug function to test trigger input (removido para não poluir)

// real work is here
void processMoves()
{

  // make sure we have a real extruder selected
  if(lastExtruder>-1)
  {

    // if so, we need to cut the filament
    displayText(30, "     Cutting...");
    connectGillotine();
    cutFilament();

    // ok... then wait for the 2nd button press to unload it
    switch(lastExtruder)
    {
      case 0:
        displayText(10, "  Press to Unload T0");
        break;
      case 1:
        displayText(10, "  Press to Unload T1");
        break;
      case 2:
        displayText(10, "  Press to Unload T2");
        break;
      case 3:
        displayText(10, "  Press to Unload T3");
        break;
    }

    // roll over to first if on last
    if( loaderMode>0 ) gotoExtruder( ( lastExtruder==3 ? 2 : (lastExtruder+1)), lastExtruder);

    // this determines which direction to move the motor, 0-1 : counterclockwise, 2-3 : clockwise
    if(lastExtruder<2)
    {
      if(loaderMode>0)rotateExtruder(counterclockwise, unloadDistance);
    }
    else
    {
      if(loaderMode>0)rotateExtruder(clockwise, unloadDistance);
    }
  }
  else
  {
    lastExtruder = 0;
  }
  disconnectGillotine();

  // tell it to actually execute that command now
  gotoExtruder(lastExtruder, currentExtruder);

  // ok... filament unloaded, time to load the new... so tell the user
  switch(currentExtruder)
  {
    case 0:
      displayText(15, "   Press to Load T0");
      break;
    case 1:
      displayText(15, "   Press to Load T1");
      break;
    case 2:
      displayText(15, "   Press to Load T2");
      break;
    case 3:
      displayText(15, "   Press to Load T3");
      break;
  }

  // same (but inversed) logic for motor direction
  if(currentExtruder<2)
  {
    if(loaderMode>0)rotateExtruder(clockwise, loadDistance);
  }
  else
  {
    if(loaderMode>0)rotateExtruder(counterclockwise, loadDistance);
  }

  // if we're loading, then load it now
  if(loaderMode>0)gotoExtruder(currentExtruder, (currentExtruder==3?2:currentExtruder+1));

  // everybody remember where we parked!
  lastExtruder = currentExtruder;

  // Save current extruder to EEPROM
  saveCurrentExtruder(currentExtruder);
}


// this function simply moves from the currentCog to the targetCog is the best way
void gotoExtruder(int currentCog, int targetCog)
{

  int newCog = targetCog - currentCog;

  // ok... which way
  int newDirection = counterclockwise;
  if(newCog<0)
  {
    // we need to move the other way
    newDirection = clockwise;

    //and since we know we went too far... let's go the other way in steps as well
    newCog = currentCog - targetCog;
  }

  // if we're already on the current cog, then do nothing
  if(newCog > 0)
  {
    // advance tool targetCog times
    for(int i=0; i<newCog; i++)
    {
      rotateSelector(newDirection, (stepsPerRev / 4) * microSteps);
    }
  }
}

// move the extruder motor in a specific direction for a specific distance (unless it's a "until button is not pressed")
void rotateExtruder(bool direction, long moveDistance)
{
  // note to bill:  make this acecelerate so it's very fast!!!

  digitalWrite(extEnable, LOW);  // lock the motor
  digitalWrite(extDir, direction); // Enables the motor to move in a particular direction
  const int fastSpeed = speedDelay/2; // double time

  // this is depricated right now... might bring it back in the future
  if(loaderMode==1)
  {

    // Makes 50 pulses for making one full cycle rotation
    for (long x = 0; x < (moveDistance-1); x++)
    {
      // this is how we pulse the motor to get it to step
      digitalWrite(extStep, HIGH);
      delayMicroseconds(fastSpeed);
      digitalWrite(extStep, LOW);
      delayMicroseconds(fastSpeed);
    }

  }

  if(loaderMode==2)
  {

   // keep waiting until button is pressed
   while (digitalRead(trigger) != 0)
    {
      delay(50);
    }

    // Move while button is pressed
    while (digitalRead(trigger) == 0)
    {

      // this is how we pulse the motor to get it to step
      digitalWrite(extStep, HIGH);
      delayMicroseconds(fastSpeed);
      digitalWrite(extStep, LOW);
      delayMicroseconds(fastSpeed);
    }
  }
  // ok, done pressing button, so make sure we're not energized (high is no, low is yes)
  digitalWrite(extEnable, HIGH);
}

// similar to extruder, but only stepping 50 (of 200) at a time
void rotateSelector(bool direction, int moveDistance)
{

  // while we are at it... can we make this faster using the magic you invented above?

  digitalWrite(selEnable, LOW); // lock the selector
  digitalWrite(selDir, direction); // Enables the motor to move in a particular direction

    // Makes 50 pulses for making one full cycle rotation
    for (int x = 0; x < (moveDistance-1); x++)
    {
      digitalWrite(selStep, HIGH);
      delayMicroseconds(speedDelay);
      digitalWrite(selStep, LOW);
      delayMicroseconds(speedDelay);
    }
}

// this cycles the servo between two positions
void cutFilament() {
  digitalWrite(selEnable, LOW); // disable stepper so we have power!
  if(reverseServo==false)
  {
    openGillotine();
    closeGillotine();
  }
  else
  {
    closeGillotine();
    openGillotine();
  }
  digitalWrite(selEnable, HIGH);
}

// enable the servo
void connectGillotine()
{
  filamentCutter.attach(SERVO_PIN);
}

// disable the servo - so it doesn't chatter when not in use
void disconnectGillotine()
{
  filamentCutter.detach();
}

// cycle servo from 135 and 180
void openGillotine()
{
    for (int pos = 135; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    filamentCutter.write(pos);              // tell servo to go to position in variable 'pos'
    delayMicroseconds(25000);                       // waits 15ms for the servo to reach the position
  }
  //filamentCutter.write(3.5);       // tell servo to go to position in variable 'pos'
  delay(50);                       // waits 15ms for the servo to reach the position
}

// reverse cycle servo from 180 back to 135
void closeGillotine()
{
  for (int pos = 180; pos >= 135; pos -= 1) { // goes from 180 degrees to 0 degrees
    filamentCutter.write(pos);              // tell servo to go to position in variable 'pos'
    delayMicroseconds(25000);                       // waits 15ms for the servo to reach the position
  }
  delay(50);                       // waits 15ms for the servo to reach the position
}

// rotate the selector clockwise too far from 4, so it'll grind on the bump stop
void homeSelector()
{
  // rotate counter clockwise to hard stop
  rotateSelector(clockwise, stepsPerRev * microSteps);

  // move just slightly to extruder 1 (this backs off a little from the hard stop)
  rotateSelector(counterclockwise, defaultBackoff * microSteps);

 currentExtruder = 0;
 lastExtruder = -2;

 // Save home position to EEPROM (no extruder selected)
 saveCurrentExtruder(-1);

}

// buzz buzz buzz
void vibrateMotor()
{
  // oscillate selector 1 time
  rotateSelector(clockwise, 2 * 16);
  rotateSelector(!clockwise, 2 * 16);
}