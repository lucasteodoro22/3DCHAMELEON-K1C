
// December 2024 - Adapted for Arduino Uno + CNC Shield with Serial Communication

/* 3DChameleon Mk4.1 Firmware - Arduino Uno CNC Shield Version with Serial Interface

Adapted for Arduino Uno with CNC Shield v3 for Klipper integration
- Z-axis: Extruder motor (filament feeding)
- X-axis: Selector motor (tool selection)
- Serial: Communication with Klipper (9600 baud)
- OLED: Removed (using Serial output)
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

Serial Commands (sent from Klipper):

SELECT Commands (tool selection only):
T0 - Select T0 (move selector to T0 position only)
T1 - Select T1 (move selector to T1 position only)
T2 - Select T2 (move selector to T2 position only)
T3 - Select T3 (move selector to T3 position only)

LOAD/UNLOAD Commands (work on current tool position):
LOAD <distance> - Load filament into current tool position with specified distance (mm)
UNLOAD <distance> - Unload filament from current tool position with specified distance (mm)

System Commands:
HOME - Home selector only
IDLE - Move selector to idle position


Posições Idle 
T0 → idle em T2 
T1 → idle em T3
T2 → idle em T0
T3 → idle em T1
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

// Servo pin (compatible with Arduino Uno)
#define SERVO_PIN 11

// Filament sensor pin (NC - Normally Closed)
#define FILAMENT_SENSOR_PIN A3

const int counterclockwise = HIGH;
const int clockwise = !counterclockwise;

const int stepsPerRev = 200;
const int microSteps = 16;
const int speedDelay = 170;     // Original: 170, Faster: 85, Fastest: 40

const int defaultBackoff = 10;

Servo filamentCutter;  // create servo object to control a servo
int cutterPos = 0;    // variable to store the servo position
bool reverseServo = true;

int currentExtruder = -1;
int nextExtruder = 0;
int lastExtruder = -1;
int tempExtruder = -1;


int loaderMode = 1;  //(0= direct drive, 1=loader/unloader - automatic mode)

// Serial communication variables
String serialBuffer = "";
bool commandReceived = false;

// Distance settings
long distance = 10;

// Steps per mm calculation (calibrated for your setup)
const float STEPS_PER_MM = 151.0; // Calibrated: 100mm command = 51mm actual -> need ~2x more steps

long unloadDistance = stepsPerRev * microSteps * distance;  // this is 10 revs - about 10"
long loadDistance   = unloadDistance * 1.1;           // this is 11 revs - about 11"

int address = 0;
byte value;


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

  // Serial output instead of OLED
  Serial.begin(9600);

  Serial.println("3DChameleon Mk4 - Arduino Uno CNC Shield");
  Serial.println("Serial Interface Ready!");

  // Sets the two pins as Outputs
  pinMode(extEnable, OUTPUT);
  pinMode(extStep, OUTPUT);
  pinMode(extDir, OUTPUT);

  pinMode(selEnable, OUTPUT);
  pinMode(selStep, OUTPUT);
  pinMode(selDir, OUTPUT);

  // Filament sensor setup (NC - Normally Closed, INPUT_PULLUP)
  pinMode(FILAMENT_SENSOR_PIN, INPUT_PULLUP);

  // Initialize extruder motor as DISABLED (HIGH = disabled)
  digitalWrite(extEnable, HIGH); // Ensure extruder motor starts disabled

  // lock the selector by energizing it - keep locked 100% of time
  digitalWrite(selEnable, LOW); // Always locked

  // Auto-home on startup to establish known position
  Serial.println("Homing...");
  homeSelector();
  Serial.println("Homing concluido");

  // Load saved extruder from EEPROM and position to it
  lastExtruder = loadSavedExtruder();
  if(lastExtruder >= 0 && lastExtruder <= 2) {  // Only T0, T1, T2 available

    // Move to the saved extruder position
    currentExtruder = lastExtruder;
    gotoExtruder(0, currentExtruder);  // From home position (0) to saved position

    // Go to idle position for the active tool
    moveToIdle();

    Serial.print("T");
    Serial.print(lastExtruder);
    Serial.println(" carregado e em idle");
  } else {
    Serial.println("Permanecendo em T0");
    currentExtruder = 0;
    lastExtruder = 0;
    saveCurrentExtruder(currentExtruder);
  }

  Serial.println("Commands:");
  Serial.println("T0, T1, T2, HOME, IDLE");
  Serial.println("LOAD <mm> (Load apos o sensor)");
  Serial.println("UNLOAD <mm> (Unload)");
  Serial.println();

}

void loop()
{
  // Check for serial commands from Klipper
  while (Serial.available() > 0)
  {
    char incomingChar = Serial.read();
    if (incomingChar == '\n' || incomingChar == '\r')
    {
      if (serialBuffer.length() > 0)
      {
        commandReceived = true;
        break;
      }
    }
    else
    {
      serialBuffer += incomingChar;
    }
  }

  // Process received command
  if (commandReceived)
  {
    Serial.print("Comando: ");
    Serial.println(serialBuffer);

    processSerialCommand(serialBuffer);
    serialBuffer = "";
    commandReceived = false;
  }

  // small delay to prevent overwhelming the processor
  delay(10);
}

// Process serial commands from Klipper
void processSerialCommand(String command)
{
  command.trim(); // Remove any whitespace
  command.toUpperCase(); // Convert to uppercase for case-insensitive comparison

  bool isLoadUnloadCommand = false;
  int toolNumber = -1;
  bool isLoad = false;
  float distance_mm = 0.0;

  // Check for LOAD/UNLOAD commands with distance parameter
  if (command.startsWith("LOAD ") || command.startsWith("UNLOAD ")) {
    isLoadUnloadCommand = true;

    // Determine if LOAD or UNLOAD
    if (command.startsWith("LOAD ")) {
      isLoad = true;
    } else if (command.startsWith("UNLOAD ")) {
      isLoad = false;
    }

    // Extract distance parameter
    int spaceIndex = command.indexOf(' ');
    String distanceStr = command.substring(spaceIndex + 1);
    distance_mm = distanceStr.toFloat();

    if (distance_mm <= 0) {
      Serial.println("Invalid distance");
      return;
    }

    // Check if a tool is currently selected
    if (currentExtruder < 0) {
      Serial.println("No tool selected. Use T0, T1, T2 or T3 first");
      return;
    }

    // Execute LOAD/UNLOAD command on current tool
    executeLoadUnload(currentExtruder, isLoad, distance_mm);
    return;
  }

  // Tool selection commands (just move selector)
  if (command == "T0") {
    selectTool(0);
    return;
  }
  else if (command == "T1") {
    selectTool(1);
    return;
  }
  else if (command == "T2") {
    selectTool(2);
    return;
  }
  else if (command == "HOME") {
    homeSelector();
    Serial.println("Selector homed");
    // Note: homeSelector() already updates currentExtruder to 0
    return;
  }
  else if (command == "IDLE") {
    moveToIdle();
    return;
  }else {
    Serial.println("ERRO: Comando desconhecido");
    return;
  }
}

// Tool selection - just move selector to position
void selectTool(int toolNumber)
{
  Serial.print("Selecionando T");
  Serial.println(toolNumber);

  // Move selector to the tool position
  // From current position to target position
  int fromPos = (currentExtruder >= 0) ? currentExtruder : 0;
  gotoExtruder(fromPos, toolNumber);

  currentExtruder = toolNumber;
  lastExtruder = toolNumber;

  // Save to EEPROM
  saveCurrentExtruder(currentExtruder);

  Serial.print("Tool T");
  Serial.print(toolNumber);
  Serial.println(" selecionado");
}

// Move selector to idle position (same as original code after tool change)
void moveToIdle()
{

  // Calculate idle position based on active tool (lastExtruder)
  int idlePos;
  if (lastExtruder == 0) {
    idlePos = 2;  // T0 -> idle at T2
  } else if (lastExtruder == 1) {
    idlePos = 3;  // T1 -> idle at T3
  } else if (lastExtruder == 2) {
    idlePos = 0;  // T2 -> idle at T0
  } else if (lastExtruder == 3) {
    idlePos = 1;  // T3 -> idle at T1
  } else {
    idlePos = 2;  // Default fallback to T2
  }

  // Only move if not already at the correct idle position
  if (currentExtruder != idlePos) {
    gotoExtruder(currentExtruder, idlePos);

    // Update currentExtruder to idle position for tracking
    // NOTE: We keep lastExtruder as the active tool, currentExtruder tracks physical position
    currentExtruder = idlePos;
    // lastExtruder stays as the active tool (not updated to idle position)

    // Save current position to EEPROM (physical position for tracking)
    saveCurrentExtruder(currentExtruder);

    Serial.println("Posicao IDLE alcançada");
  }

  
}

// Load filament until sensor is triggered, then continue with specified distance
long loadUntilSensor(bool direction, float additionalDistance_mm)
{
  Serial.println("Carregando filamento ate o sensor...");

  // Ensure motor is disabled before enabling (safety check)
  digitalWrite(extEnable, HIGH);
  digitalWrite(extEnable, LOW);  // lock the motor
  digitalWrite(extDir, direction); // Set direction

  long stepsToSensor = 0;
  long maxSteps = (long)(2000.0 * STEPS_PER_MM); // Maximum 2000mm to prevent infinite loop
  bool sensorTriggered = false;

  // Move slowly until sensor is triggered (NC sensor goes HIGH when filament blocks it)
  const int sensorSpeed = speedDelay/5 ; // Slower speed for sensor detection

  while (stepsToSensor < maxSteps && !sensorTriggered) {
    // Check sensor state (NC = HIGH when filament is present)
    if (digitalRead(FILAMENT_SENSOR_PIN) == HIGH) { // Sensor triggered (filament detected)
      sensorTriggered = true;
      Serial.println("Sensor de filamento detectado!");
      break;
    }

    // Move one step
    digitalWrite(extStep, HIGH);
    delayMicroseconds(sensorSpeed);
    digitalWrite(extStep, LOW);
    delayMicroseconds(sensorSpeed);

    stepsToSensor++;
  }

  if (!sensorTriggered) {
    Serial.println("ERRO: Sensor nao detectado apos movimento maximo!");
    digitalWrite(extEnable, HIGH);
    return 0;
  }

  Serial.print("Filamento posicionado no sensor. Movimentacao ate sensor: ");
  Serial.print((float)stepsToSensor / STEPS_PER_MM);
  Serial.println("mm");

  // Now continue with the specified additional distance
  if (additionalDistance_mm > 0) {
    long additionalSteps = (long)(additionalDistance_mm * STEPS_PER_MM);
    Serial.print("Continuando com distancia adicional: ");
    Serial.print(additionalDistance_mm);
    Serial.println("mm");

    // Set back to original direction
    digitalWrite(extDir, direction);

    // Move the additional distance
    for (long i = 0; i < additionalSteps; i++) {
      digitalWrite(extStep, HIGH);
      delayMicroseconds(sensorSpeed);
      digitalWrite(extStep, LOW);
      delayMicroseconds(sensorSpeed);
    }
  }

  // Disable motor
  digitalWrite(extEnable, HIGH);

  long totalSteps = stepsToSensor + (long)(additionalDistance_mm * STEPS_PER_MM);
  Serial.print("Movimento total concluido: ");
  Serial.print((float)totalSteps / STEPS_PER_MM);
  Serial.println("mm");

  return totalSteps;
}

// Execute LOAD/UNLOAD commands with specific distance
void executeLoadUnload(int toolNumber, bool isLoad, float distance_mm)
{
  Serial.print(isLoad ? "LOAD_T" : "UNLOAD_T");
  Serial.print(toolNumber);
  Serial.print(" ");
  Serial.print(distance_mm);
  Serial.println("mm");

  // If this is different from current tool, move to the tool first
  if (currentExtruder != toolNumber) {
    Serial.println("Movendo para o tool correto...");
    gotoExtruder(currentExtruder >= 0 ? currentExtruder : 0, toolNumber);
    currentExtruder = toolNumber;
  }

  // Determine direction based on tool number and operation
  bool direction;
  if (isLoad) {
    // Load direction: clockwise for T0,T1, counterclockwise for T2,T3
    direction = (toolNumber < 2) ? clockwise : counterclockwise;
    Serial.println("Executando LOAD com sensor...");

    // Use sensor-based loading for LOAD operations
    loadUntilSensor(direction, distance_mm);

  } else {
    // Unload direction: opposite of load
    direction = (toolNumber < 2) ? counterclockwise : clockwise;
    Serial.println("Executando UNLOAD...");

    // Use traditional distance-based movement for UNLOAD
    long steps = (long)(distance_mm * STEPS_PER_MM);
    rotateExtruder(direction, steps);
  }

  // Update last extruder
  lastExtruder = toolNumber;

  // Save to EEPROM
  saveCurrentExtruder(currentExtruder);

  Serial.println("Operacao concluida");
}

// real work is here
void processMoves()
{
  // make sure we have a real extruder selected
  if(lastExtruder>-1)
  {
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

  // tell it to actually execute that command now
  gotoExtruder(lastExtruder, currentExtruder);

  // Load the new filament
  Serial.println("Loading new filament...");

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

  // Ensure motor is disabled before enabling (safety check)
  digitalWrite(extEnable, HIGH);
  digitalWrite(extEnable, LOW);  // lock the motor
  digitalWrite(extDir, direction); // Enables the motor to move in a particular direction

  // Speed parameters for ramping
  const int minSpeed = speedDelay;       // Starting speed (slower) - 170µs
  const int maxSpeed = speedDelay/6;     // Maximum speed (faster) - ~28µs (6x mais rápido)

  // Automatic loading/unloading mode
  if(loaderMode==1)
  {
    // Check if this is a short move (< 100mm) - use slow constant speed
    float moveDistanceMM = (float)moveDistance / STEPS_PER_MM;
    if (moveDistanceMM < 100.0) {
      // Short move - use constant slow speed for precision
      Serial.print("Movimento curto: "); Serial.print(moveDistanceMM); Serial.println("mm - modo devagar");
      for (long x = 0; x < (moveDistance-1); x++)
      {
        digitalWrite(extStep, HIGH);
        delayMicroseconds(minSpeed);  // Use minimum speed (slowest)
        digitalWrite(extStep, LOW);
        delayMicroseconds(minSpeed);
      }
      Serial.println("Movimento curto concluido");
    }
    else {
      // Long move - use ramping (5% accel, 90% constant, 5% decel)
      long accelEnd = moveDistance * 0.05;   // 5% for acceleration
      long decelStart = moveDistance * 0.95; // Start deceleration at 95%
      // Long move - use percentage-based ramping
      Serial.print("Rampa %: accelEnd="); Serial.print(accelEnd);
      Serial.print(" (5%) decelStart="); Serial.print(decelStart);
      Serial.print(" (95%) total="); Serial.println(moveDistance);

      for (long x = 0; x < (moveDistance-1); x++)
      {
        int currentSpeed;

        if (x < accelEnd) {
          // Acceleration ramp: 20% of movement - gradually increase speed
          float rampProgress = (float)x / accelEnd;  // 0.0 to 1.0 over 20% of distance
          currentSpeed = minSpeed - (int)((minSpeed - maxSpeed) * rampProgress);
        }
        else if (x >= decelStart) {
          // Deceleration ramp: last 20% of movement - gradually decrease speed
          float rampProgress = (float)(x - decelStart) / (moveDistance - decelStart);  // 0.0 to 1.0 over last 20%
          currentSpeed = maxSpeed + (int)((minSpeed - maxSpeed) * rampProgress);
        }
        else {
          // Constant maximum speed in the middle 60%
          currentSpeed = maxSpeed;
        }

        // Ensure speed stays within bounds
        currentSpeed = constrain(currentSpeed, maxSpeed, minSpeed);

        // Pulse the motor
        digitalWrite(extStep, HIGH);
        delayMicroseconds(currentSpeed);
        digitalWrite(extStep, LOW);
        delayMicroseconds(currentSpeed);
      }

      Serial.println("Rampa % concluida");
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

// rotate the selector clockwise too far from 4, so it'll grind on the bump stop
void homeSelector()
{
  // rotate counter clockwise to hard stop
  rotateSelector(clockwise, stepsPerRev * microSteps);

  // move just slightly to extruder 1 (this backs off a little from the hard stop)
  rotateSelector(counterclockwise, defaultBackoff * microSteps);

  // After homing, we're at a known position (backoff from hard stop = position 0 = T0)
  currentExtruder = 0;  // We're at T0 position
  lastExtruder = 0;     // Consider this as a valid position

  // Save current position to EEPROM (T0 = position 0)
  saveCurrentExtruder(currentExtruder);
}