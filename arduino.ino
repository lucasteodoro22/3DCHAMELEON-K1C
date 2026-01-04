// November 28, 2024

/* 3DChameleon Mk4.1 Firmware - Adaptado para CNC Shield

Copyright 2024 William J. Steele

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

=== CONFIGURAÇÃO SERVO 360° CONTÍNUO ===
- Modelo: MG996R 360° (rotação contínua)
- Pino: Arduino 11 (SPINDLE_PWM da CNC Shield)
- Conversão: Movimento angular → tempo de rotação
- Movimento: 45° = 1.1 segundos por direção
- PWM: 1300µs (sentido A), 1700µs (sentido B), 1500µs (parado)
- Segurança: Mesmo tempo para ida/volta garante retorno preciso

Adaptado para CNC Shield com Driver DVR8825:
- Motor Extruder: Eixo Z (Step=4, Dir=7, Enable=8)
- Motor Selector: Eixo X (Step=2, Dir=5, Enable=8)
- PC817C Optocoupler: A3 (com debounce para LED delay)
- Driver: DVR8825 (Enable ativo-baixo: LOW=habilita, HIGH=desabilita)

Single Button Press Commands (count pulses of selector)

#1 - 1
#2 - 2 (Switch to T0)
#3 - 3 (Switch to T1)
#4 - 4 (Switch to T2)
#5 - 5 (Switch to T3)
#6 - Home and Load
#7 - Unload Last and Home
#8 - Home
#9 - Next Filament
#10 - Random Filament
#11 - Load T0 (no cut - for unused filaments)
#12 - Load T1 (no cut - for unused filaments)
#13 - Load T2 (no cut - for unused filaments)
#14 - Load T3 (no cut - for unused filaments)
#15 - Unload T0 (no cut - for unused filaments)
#16 - Unload T1 (no cut - for unused filaments)
#17 - Unload T2 (no cut - for unused filaments)
#18 - Unload T3 (no cut - for unused filaments)

*/

/*
 * CONFIGURAÇÕES PARA DVR8825:
 * ===========================
 *
 * 1. PINOS DE CONTROLE:
 *    - Enable: LOW = motor HABILITADO, HIGH = motor DESABILITADO (ativo-baixo)
 *    - Step: Pulso HIGH->LOW para cada passo
 *    - Direction: HIGH ou LOW para direção
 *
 * 2. MICROSTEPPING:
 *    - Configurar jumpers M0,M1,M2 no DVR8825
 *    - Código configurado para 1/8 microstepping (8)
 *    - SUA CONFIGURAÇÃO: M0=ON, M1=ON, M2=OFF
 *    - Outras opções: FULL=1(OFF,OFF,OFF), HALF=2(ON,OFF,OFF), 1/4=4(OFF,ON,OFF),
 *                     1/8=8(ON,ON,OFF), 1/16=16(OFF,OFF,ON), 1/32=32(ON,OFF,ON)
 *
 * 3. AJUSTES DE PERFORMANCE:
 *    - speedDelay: 170μs (ajustar para velocidade desejada)
 *    - Corrente: Ajustar potenciômetro no DVR8825 (não exceder rating do motor)
 *    - Resfriamento: DVR8825 aquece, garantir ventilação
 *
 * 4. CONEXÕES NO CNC SHIELD:
 *    - Motor X: Step=2, Dir=5, Enable=8 (Selector)
 *    - Motor Z: Step=4, Dir=7, Enable=8 (Extruder)
 *    - PC817C Optocoupler: A3 (com debounce para evitar pulsos fantasmas)
 */

#include <SSD1306Ascii.h> //i2C OLED
#include <SSD1306AsciiWire.h> //i2C OLED
#include <SparkFunSX1509.h> // sparkfun i/o expansion board - used for additional filament sensors as well as communications with secondary boards
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <EEPROM.h> // Para armazenar estado persistentemente
 
 #define SCREEN_WIDTH 128 // OLED display width, in pixels
 #define SCREEN_HEIGHT 64 // OLED display height, in pixels
 
 /**
  * Made with Marlin Bitmap Converter
  * https://marlinfw.org/tools/u8glib/converter.html
  *
  * This bitmap from 128x64 pasted image
  */
 #pragma once
 
 const unsigned char bitmap_logo[] PROGMEM = {
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00001000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00110000,B00001100,B00100000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01111100,B00001110,B00110000,B00011100,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01111111,B00011111,B01111000,B00011111,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01111111,B10011111,B11111000,B00011111,B10000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01111111,B11011111,B11110110,B00011111,B11000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01111111,B10000000,B00000110,B00011111,B11100000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B11111100,B01111101,B11111111,B11110010,B00111111,B11110000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B11111111,B00001111,B11111111,B11111110,B00111111,B11111000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01111111,B00111111,B11111111,B11111111,B00111111,B11111100,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01111110,B00111111,B11111111,B11111111,B01111111,B11111100,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01111101,B11111111,B11111111,B11111111,B01111111,B11111110,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01111011,B11111111,B11111111,B11111110,B11111111,B11111110,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01100111,B11101111,B11111111,B11111111,B11111111,B11111110,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000111,B10001111,B11110111,B11111111,B11111111,B11111111,B11111111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00001111,B10111111,B11111011,B11111111,B11111101,B11111110,B00011111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00001111,B10111111,B11111001,B11111111,B11111111,B11111100,B00001111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00001111,B01111111,B11111101,B11111111,B11111111,B11111001,B11110111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000111,B11111111,B11111110,B11111111,B11111111,B11111011,B11110011,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000010,B11111111,B11111110,B11111111,B11111111,B11111011,B10111011,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00111100,B11111111,B11111111,B01111111,B11111111,B11111011,B11110011,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B01111101,B11111111,B11111111,B01111111,B11111111,B11111001,B11110111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B01111101,B11111111,B11111111,B10111111,B11111101,B11111100,B11101111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00111101,B11111111,B11111111,B10000000,B00000000,B11111111,B11011111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00011001,B11111111,B11111111,B10000000,B00000000,B11111111,B11111111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00001011,B11111111,B11111111,B01100000,B00000000,B01111111,B11111111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000001,B11111111,B11111110,B11111000,B00000000,B00111111,B11111111,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00011010,B01111111,B11111101,B11111110,B00000000,B00001111,B11111110,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B01111011,B11100111,B11111011,B11111110,B00000000,B00000111,B11111110,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B11111011,B11111100,B01110000,B01111111,B00000000,B00000001,B11111110,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B01111011,B11111111,B00000000,B01111111,B10000000,B00000000,B01111100,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00011011,B11111111,B11000000,B11111111,B11111000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00001011,B11111111,B11100001,B11111111,B11111110,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00111011,B11111111,B11100001,B11111111,B11111111,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B01111011,B11111111,B11100011,B11111111,B11110011,B10000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B11111011,B11111111,B11100000,B00000001,B11111100,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B01111011,B11111111,B11100000,B00000000,B01111110,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000011,B11111111,B11100000,B00000000,B00011110,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000101,B11111111,B11100000,B00000000,B00000010,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00001101,B11111111,B11100000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00011111,B11111111,B11100000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00011110,B11111111,B10000000,B00000111,B11000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000001,B11111111,B01110000,B11111111,B11100000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000011,B01111111,B11111111,B11111111,B11100000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000011,B11101111,B11111111,B11111111,B11100000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000111,B10011111,B11111111,B11111100,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00011111,B11111111,B10000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01011111,B11111111,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01101111,B11111111,B10000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B01110111,B11111111,B10000000,B00000000,B01100000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000011,B11111111,B11111111,B11111111,B11000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00001110,B11111111,B11111111,B11111111,B10000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00001110,B01111111,B11111111,B11111111,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00001100,B11011111,B11111111,B11111100,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B11011001,B11111111,B11110000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00011001,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000001,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
   B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000
 };
 
 
 // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
 #define OLED_RESET  -1 // Reset pin # (or -1 if sharing Arduino reset pin)
 #define OLED_I2C_ADDRESS 0x3C
 SSD1306AsciiWire oled;
 
 // define sparkfun io expansion
 const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C address
 #define SX1509_FILAMENT_0 0
 #define SX1509_FILAMENT_1 1
 #define SX1509_FILAMENT_2 2
 #define SX1509_FILAMENT_3 3
 #define SX1509_OUTPUT 4
 SX1509 io;                        // Create an SX1509 object to be used throughout
 
 
 // defines pins numbers - CNC Shield (Motor no X e Z)
 #define extEnable 8    // Z axis enable (compartilhado)
 #define extStep 4      // Z axis step
 #define extDir 7       // Z axis direction
 
 #define selEnable 8    // X axis enable (compartilhado)
 #define selStep 2      // X axis step
 #define selDir 5       // X axis direction
 
 #define trigger A3     // PC817C optocoupler (LOW=ativado, HIGH=desativado)
 #define s_limit A4
 #define filament A5
 
 const int counterclockwise = HIGH;
 const int clockwise = !counterclockwise;
 
 const int stepsPerRev = 200;    // Passos por revolução do motor (200 para motores NEMA)
 const int microSteps = 8;      // Microstepping do DVR8825 (1/8 - jumpers M0=ON, M1=ON, M2=OFF)
 const int speedDelay = 170;    // Ajustar conforme necessário: menor = mais rápido, maior = mais lento
 
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
 //int sensorEnabled = 0;
 
 long randomNumber = 0;
 
void setup()
{

  Serial.begin(9600); // Inicializar comunicação serial para debug
  Wire.begin(); //start i2C
  Wire.setClock(400000L); //set clock
 
   // set up IO expander
   if (io.begin(SX1509_ADDRESS) == true)
   {
     io.pinMode(SX1509_FILAMENT_0, INPUT_PULLUP);
     io.pinMode(SX1509_FILAMENT_1, INPUT_PULLUP);
     io.pinMode(SX1509_FILAMENT_2, INPUT_PULLUP);
     io.pinMode(SX1509_FILAMENT_3, INPUT_PULLUP);
     io.pinMode(SX1509_OUTPUT, OUTPUT);
     ioEnabled = true;
   }
 
   // enable OLED display
   oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
 
   // wait for it to start up
   delay(50);
     
 
   // welcome screen
   oled.setFont(Adafruit5x7);
   oled.clear(); //clear display
   oled.println("");
   oled.println("       Welcome!"); //print a welcome message  
   oled.println("");
   oled.println("    3DChameleon Mk4"); //print a welcome message
   oled.println("");
   oled.println("         Pro");
   delay(3000);
 
   displayText(0, "       Ready!");
   
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
 
   // CNC Shield: A4 e A5 são usados para I2C (OLED e SX1509)
   // Nota: Verificar se há conflito com funções do CNC Shield nestes pinos
   pinMode(s_limit, OUTPUT);
   pinMode(filament, OUTPUT); 
 
  // lock the selector by energizing it
  digitalWrite(selEnable, HIGH);

  // CARREGAR ESTADO PERSISTENTE DA EEPROM
  // Endereço 0: currentExtruder, Endereço 1: lastExtruder
  int savedCurrent = EEPROM.read(0);
  int savedLast = EEPROM.read(1);

  // Verificar se os valores são válidos (0-3 para extrusores)
  if(savedCurrent >= 0 && savedCurrent <= 3) {
    currentExtruder = savedCurrent;
  } else {
    currentExtruder = 0; // Valor padrão se inválido
  }

  if(savedLast >= -2 && savedLast <= 3) { // -2 é posição de home
    lastExtruder = savedLast;
  } else {
    lastExtruder = -2; // Valor padrão se inválido
  }

  // REMOVIDO: make sure filament isn't blocked by gillotine
  // connectGillotine();
  // cutFilament();  // ← CAUSAVA CORTE NA INICIALIZAÇÃO
  // disconnectGillotine();

  // Servo será inicializado apenas quando necessário (connectGillotine)
  // Inicialização mínima sem movimento para evitar primeira falha
  filamentCutter.attach(11);
  filamentCutter.writeMicroseconds(1500); // Centro/parado
  delay(100);
  filamentCutter.detach(); // Desconectar até precisar

  prevCommand = 0;

}
 
 int lastLoop = 0;
 
 void loop()
 {
   static uint32_t lastTime = 0;
 
   seenCommand = 0;
   idleCount++;
 
   // process PC817C optocoupler signal (LOW = ativado, HIGH = desativado)
   if (digitalRead(trigger) == 0)
   {
     idleCount = 0;
     logoActive = false;
     unsigned long nextPulse;
     unsigned long pulseCount = 0;
     unsigned long commandCount = 0;
 
       // CONTAGEM DE PULSOS: Sistema conta tempo de pressão, não pulsos reais
     // - Cada "pulso" = 400ms de pressão contínua no botão
     // - Pressão rápida pode ser contada como múltiplos pulsos
     // - Ex: 1 pressão rápida = ~2 pulsos, dependendo da duração
     // - Para 1 pulso real: pressione por exatamente 400ms
     // - Para 2 pulsos: pressione por 800ms (2x400ms)
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
 
     // Debounce delay for PC817C to prevent ghost pulses when LED turns off
     delay(100);  // Wait for optocoupler LED to fully turn off
 
     processCommand(pulseCount); // ok... execute whatever command was caught (by pulse count)
     pulseCount = 0;
   }
 
   // updates IO block, duh!  No really, grabs the state of the sparkfun gpio expansion
   updateIOBlock();
 
   // each loop adds 50ms delay, so that gets added AFTER the command is processed before the next one can start
   delay(50);
 }
 
 // read the sparkfun SX1509 io
 void updateIOBlock()
 {
     if(ioEnabled)
     {
       T0Loaded = io.digitalRead(SX1509_FILAMENT_0);
       T1Loaded = io.digitalRead(SX1509_FILAMENT_1);
       T2Loaded = io.digitalRead(SX1509_FILAMENT_2);
       T3Loaded = io.digitalRead(SX1509_FILAMENT_3);
     }
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
   case 11:
     displayText(40, "    Load T0*");
     break;
   case 12:
     displayText(40, "    Load T1*");
     break;
   case 13:
     displayText(40, "    Load T2*");
     break;
   case 14:
     displayText(40, "    Load T3*");
     break;
   case 15:
     displayText(35, "   Unload T0*");
     break;
   case 16:
     displayText(35, "   Unload T1*");
     break;
   case 17:
     displayText(35, "   Unload T2*");
     break;
   case 18:
     displayText(35, "   Unload T3*");
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
    EEPROM.write(0, currentExtruder); // Salvar estado
    processMoves();
    displayText(35, "      Idle - T0");
    break;

  case 3: // unload current, switch to #1, load
    displayText(30, "     T1 Selected");
    currentExtruder = 1;
    EEPROM.write(0, currentExtruder); // Salvar estado
    processMoves();
    displayText(35, "      Idle - T1");
    break;

  case 4: // unload current, switch to #3, load
    displayText(30, "     T2 Selected");
    currentExtruder = 2;
    EEPROM.write(0, currentExtruder); // Salvar estado
    processMoves();
    displayText(35, "      Idle - T2");
    break;

  case 5: // unload current, switch to #4, load
    displayText(30, "     T3 Selected");
    currentExtruder = 3;
    EEPROM.write(0, currentExtruder); // Salvar estado
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
    EEPROM.write(0, currentExtruder); // Salvar estado
    EEPROM.write(1, lastExtruder);    // Salvar estado
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
     // Garantir que o servo seja reinicializado após unload/home
     connectGillotine();
     filamentCutter.writeMicroseconds(1500); // Centro/parado
     delay(200);
     disconnectGillotine();
     displayText(50, "        Idle");
     break;
 
   case 8:
     displayText(40, "     Homing...");
     homeSelector();
     // Garantir que o servo seja reinicializado após home
     connectGillotine();
     filamentCutter.writeMicroseconds(1500); // Centro/parado
     delay(200);
     disconnectGillotine();
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

   case 11: // Load T0 directly (no cutting - for unused filaments)
     displayText(30, "     Loading T0");
     displayText(15, "   Press to Load T0");
     gotoExtruder(lastExtruder, 0);
     if(loaderMode>0)rotateExtruder(clockwise, loadDistance);
     if(loaderMode>0)gotoExtruder(0, 1);
     currentExtruder = 0;
     lastExtruder = 0;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T0");
     break;

   case 12: // Load T1 directly (no cutting - for unused filaments)
     displayText(30, "     Loading T1");
     displayText(15, "   Press to Load T1");
     gotoExtruder(lastExtruder, 1);
     if(loaderMode>0)rotateExtruder(clockwise, loadDistance);
     if(loaderMode>0)gotoExtruder(1, 2);
     currentExtruder = 1;
     lastExtruder = 1;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T1");
     break;

   case 13: // Load T2 directly (no cutting - for unused filaments)
     displayText(30, "     Loading T2");
     displayText(15, "   Press to Load T2");
     gotoExtruder(lastExtruder, 2);
     if(loaderMode>0)rotateExtruder(counterclockwise, loadDistance);
     if(loaderMode>0)gotoExtruder(2, 3);
     currentExtruder = 2;
     lastExtruder = 2;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T2");
     break;

   case 14: // Load T3 directly (no cutting - for unused filaments)
     displayText(30, "     Loading T3");
     displayText(15, "   Press to Load T3");
     gotoExtruder(lastExtruder, 3);
     if(loaderMode>0)rotateExtruder(counterclockwise, loadDistance);
     if(loaderMode>0)gotoExtruder(3, 0);
     currentExtruder = 3;
     lastExtruder = 3;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T3");
     break;

   case 15: // Unload T0 directly (no cutting - for unused filaments)
     displayText(30, "    Unloading T0");
     displayText(10, "  Press to Unload T0");
     if(lastExtruder != 0) gotoExtruder(lastExtruder, 0);
     if(loaderMode>0)rotateExtruder(counterclockwise, unloadDistance);
     currentExtruder = 0;
     lastExtruder = 0;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T0");
     break;

   case 16: // Unload T1 directly (no cutting - for unused filaments)
     displayText(30, "    Unloading T1");
     displayText(10, "  Press to Unload T1");
     if(lastExtruder != 1) gotoExtruder(lastExtruder, 1);
     if(loaderMode>0)rotateExtruder(counterclockwise, unloadDistance);
     currentExtruder = 1;
     lastExtruder = 1;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T1");
     break;

   case 17: // Unload T2 directly (no cutting - for unused filaments)
     displayText(30, "    Unloading T2");
     displayText(10, "  Press to Unload T2");
     if(lastExtruder != 2) gotoExtruder(lastExtruder, 2);
     if(loaderMode>0)rotateExtruder(clockwise, unloadDistance);
     currentExtruder = 2;
     lastExtruder = 2;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T2");
     break;

   case 18: // Unload T3 directly (no cutting - for unused filaments)
     displayText(30, "    Unloading T3");
     displayText(10, "  Press to Unload T3");
     if(lastExtruder != 3) gotoExtruder(lastExtruder, 3);
     if(loaderMode>0)rotateExtruder(clockwise, unloadDistance);
     currentExtruder = 3;
     lastExtruder = 3;
     EEPROM.write(0, currentExtruder); // Salvar estado
     EEPROM.write(1, lastExtruder);    // Salvar estado
     displayText(35, "      Idle - T3");
     break;

   default:
     displayText(47, "       Clear");
     delay(200);

     displayText(50, "        Idle");
     break;
   }
 }
 
// just the routine to update the OLED and Serial console
void displayText(int offset, String str)
{

  // Display no console Serial
  Serial.println("=== 3DChameleon Mk4 ===");
  Serial.println(str);

  //if(displayEnabled){
    oled.clear();
    oled.println("");
    oled.println("   3DChameleon Mk4"); //print a welcome message
    oled.println("");
    oled.println("");
    oled.println(str);
    oled.println("");
    oled.println("");
    if(ioEnabled)
    {
      oled.print("     ");
      String statusStr = "     ";

      if(T0Loaded)
      {
        oled.print("0  ");
        statusStr += "0  ";
      }
      else
      {
        oled.print("+  ");
        statusStr += "+  ";
      }
      if(T1Loaded)
      {
        oled.print("1  ");
        statusStr += "1  ";
      }
      else
      {
        oled.print("+  ");
        statusStr += "+  ";
      }
      if(T2Loaded)
      {
        oled.print("2  ");
        statusStr += "2  ";
      }
      else
      {
        oled.print("+  ");
        statusStr += "+  ";
      }
      if(T3Loaded)
      {
        oled.println("3");
        statusStr += "3";
      }
      else
      {
        oled.println("+");
        statusStr += "+";
      }

      Serial.println(statusStr);
    }
    else
    {
      oled.println("     -  -  -  -");
      Serial.println("     -  -  -  -");
    }

    Serial.println("=======================");
  //  }
}
 
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
  EEPROM.write(0, currentExtruder); // Salvar estado
  EEPROM.write(1, lastExtruder);    // Salvar estado
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
 
    // keep waiting until button is pressed (NO: LOW = pressionado)
    while (digitalRead(trigger) != 0)
     {
       delay(50);
     }
 
     // Move while button is pressed (NO: LOW = pressionado)
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
 
   digitalWrite(selEnable, LOW); // lock the selector (LOW = enable para DVR8825)
   digitalWrite(selDir, direction); // Enables the motor to move in a particular direction
 
     // Makes pulses for rotation
     for (int x = 0; x < (moveDistance-1); x++)
     {
       digitalWrite(selStep, HIGH);
       delayMicroseconds(speedDelay);
       digitalWrite(selStep, LOW);
       delayMicroseconds(speedDelay);
     }
 
   digitalWrite(selEnable, HIGH); // unlock the selector (HIGH = disable para DVR8825)
 }
 
 // this cycles the servo between two positions (ADAPTADO PARA SERVO 360° CONTÍNUO)
 void cutFilament() {
   digitalWrite(selEnable, LOW); // enable stepper power for servo operation (LOW = enable para DVR8825)
 
   // Garantir que o servo esteja pronto antes do movimento
   filamentCutter.writeMicroseconds(1500); // Centro/parado
   delay(300); // Estabilização extra para primeira chamada
 
   // === ADAPTAÇÃO PARA SERVO 360° CONTÍNUO ===
   // Código original usava ângulos: 135° → 180° → 135° (45° em cada direção)
   // Servo 360° não tem ângulos - converte movimento angular em TEMPO de rotação
   //
   // AJUSTADO: Movimento EQUILIBRADO e simétrico
   // - Tempo: 0.3s por direção (movimento rápido)
   // - PWM simétrico: 1300µs e 1700µs (±200µs do centro 1500µs)
   // - Movimento igual nos dois sentidos para retorno preciso
 
   // Sentido A: abrir/cortar (equivalente ao openGillotine original)
   filamentCutter.writeMicroseconds(1300); // PWM: -200µs do centro (sentido horário)
   delay(300); // Tempo fixo: 0.3s
 
   // Parar momentaneamente
   filamentCutter.writeMicroseconds(1500); // PWM: centro/parado
   delay(100); // Pausa breve
 
   // Sentido B: fechar/retornar (equivalente ao closeGillotine original)
   filamentCutter.writeMicroseconds(1700); // PWM: +200µs do centro (sentido anti-horário)
   delay(300); // Mesmo tempo para movimento simétrico
 
   // Parar na posição original
   filamentCutter.writeMicroseconds(1500); // PWM: centro/parado
   delay(100); // Pausa final
 
   digitalWrite(selEnable, HIGH); // disable stepper (HIGH = disable para DVR8825)
   // NOTA: Não fazer detach aqui - deixar para disconnectGillotine()
 }
 
 // enable the servo
 void connectGillotine()
 {
   filamentCutter.attach(11);
   // Inicializar servo na posição parada para evitar movimento inesperado
   filamentCutter.writeMicroseconds(1500); // Centro/parado
   delay(500); // Aguardar estabilização (aumentado para primeira chamada)
 }
 
 // disable the servo - so it doesn't chatter when not in use
 void disconnectGillotine()
 {
   filamentCutter.detach();
 }
 
 // FUNÇÕES REMOVIDAS - SUBSTITUÍDAS POR LÓGICA DE TEMPO PARA SERVO 360°
 // openGillotine() e closeGillotine() não são mais necessárias
 // O movimento angular foi convertido em tempo de rotação
 
// rotate the selector clockwise too far from 4, so it'll grind on the bump stop
void homeSelector()
{
  // rotate counter clockwise to hard stop
  rotateSelector(clockwise, stepsPerRev * microSteps);

  // move just slightly to extruder 1 (this backs off a little from the hard stop)
  rotateSelector(counterclockwise, defaultBackoff * microSteps);

  // NÃO sobrescrever se já temos estado válido da EEPROM
  // Apenas definir valores padrão se necessário
  if(currentExtruder < 0 || currentExtruder > 3) {
    currentExtruder = 0;
  }
  if(lastExtruder < -2 || lastExtruder > 3) {
    lastExtruder = -2;
  }

  // Salvar estado na EEPROM
  EEPROM.write(0, currentExtruder);
  EEPROM.write(1, lastExtruder);

}
 
 // buzz buzz buzz
 void vibrateMotor()
 {
   // oscillate selector 1 time
   rotateSelector(clockwise, 2 * 16);
   rotateSelector(!clockwise, 2 * 16);
 }
 
 