/* Firmware 3DChameleon Mk4.1 - Versão Arduino Uno CNC Shield com Interface Serial

Adaptado para Arduino Uno com CNC Shield v3 para integração com Klipper
- Eixo Z: Motor da extrusora (alimentação de filamento)
- Eixo X: Motor do seletor (seleção de ferramenta)
- Serial: Comunicação com Klipper (9600 baud)
- OLED: Removido (usando saída Serial)
- Servo: Pino 11 (cortador de filamento/coletor de purga)

Pinos do Hardware:
- Controle de Motores: Eixo Z (D4/D7/D12), Eixo X (D2/D5/D8)
- Servo: D11 (coletor de purga)
- Sensores de Filamento (NC): 
  * T0: D9 (X Endstop)
  * T1: D10 (Y Endstop)
  * T2: D13 (SpnDir)
  * Hotend: A3 (CoolEnd)
- Sensor de Alimentação de Filamento (NO): T1: A2 (Resume)
- Sensores de Buffer (NC): A0 (HOLD), A1 (ABORT)

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

Comandos Serial (enviados do Klipper):

Comandos SELECT (apenas seleção de ferramenta):
T0 - Seleciona T0 (move seletor apenas para posição T0)
T1 - Seleciona T1 (move seletor apenas para posição T1)
T2 - Seleciona T2 (move seletor apenas para posição T2)
T3 - Seleciona T3 (move seletor apenas para posição T3)

Comandos LOAD/UNLOAD (trabalham na posição da ferramenta atual):
LOAD <distância> - Carrega filamento na posição da ferramenta atual com distância especificada (mm)
  * Usa sensor de filamento do hotend (A3) para detectar filamento, depois continua com distância especificada
UNLOAD - Descarrega filamento até sensor específico da ferramenta detectar ausência de filamento (parada automática)
  * T0: Usa sensor no D9 (X Endstop) - sensor NC (LOW = sem filamento)
  * T1: Usa sensor no D10 (Y Endstop) - sensor NC (LOW = sem filamento)
  * T2: Usa sensor no D13 (SpnDir) - sensor NC (LOW = sem filamento)
  * Para automaticamente quando sensor indica que filamento saiu
UNLOAD <distância> - Descarrega filamento da posição da ferramenta atual com distância especificada (mm) - modo legado
UNLOAD_RETRACAO <distância> - Descarrega filamento com velocidade fixa (5 mm/s) para sincronização de retração com Klipper

Carregamento Baseado em Sensor:
- loadUntilSensorAlimentacaoDeFilamento(toolNumber): Carrega filamento até sensor específico da ferramenta detectar presença
  * Move automaticamente seletor para ferramenta, carrega até sensor, depois retorna para posição original
  * T0: D9, T1: D10, T2: D13 (sensores NC - HIGH = filamento presente)

Comandos do Sistema:
HOME - Faz home apenas do seletor
IDLE - Move seletor para posição idle
COLETOR_ON - Abre coletor de purga (gira por 1 segundo)
COLETOR_OFF - Fecha coletor de purga (gira por 1 segundo)


Posições Idle 
T0 → idle em T2 
T1 → idle em T3
T2 → idle em T0
T3 → idle em T1
*/

#include <SPI.h>
#include <Servo.h>
#include <EEPROM.h>


// OLED removido - usando saída Serial ao invés

// CNC Shield não possui expansor SX1509 IO

// Define números dos pinos - Arduino Uno com CNC Shield
// Eixo Z para motor da extrusora (alimentação de filamento)
#define extEnable 12   // Z_ENABLE
#define extStep 4      // Z_STEP
#define extDir 7       // Z_DIR

// Eixo X para motor do seletor (seletor de ferramenta)
#define selEnable 8    // X_ENABLE
#define selStep 2      // X_STEP
#define selDir 5       // X_DIR

// Pino PURGA_COLETOR (compatível com Arduino Uno)
#define PURGA_COLETOR_PIN 11 // Coletor de purga -  Z+ Endstop da placa CNCShield

// Pino do sensor de filamento (NC - Normalmente Fechado)
#define FILAMENT_SENSOR_PIN A3 // CoolEnd da placa CNCShield

// Sensores de filamento específicos por ferramenta (NC - Normalmente Fechado)
// Usado para comando UNLOAD para detectar quando filamento saiu da ferramenta
// Lógica: HIGH = filamento presente, LOW = sem filamento
#define FILAMENT_SENSOR_T0_PIN 9   // Sensor de filamento T0 - X Endstop da placa CNCShield
#define FILAMENT_SENSOR_T1_PIN 10  // Sensor de filamento T1 - Y Endstop da placa CNCShield
#define FILAMENT_SENSOR_T2_PIN 13  // Sensor de filamento T2 - SpnDir da placa CNCShield

// Sensor de alimentação de filamento (detecta presença de filamento na ferramenta)
// Usado para monitorar se filamento está pré-carregado nas ferramentas
//#define FILAMENT_ALIMENTATION_SENSOR_T0_PIN A4 // Sensor de alimentação de filamento - CoolEnd da placa CNCShield
#define FILAMENT_ALIMENTATION_SENSOR_T1_PIN A2 // Sensor de alimentação T1 - NO (Normalmente Aberto) - Resume da placa CNCShield
// Lógica: HIGH (1) = filamento presente, LOW (0) = sem filamento
//#define FILAMENT_ALIMENTATION_SENSOR_T2_PIN A5 // Sensor de alimentação de filamento T2 - SpnDir da placa CNCShield
bool PRESENCA_FILAMENTO_T0 = false;
bool PRESENCA_FILAMENTO_T1 = false;
bool PRESENCA_FILAMENTO_T2 = false;

// Pinos do sistema de buffer (NC - Normalmente Fechado)
#define BUFFER_EMPTY_PIN A1      // ABORT - Microswitch 1 - Buffer vazio - HOLD da placa CNCShield
#define BUFFER_FULL_PIN A0       // HOLD - Microswitch 2 - Buffer cheio - ABORT da placa CNCShield

// Constantes do sistema de buffer
#define BUFFER_CHECK_INTERVAL 100   // Verificar buffer a cada 0.1s (resposta mais rápida)

bool imprimindo = false;

// Variaveis para controle do motor
const int counterclockwise = HIGH;
const int clockwise = !counterclockwise;

const int stepsPerRev = 200;
const int microSteps = 16;
const int speedDelay = 170;     // Original: 170, Faster: 85, Fastest: 40
const int selectorSpeedDelay = 60; // Velocidade específica para o seletor (3x mais rápido)

const int defaultBackoff = 5;

Servo filamentCutter;  // cria objeto servo para controlar um servo
int cutterPos = 0;    // variável para armazenar a posição do servo
bool reverseServo = true;

// Coletor de purga (360 graus servo)
const int COLETOR_STOP = 1500;           // Servo parado
const int COLETOR_OPEN_SPEED = 1300;     // Velocidade para abrir coletor (sentido horario)
const int COLETOR_CLOSE_SPEED = 1700;    // Velocidade para fechar coletor (sentido anti-horario)
const int COLETOR_OPEN_TIME = 300;       // Tempo para abrir coletor em ms (0.5 segundo)
const int COLETOR_CLOSE_TIME = 180;      // Tempo para fechar coletor em ms (0.5 segundo)

int currentExtruder = -1;
int nextExtruder = 0;
int lastExtruder = -1;
int tempExtruder = -1;

// Variáveis do sistema de buffer
unsigned long lastBufferCheck = 0;
bool lastBufferEmptyState = false;  // Estado anterior do sensor vazio
bool lastBufferFullState = false;   // Estado anterior do sensor cheio

// Timeout de idle baseado em inatividade do buffer (1 minuto = 60000ms)
#define IDLE_TIMEOUT 120000  // 1 minuto sem uso do buffer = impressora parada
unsigned long lastBufferActivity = 0;  // Última vez que o buffer foi usado
bool isInIdleMode = false;  // Flag para saber se está em modo idle


int loaderMode = 1;  //(0= direto, 1=loader/unloader - modo automático)

// Variáveis de comunicação serial
String serialBuffer = "";
bool commandReceived = false;

// Configurações de distância
long distance = 10;

// Cálculo de passos por mm (calibrado para sua configuração)
const float STEPS_PER_MM = 151.0; // Calibrado: comando 100mm = 51mm real -> precisa ~2x mais passos

long unloadDistance = stepsPerRev * microSteps * distance;  // são 10 voltas - cerca de 10"
long loadDistance   = unloadDistance * 1.1;           // são 11 voltas - cerca de 11"

int address = 0;
byte value;


// Endereços EEPROM
#define EEPROM_EXTRUDER_ADDR 0  // Endereço para armazenar extrusora atual

// Funções para salvar/carregar extrusora da EEPROM
void saveCurrentExtruder(int extruder) {
  EEPROM.write(EEPROM_EXTRUDER_ADDR, extruder + 1); // +1 porque EEPROM não pode armazenar -1
}

int loadSavedExtruder() {
  int saved = EEPROM.read(EEPROM_EXTRUDER_ADDR);
  return saved - 1; // -1 para restaurar valor original (-1 significa sem extrusora)
}

void setup()
{

  // Saída Serial ao invés de OLED
  Serial.begin(9600);

  Serial.println("3DChameleon Mk4 Ready");

  // Configura os pinos como Saídas
  pinMode(extEnable, OUTPUT);
  pinMode(extStep, OUTPUT);
  pinMode(extDir, OUTPUT);

  pinMode(selEnable, OUTPUT);
  pinMode(selStep, OUTPUT);
  pinMode(selDir, OUTPUT);

  // Configuração do sensor de filamento (NC - Normalmente Fechado, INPUT_PULLUP)
  pinMode(FILAMENT_SENSOR_PIN, INPUT_PULLUP);

  // Configuração dos sensores de filamento específicos por ferramenta (NC - Normalmente Fechado, INPUT_PULLUP)
  pinMode(FILAMENT_SENSOR_T0_PIN, INPUT_PULLUP);
  pinMode(FILAMENT_SENSOR_T1_PIN, INPUT_PULLUP);
  pinMode(FILAMENT_SENSOR_T2_PIN, INPUT_PULLUP);

  // Configuração do sensor de alimentação de filamento (NO - Normalmente Aberto, INPUT_PULLUP)
  //pinMode(FILAMENT_ALIMENTATION_SENSOR_T0_PIN, INPUT_PULLUP);
  pinMode(FILAMENT_ALIMENTATION_SENSOR_T1_PIN, INPUT_PULLUP);
  //pinMode(FILAMENT_ALIMENTATION_SENSOR_T2_PIN, INPUT_PULLUP);

  //Verifica quais filamentos estão pre alimentados
  //PRESENCA_FILAMENTO_T0 = (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T0_PIN) == HIGH);
  PRESENCA_FILAMENTO_T1 = (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T1_PIN) == HIGH); // HIGH = filamento presente, LOW = sem filamento
  //PRESENCA_FILAMENTO_T2 = (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T2_PIN) == HIGH);

  // Configuração dos sensores de buffer (NC - Normalmente Fechado, INPUT_PULLUP)
  pinMode(BUFFER_EMPTY_PIN, INPUT_PULLUP);  // A0 (HOLD)
  pinMode(BUFFER_FULL_PIN, INPUT_PULLUP);   // A1 (ABORT)

  // Inicializa estados dos sensores de buffer
  lastBufferEmptyState = (digitalRead(BUFFER_EMPTY_PIN) == HIGH);
  lastBufferFullState = (digitalRead(BUFFER_FULL_PIN) == HIGH);

  // Inicializa timer de atividade do buffer e modo idle
  lastBufferActivity = millis();
  isInIdleMode = false;

  // Inicializa servo para cortador de filamento/coletor de purga
  filamentCutter.attach(PURGA_COLETOR_PIN);
  filamentCutter.writeMicroseconds(COLETOR_STOP); // Inicia parado

  // Inicializa motor da extrusora como DESABILITADO (HIGH = desabilitado)
  digitalWrite(extEnable, HIGH); // Garante que motor da extrusora inicia desabilitado

  // Trava o seletor energizando-o - mantém travado 100% do tempo
  digitalWrite(selEnable, LOW); // Sempre travado

  // Auto-home na inicialização para estabelecer posição conhecida
  homeSelector();
  Serial.println("Home OK");

  // Carrega extrusora salva da EEPROM e posiciona nela
  lastExtruder = loadSavedExtruder();
  if(lastExtruder >= 0 && lastExtruder <= 2) {  // Apenas T0, T1, T2 disponíveis

    // Move para a posição da extrusora salva
    currentExtruder = lastExtruder;
    gotoExtruder(0, currentExtruder);  // Da posição home (0) para posição salva

    // Mantém o seletor na posição da ferramenta ativa
    Serial.print("T");
    Serial.print(lastExtruder);
    Serial.println(" loaded");
  } else {
    Serial.println("Stay T0");
    currentExtruder = 0;
    lastExtruder = 0;
    saveCurrentExtruder(currentExtruder);
  }
  
  Serial.print("T0: ");
  if(PRESENCA_FILAMENTO_T0) {
    Serial.println("Ocupado");
  } else {
    Serial.println("Livre");
  }
  Serial.print("\nT1: ");
  if(PRESENCA_FILAMENTO_T1) {
    Serial.println("Ocupado");
  } else {
    Serial.println("Livre");
  }
  Serial.print("\nT2: ");
  if(PRESENCA_FILAMENTO_T2) {
    Serial.println("Ocupado");
  } else {
    Serial.println("Livre");
  }

  Serial.println("\n\nCmd: T0-T2\nHOME\nIDLE\nLOAD <mm>\nUNLOAD (sensor) ou UNLOAD <mm>\nUNLOAD_RETRACAO <mm>\nCOLETOR_ON/OFF\nSTART_PRINT\nSTOP_PRINT\nTEST_SPEED\n\n");

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

    // RESETAR TIMER DE INATIVIDADE - houve atividade do usuário/Klipper
    lastBufferActivity = millis();
    isInIdleMode = false;  // SAIR DO MODO IDLE
    // Serial.println("Atividade detectada (comando serial) - saindo do idle");

    processSerialCommand(serialBuffer);
    serialBuffer = "";
    commandReceived = false;
  }

  // ========== MONITORAMENTO DE MUDANÇAS DO BUFFER ==========
  monitorBufferStateChanges();

  // ========== MONITORAMENTO AUTOMÁTICO DO BUFFER ==========
  unsigned long currentTime = millis();
  if (currentTime - lastBufferCheck >= BUFFER_CHECK_INTERVAL) {
    maintainBuffer();
    lastBufferCheck = currentTime;
  }

  // ========== MONITORAMENTO DE IDLE (sempre ativo) ==========
  //checkIdleTimeout(currentTime);

  //Check se precisa alimentar novo filamento adicionado antes do 3dchameleon
  monitorFilamentAlimentation();

  // small delay to prevent overwhelming the processor
  delay(10);
}



void monitorFilamentAlimentation() {
  if(imprimindo) {
    return;
  }else{
    // //Verifica se T0 está alimentado
    // if (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T0_PIN) == HIGH && PRESENCA_FILAMENTO_T0 == false) {
    //   Serial.println("Alimentando T0");
    //   loadUntilSensorAlimentacaoDeFilamento(0); // Alimenta T0 com novo filamento
    //   PRESENCA_FILAMENTO_T0 = true;
    // }
    // if (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T0_PIN) == LOW && PRESENCA_FILAMENTO_T0 == true) {
    //   Serial.println("T0 Livre");
    //   PRESENCA_FILAMENTO_T0 = false;
    // }
    
    //Verifica se T1 está alimentado
    // Sensor: HIGH (1) = filamento presente, LOW (0) = sem filamento
    if (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T1_PIN) == HIGH && PRESENCA_FILAMENTO_T1 == false) {
      Serial.println("Alimentando T1");
      loadUntilSensorAlimentacaoDeFilamento(1); // Alimenta T1 com novo filamento
      PRESENCA_FILAMENTO_T1 = true;
    }
    if (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T1_PIN) == LOW && PRESENCA_FILAMENTO_T1 == true) {
      Serial.println("T1 Livre");
      PRESENCA_FILAMENTO_T1 = false;
    }
    
    // //Verifica se T2 está alimentado
    // if (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T2_PIN) == HIGH && PRESENCA_FILAMENTO_T2 == false) {
    //   Serial.println("Alimentando T2");
    //   loadUntilSensorAlimentacaoDeFilamento(2); // Alimenta T2 com novo filamento
    //   PRESENCA_FILAMENTO_T2 = true;
    // }
    // if (digitalRead(FILAMENT_ALIMENTATION_SENSOR_T2_PIN) == LOW && PRESENCA_FILAMENTO_T2 == true) {
    //   Serial.println("T2 Livre");
    //   PRESENCA_FILAMENTO_T2 = false;
    // }
  }
}

// Funções do coletor de purga
void coletorOn() {
  filamentCutter.writeMicroseconds(COLETOR_OPEN_SPEED);

  // Gira por tempo definido para abrir
  delay(COLETOR_OPEN_TIME);

  // Para o servo
  filamentCutter.writeMicroseconds(COLETOR_STOP);
  Serial.println("Coletor ON");
}

void coletorOff() {
  filamentCutter.writeMicroseconds(COLETOR_CLOSE_SPEED);

  // Gira por tempo definido para fechar
  delay(COLETOR_CLOSE_TIME);

  // Para o servo
  filamentCutter.writeMicroseconds(COLETOR_STOP);
  Serial.println("Coletor OFF");
}

// ========== SISTEMA DE BUFFER DE FILAMENTO ==========

// Monitora mudanças de estado dos sensores do buffer
void monitorBufferStateChanges() {
  bool currentEmptyState = (digitalRead(BUFFER_EMPTY_PIN) == HIGH);
  bool currentFullState = (digitalRead(BUFFER_FULL_PIN) == HIGH);

  // Verifica mudança no sensor de buffer vazio
  if (currentEmptyState != lastBufferEmptyState) {
    Serial.print("Empty: ");
    Serial.print(lastBufferEmptyState ? "ON" : "OFF");
    Serial.print("->");
    Serial.println(currentEmptyState ? "ON" : "OFF");

    // Qualquer mudança no buffer = atividade, resetar timer de inatividade
    lastBufferActivity = millis();
    isInIdleMode = false;  // SAIR DO MODO IDLE

    lastBufferEmptyState = currentEmptyState;
  }

  // Verifica mudança no sensor de buffer cheio
  if (currentFullState != lastBufferFullState) {
    Serial.print("Full: ");
    Serial.print(lastBufferFullState ? "ON" : "OFF");
    Serial.print("->");
    Serial.println(currentFullState ? "ON" : "OFF");

    // Mudança no sensor cheio também é atividade
    lastBufferActivity = millis();
    isInIdleMode = false;  // SAIR DO MODO IDLE

    lastBufferFullState = currentFullState;
  }

  // Verifica mudança no sensor de buffer cheio
  if (currentFullState != lastBufferFullState) {
    Serial.print("Full: ");
    Serial.print(lastBufferFullState ? "ON" : "OFF");
    Serial.print("->");
    Serial.println(currentFullState ? "ON" : "OFF");
    lastBufferFullState = currentFullState;
  }
}

// Mantém buffer carregado quando necessário (automático)
void maintainBuffer() {
  bool filamentInHotend = (digitalRead(FILAMENT_SENSOR_PIN) == HIGH);
  if (!filamentInHotend) return;  // Sem filamento no hotend, não faz nada

  bool bufferEmpty = (digitalRead(BUFFER_EMPTY_PIN) == HIGH);
  unsigned long currentTime = millis();

  // Se buffer está vazio, alimentar imediatamente (atividade normal de impressão)
  if (bufferEmpty) {
    Serial.println("Loading Buffer");

    // Resetar timer de inatividade (houve atividade)
    lastBufferActivity = currentTime;

    // VERIFICAÇÃO CRÍTICA: Seletor deve estar na posição da ferramenta ativa
    if (currentExtruder != lastExtruder) {
      Serial.print("Movendo para T");
      Serial.print(lastExtruder);
      gotoExtruder(currentExtruder >= 0 ? currentExtruder : 0, lastExtruder);
      currentExtruder = lastExtruder;
    }

    // Alimenta o buffer se o seletor estiver na posição correta
    if (currentExtruder == lastExtruder) {
      feedBuffer();
      Serial.println("Buffer recarregado");
    } else {
      Serial.println("ERRO: Seletor error!");
    }
  }

  // Verificar idle timeout baseado em inatividade
  //checkIdleTimeout(currentTime);
}

// Verifica se deve ir para idle baseado em inatividade geral
// void checkIdleTimeout(unsigned long currentTime) {
//   return;
//   // Se já está em modo idle, não verificar timeout
//   if (isInIdleMode) {
//     return;
//   }

//   // Verificar se passou o timeout de inatividade DESDE O STARTUP
//   if ((currentTime - lastBufferActivity) >= IDLE_TIMEOUT) {
//     Serial.println("Inatividade geral detectada - indo para idle...");

//     // Verificar se já não está em idle
//     int idlePos;
//     if (lastExtruder == 0) idlePos = 2;
//     else if (lastExtruder == 1) idlePos = 3;
//     else if (lastExtruder == 2) idlePos = 0;
//     else idlePos = 2; // Default

//     // ANTES DE IR PARA IDLE: Verificar e preparar buffer se possível
//     bool filamentInHotend = (digitalRead(FILAMENT_SENSOR_PIN) == HIGH);
//     bool bufferFull = (digitalRead(BUFFER_FULL_PIN) == HIGH);

//     if (filamentInHotend && !bufferFull) {
//       Serial.println("Buffer não cheio - preparando antes de ir para idle...");

//       // Seletor deve estar na posição da ferramenta ativa para alimentar buffer
//       if (currentExtruder != lastExtruder) {
//         Serial.print("Seletor fora de posição! Movendo T");
//         Serial.print(lastExtruder);
//         Serial.println("...");
//         gotoExtruder(currentExtruder >= 0 ? currentExtruder : 0, lastExtruder);
//         currentExtruder = lastExtruder;
//         Serial.println("Seletor reposicionado");
//       }

//       // Só alimentar se estiver na posição correta
//       if (currentExtruder == lastExtruder) {
//         Serial.println("Alimentando buffer...");
//         feedBuffer();
//         Serial.println("Buffer preparado");
//       }
//     } else if (filamentInHotend && bufferFull) {
//       Serial.println("Buffer já está cheio - ok para idle");
//     } else if (!filamentInHotend) {
//       Serial.println("Sem filamento no hotend - indo para idle sem preparar buffer");
//     }

//     // IR PARA IDLE
//     if (currentExtruder != idlePos) {
//       Serial.println("Movendo seletor para idle...");
//       moveToIdle();
//     }
//     Serial.println("Seletor em idle (motor não aquece)");

//     // ATIVAR MODO IDLE - para não ficar repetindo
//     isInIdleMode = true;
//   }
// }

// Alimenta filamento para o buffer até sensor full
void feedBuffer() {
  // Direção baseada na ferramenta atual (mesma lógica do load)
  bool direction = (lastExtruder < 2) ? clockwise : counterclockwise;

  // Habilita motor do extruder
  digitalWrite(extEnable, LOW);
  digitalWrite(extDir, direction);

  long stepsFed = 0;
  long maxSteps = (long)(300.0 * STEPS_PER_MM); // Máximo 300mm de segurança
  bool bufferFull = false;

  // Velocidade lenta para controle preciso
  const int feedSpeed = speedDelay;

  while (stepsFed < maxSteps && !bufferFull) {
    // Verifica se buffer está cheio (segundo microswitch)
    if (digitalRead(BUFFER_FULL_PIN) == HIGH) {
      bufferFull = true;
      Serial.println("Buffer Full");
      break;
    }

    // Move um passo
    digitalWrite(extStep, HIGH);
    delayMicroseconds(feedSpeed);
    digitalWrite(extStep, LOW);
    delayMicroseconds(feedSpeed);

    stepsFed++;
  }

  // Desabilita motor
  digitalWrite(extEnable, HIGH);

  if (!bufferFull) {
    Serial.println("ERRO: Buffer nao cheio!");
  } else {
    float distanceFed = (float)stepsFed / STEPS_PER_MM;
    Serial.print("Buffer: ");
    Serial.print(distanceFed);
    Serial.println("mm");
  }
}

// Processa comandos serial do Klipper
void processSerialCommand(String command)
{
  command.trim(); // Remove espaços em branco
  command.toUpperCase(); // Converte para maiúsculas para comparação sem diferenciação de maiúsculas/minúsculas

  bool isLoadUnloadCommand = false;
  int toolNumber = -1;
  bool isLoad = false;
  float distance_mm = 0.0;

  // Verifica comando UNLOAD_RETRACAO (descarrega com velocidade fixa)
  if (command.startsWith("UNLOAD_RETRACAO ")) {
    // Extrai parâmetro de distância
    int spaceIndex = command.indexOf(' ');
    String distanceStr = command.substring(spaceIndex + 1);
    distance_mm = distanceStr.toFloat();

    if (distance_mm <= 0) {
      Serial.println("Distancia invalida");
      return;
    }

    // Verifica se uma ferramenta está selecionada
    if (currentExtruder < 0) {
      Serial.println("Selecione tool primeiro");
      return;
    }

    // Executa UNLOAD_RETRACAO com velocidade fixa
    executeUnloadRetracao(currentExtruder, distance_mm);
    return;
  }

  // Verifica comando UNLOAD sem distância (usa sensor)
  if (command == "UNLOAD") {
    // Verifica se uma ferramenta está selecionada
    if (currentExtruder < 0) {
      Serial.println("Selecione tool primeiro");
      return;
    }

    // Executa UNLOAD usando sensor (sem necessidade de distância)
    unloadUntilSensorTrocaDeFilamento(currentExtruder);
    return;
  }

  // Verifica comandos LOAD/UNLOAD com parâmetro de distância
  if (command.startsWith("LOAD ") || command.startsWith("UNLOAD ")) {
    isLoadUnloadCommand = true;

    // Determina se é LOAD ou UNLOAD
    if (command.startsWith("LOAD ")) {
      isLoad = true;
    } else if (command.startsWith("UNLOAD ")) {
      isLoad = false;
    }

    // Extrai parâmetro de distância
    int spaceIndex = command.indexOf(' ');
    String distanceStr = command.substring(spaceIndex + 1);
    distance_mm = distanceStr.toFloat();

    if (distance_mm <= 0) {
      Serial.println("Distancia invalida");
      return;
    }

    // Verifica se uma ferramenta está selecionada
    if (currentExtruder < 0) {
      Serial.println("Selecione tool primeiro");
      return;
    }

    // Executa comando LOAD/UNLOAD na ferramenta atual
    executeLoadUnload(currentExtruder, isLoad, distance_mm);
    return;
  }

  // Comandos de seleção de ferramenta (apenas move seletor)
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
    // Nota: homeSelector() já atualiza currentExtruder para 0
    return;
  }
  else if (command == "IDLE") {
    moveToIdle();
    return;
  }
  else if (command == "COLETOR_ON" || command == "coletor_on") {
    coletorOn();
    return;
  }
  else if (command == "COLETOR_OFF" || command == "coletor_off") {
    coletorOff();
    return;
  }
  else if (command == "START_PRINT") {
    imprimindo = true;
    return;
  }
  else if (command == "STOP_PRINT") {
    imprimindo = false;
    return;
  }
  else {
    Serial.println("Cmd invalido");
    return;
  }
}

// Seleção de ferramenta - apenas move seletor para posição
void selectTool(int toolNumber)
{
  Serial.print("Sel T");
  Serial.println(toolNumber);

  // Move seletor para a posição da ferramenta
  // Da posição atual para posição alvo
  int fromPos = (currentExtruder >= 0) ? currentExtruder : 0;
  gotoExtruder(fromPos, toolNumber);

  currentExtruder = toolNumber;
  lastExtruder = toolNumber;

  // Salva na EEPROM
  saveCurrentExtruder(currentExtruder);

  Serial.print("T");
  Serial.print(toolNumber);
  Serial.println(" OK");
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

    // Atualiza currentExtruder para posição idle para rastreamento
    // NOTA: Mantemos lastExtruder como ferramenta ativa, currentExtruder rastreia posição física
    currentExtruder = idlePos;
    // lastExtruder permanece como ferramenta ativa (não atualizado para posição idle)

    // Salva posição atual na EEPROM (posição física para rastreamento)
    saveCurrentExtruder(currentExtruder);

    Serial.println("Idle OK");
  }

  
}

// Descarrega filamento até sensor detectar ausência de filamento (sensor NC vai para LOW quando filamento saiu)
void unloadUntilSensorTrocaDeFilamento(int toolNumber)
{
  Serial.print("Unloading T");
  Serial.print(toolNumber);

  // Seleciona o pino do sensor correto baseado no número da ferramenta
  int sensorPin;
  if (toolNumber == 0) {
    sensorPin = FILAMENT_SENSOR_T0_PIN;
  } else if (toolNumber == 1) {
    sensorPin = FILAMENT_SENSOR_T1_PIN;
  } else if (toolNumber == 2) {
    sensorPin = FILAMENT_SENSOR_T2_PIN;
  } else {
    Serial.println("ERRO: Tool invalido para sensor!");
    return;
  }

  // Determina direção de descarga baseada no número da ferramenta (oposto do carregamento)
  bool direction = (toolNumber < 2) ? counterclockwise : clockwise;

  // Garante que motor está desabilitado antes de habilitar (verificação de segurança)
  digitalWrite(extEnable, HIGH);
  digitalWrite(extEnable, LOW);  // trava o motor
  digitalWrite(extDir, direction); // Define direção

  long stepsUnloaded = 0;
  long maxSteps = (long)(2000.0 * STEPS_PER_MM); // Máximo 2000mm para prevenir loop infinito
  bool filamentGone = false;

  // Move lentamente para detectar quando filamento sai (sensor NC vai para LOW quando filamento saiu)
  const int sensorSpeed = speedDelay / 5; // Velocidade mais lenta para detecção do sensor

  while (stepsUnloaded < maxSteps && !filamentGone) {
    // Verifica estado do sensor (NC = LOW quando filamento saiu, HIGH quando filamento presente)
    if (digitalRead(sensorPin) == HIGH) { // Sensor indica sem filamento
      filamentGone = true;
      break;
    }

    // Move um passo
    digitalWrite(extStep, HIGH);
    delayMicroseconds(sensorSpeed);
    digitalWrite(extStep, LOW);
    delayMicroseconds(sensorSpeed);

    stepsUnloaded++;
  }

  // Desabilita motor
  digitalWrite(extEnable, HIGH);

  if (!filamentGone) {
    Serial.println("ERRO: Sensor nao detectou saida do filamento apos movimento maximo!");
    Serial.print("Movido: ");
    Serial.print((float)stepsUnloaded / STEPS_PER_MM);
    Serial.println("mm");
  } else {
    Serial.print("Unload OK: ");
    Serial.print((float)stepsUnloaded / STEPS_PER_MM);
    Serial.println("mm");
  }
}

// Load filament until sensor is triggered, then continue with specified distance
long loadUntilSensorTrocaDeFilamento(bool direction, float additionalDistance_mm)
{
  Serial.println("Loading to sensor...");

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
      Serial.println("Sensor OK!");
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

  Serial.print("To sensor: ");
  Serial.print((float)stepsToSensor / STEPS_PER_MM);
  Serial.println("mm");

  // Now continue with the specified additional distance
  if (additionalDistance_mm > 0) {
    long additionalSteps = (long)(additionalDistance_mm * STEPS_PER_MM);
    Serial.print("Extra: ");
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
  Serial.print("Total: ");
  Serial.print((float)totalSteps / STEPS_PER_MM);
  Serial.println("mm");

  return totalSteps;
}

// Execute UNLOAD_RETRACAO with fixed speed (for synchronization with Klipper retraction)
void executeUnloadRetracao(int toolNumber, float distance_mm)
{
  Serial.print("UNLOAD_RETRACAO_T");
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

  // Determine direction based on tool number (unload direction: opposite of load)
  bool direction = (toolNumber < 2) ? counterclockwise : clockwise;

  // Calculate steps from distance
  long steps = (long)(distance_mm * STEPS_PER_MM);

  // Fixed speed for retraction (5 mm/s)
  // Convert to delay: 5 mm/s * 151 steps/mm = 755 steps/s
  // Each step needs 2 pulses (HIGH + LOW) = 1510 pulses/s
  // Delay per pulse = 1000000 / 1510 = ~662 µs per pulse
  const int fixedRetractionSpeed = 662; // Fixed delay in microseconds (662µs = 1510 pulses/s = 755 steps/s = 5 mm/s)

  Serial.println("Executando UNLOAD_RETRACAO com velocidade fixa...");

  // Ensure motor is disabled before enabling (safety check)
  digitalWrite(extEnable, HIGH);
  digitalWrite(extEnable, LOW);  // lock the motor
  digitalWrite(extDir, direction); // Set direction

  // Move with fixed speed (no ramping)
  for (long x = 0; x < steps; x++)
  {
    digitalWrite(extStep, HIGH);
    delayMicroseconds(fixedRetractionSpeed);
    digitalWrite(extStep, LOW);
    delayMicroseconds(fixedRetractionSpeed);
  }

  // Disable motor
  digitalWrite(extEnable, HIGH);

  Serial.println("UNLOAD_RETRACAO concluida");
}

// Execute LOAD/UNLOAD commands with specific distance
void executeLoadUnload(int toolNumber, bool isLoad, float distance_mm)
{
  Serial.print("LOAD_T");
  Serial.print(toolNumber);
  Serial.print(" ");
  Serial.print(distance_mm);
  Serial.println("mm");

  // If this is different from current tool, move to the tool first
  if (currentExtruder != toolNumber) {
    gotoExtruder(currentExtruder >= 0 ? currentExtruder : 0, toolNumber);
    currentExtruder = toolNumber;
  }

  // Determine direction based on tool number and operation
  bool direction;
  // Load direction: clockwise for T0,T1, counterclockwise for T2,T3
  direction = (toolNumber < 2) ? clockwise : counterclockwise;

  // Use sensor-based loading for LOAD operations
  loadUntilSensorTrocaDeFilamento(direction, distance_mm);

  // Update last extruder
  lastExtruder = toolNumber;

  // Save to EEPROM
  saveCurrentExtruder(currentExtruder);

  Serial.println("LOAD OK");
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
  Serial.print("Movendo seletor T");
  Serial.print(currentCog);
  Serial.print("->T");
  Serial.println(targetCog);

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

  Serial.println("Seletor movido");
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
  digitalWrite(selEnable, LOW); // lock the selector
  digitalWrite(selDir, direction); // Enables the motor to move in a particular direction

    // Makes 50 pulses for making one full cycle rotation
    // Usa velocidade específica do seletor (mais rápida)
    for (int x = 0; x < (moveDistance-1); x++)
    {
      digitalWrite(selStep, HIGH);
      delayMicroseconds(speedDelay);  // Velocidade otimizada para seletor
      digitalWrite(selStep, LOW);
      delayMicroseconds(speedDelay);
    }
}

// rotate the selector clockwise too far from 4, so it'll grind on the bump stop
void homeSelector()
{
  Serial.println("Homing seletor...");
  // rotate counter clockwise to hard stop
  rotateSelector(clockwise, stepsPerRev * microSteps);

  // move just slightly to extruder 1 (this backs off a little from the hard stop)
  rotateSelector(counterclockwise, defaultBackoff * microSteps);

  // After homing, we're at a known position (backoff from hard stop = position 0 = T0)
  currentExtruder = 0;  // We're at T0 position physically
  // NOTE: Não alterar lastExtruder aqui - homing não deve sobrescrever a memória da última ferramenta usada

  // NÃO salvar T0 na EEPROM durante homing - isso sobrescreveria a última ferramenta selecionada!
  // saveCurrentExtruder(currentExtruder); // ← REMOVIDO - era o bug!

  Serial.println("Home OK");
}


// Carrega filamento até sensor ser acionado, depois continua com distância especificada
void loadUntilSensorAlimentacaoDeFilamento(int toolNumber)
{
  Serial.print("Loading T");
  Serial.print(toolNumber);
  Serial.println(" until sensor detects filament...");

  // Salva a posição original da ferramenta para retornar depois
  int originalTool = currentExtruder >= 0 ? currentExtruder : 0;

  // Se esta é diferente da ferramenta atual, move para a ferramenta primeiro
  if (currentExtruder != toolNumber) {
    Serial.println("Movendo para o tool correto...");
    gotoExtruder(currentExtruder >= 0 ? currentExtruder : 0, toolNumber);
    currentExtruder = toolNumber;
  }

  // Seleciona o pino do sensor correto baseado no número da ferramenta
  int sensorPin;
  if (toolNumber == 0) {
    sensorPin = FILAMENT_SENSOR_T0_PIN;
  } else if (toolNumber == 1) {
    sensorPin = FILAMENT_SENSOR_T1_PIN;
  } else if (toolNumber == 2) {
    sensorPin = FILAMENT_SENSOR_T2_PIN;
  } else {
    Serial.println("ERRO: Tool invalido para sensor!");
    return;
  }

  // Determina direção de carregamento baseada no número da ferramenta
  // T0, T1: horário, T2, T3: anti-horário
  bool direction = (toolNumber < 2) ? clockwise : counterclockwise;

  // Garante que motor está desabilitado antes de habilitar (verificação de segurança)
  digitalWrite(extEnable, HIGH);
  digitalWrite(extEnable, LOW);  // trava o motor
  digitalWrite(extDir, direction); // Define direção

  long stepsToSensor = 0;
  long maxSteps = (long)(2000.0 * STEPS_PER_MM); // Máximo 2000mm para prevenir loop infinito
  bool sensorTriggered = false;

  // Move lentamente até sensor ser acionado (sensor NC vai para HIGH quando filamento está presente)
  const int sensorSpeed = speedDelay / 5; // Velocidade mais lenta para detecção do sensor

  while (stepsToSensor < maxSteps && !sensorTriggered) {
    // Verifica estado do sensor (NC = HIGH quando filamento está presente)
    if (digitalRead(sensorPin) == HIGH) { // Sensor acionado (filamento detectado)
      sensorTriggered = true;
      Serial.println("Sensor OK! Filamento detectado!");
      break;
    }

    // Move um passo
    digitalWrite(extStep, HIGH);
    delayMicroseconds(sensorSpeed);
    digitalWrite(extStep, LOW);
    delayMicroseconds(sensorSpeed);

    stepsToSensor++;
  }

  // Desabilita motor
  digitalWrite(extEnable, HIGH);

  if (!sensorTriggered) {
    Serial.println("ERRO: Sensor nao detectado apos movimento maximo!");
    Serial.print("Movido: ");
    Serial.print((float)stepsToSensor / STEPS_PER_MM);
    Serial.println("mm");
  } else {
    Serial.print("Load completo: ");
    Serial.print((float)stepsToSensor / STEPS_PER_MM);
    Serial.println("mm");
  }

  // Retorna para posição original da ferramenta
  if (currentExtruder != originalTool) {
    Serial.println("Retornando para a ferramenta original...");
    gotoExtruder(currentExtruder, originalTool);
    currentExtruder = originalTool;
  }
}