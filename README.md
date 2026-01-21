# 3DChameleon K1C - Sistema de Troca Automática de Filamentos

[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino-Uno-blue.svg)](https://www.arduino.cc/)
[![Klipper](https://img.shields.io/badge/Klipper-Integration-orange.svg)](https://www.klipper3d.org/)

## 📋 Visão Geral

Este projeto é uma implementação adaptada do **3DChameleon Mk4** para a impressora **Creality K1/K1C**, utilizando comunicação serial integrada com o **Klipper**. O sistema permite troca automática de até 4 filamentos diferentes durante a impressão, com controle completo via interface Klipper.

### 🎯 Características Principais

- **Troca Automática**: Até 4 filamentos diferentes
- **Integração Klipper**: Controle completo via G-code
- **Sistema de Buffer**: Alimentação automática de filamento (já implementado)
- **Coletor de Purga**: Sistema inteligente de coleta de purga
- **Estado Persistente**: Salva estado entre reinicializações
- **Corte Automático**: Sequência segura de corte de filamento

### 🔧 Integrações Já Implementadas

Este projeto utiliza e adapta os seguintes sistemas já prontos:

#### 📦 **Sistema de Buffer Automático** (`FILAMENT_BUFFER_MOUNT.stl`)
- **Montagem**: Modelo STL customizado para Creality K1
- **Funcionalidade**: Alimentação automática quando buffer vazio
- **Sensores**: Microswitches NC para detecção de nível
- **Integração**: Completamente integrada ao firmware Arduino

#### 🐢 **Sistema de Buffer TurtleNeck** (`TurtleNeck-main/`)
- **Uso**: Sistema de buffer de filamento (TurtleNeck buffer)
- **Modelos**: `frame.stl`, `lid.stl`, `[a]_slide.stl`, `TN_horizontal mount.stl`
- **Integração**: Buffer de filamento compatível com montagem horizontal na Creality K1
- **Função**: Mantém reservatório de filamento para reduzir tensão na extrusora

#### 🔀 **Hub PTFE 3-para-1** (`ptfe-3-to-1-hub-model_files/`)
- **Distribuição**: `ptfe3to1hub_v2.stl` para 3 filamentos
- **Versões**: Full (`ptfe3x1_full.stl`) e Half (`ptfe3x1_half.stl`)
- **Integração**: Sistema de distribuição para múltiplos filamentos

---

## 🏗️ Arquitetura do Sistema

### Componentes Principais

#### 1. **Arduino Uno + CNC Shield v3** (`3dchameleon_serial.ino`)
- **Eixo Z**: Motor da extrusora (alimentação de filamento)
- **Eixo X**: Motor do seletor (seleção de ferramenta)
- **Servo Pin 11**: Coletor de purga
- **Sensores**: Filamento e buffer (microswitches NC)
- **Comunicação**: Serial 9600 baud com Klipper

#### 2. **Integração Klipper** (`3dchameleon_serial.cfg`)
- Macros completas de troca automática
- Sistema de estado persistente
- Controle de temperatura automático
- Sequências de corte e purga

#### 3. **Script Python** (`control_arduino.py`)
- Gerenciamento de estado persistente
- Comunicação serial com Arduino
- Armazenamento JSON independente

### Fluxo de Operação

```
Seleção de Filamento (T0-T3)
        ↓
Corte de Filamento Atual
        ↓
Remoção (Unload) + Purga
        ↓
Seleção de Novo Filamento
        ↓
Carregamento (Load) + Purga
        ↓
Retorno à Posição Anterior
```

---

## 🔧 Hardware Necessário

### Eletrônica
- **Arduino Uno R3**
- **CNC Shield v3** para Arduino Uno
- **2x Drivers A4988 ou DRV8825** (para motores stepper)
- **Servo 360°** (para coletor de purga)
- **2x Microswitches NC** (sensores de buffer)
- **1x Microswitch NC** (sensor de filamento, que fica na extrusora da impressora)

### Motores
- **Motor Stepper 1**: Seletor de ferramentas (NEMA 17)
- **Motor Stepper 2**: Extrusora/buffer (NEMA 17)

### Sensores
- **Sensor de Filamento**: Microswitch NC no hotend
- **Buffer Empty**: Microswitch NC (posição A1)
- **Buffer Full**: Microswitch NC (posição A0)

---

## 📁 Estrutura de Arquivos

```
3DCHAMELEON-K1C/
├── 📄 3dchameleon_serial.ino          # Firmware Arduino principal
├── 📄 3dchameleon_serial.cfg          # Configuração Klipper
├── 📄 control_arduino.py              # Script de comunicação
├── 📁 STLs/                           # Modelos 3D
│   ├── 📁 3DChameleonMk4/            # Modelos oficiais do projeto
│   │   ├── 3DChameleon 4.0 (A Part).stl
│   │   ├── 3DChameleon 4.0 (B Part).stl
│   │   ├── 3DChameleon 4.0 (Bearing Carrier).stl
│   │   ├── 3DChameleon 4.0 (Cam).stl
│   │   ├── 3DChameleon 4.0 (Electronics Enclosure).stl
│   │   ├── 3DChameleon 4.0 (PTFE Template).stl
│   │   ├── 3DChameleon 4.0 (Spacer).stl
│   │   ├── 3DChameleon 4.0 (Switch Mount).stl
│   │   ├── Creality K1 Switch Mounting Plate.stl
│   │   └── 3DChameleon4YAdapter.stl
│   ├── 📁 creality-k1-hopper-model_files/
│   │   ├── holder.stl                # Suporte do hopper Creality K1
│   │   └── hopper.stl                # Hopper Creality K1
│   ├── 📁 ptfe-3-to-1-hub-model_files/
│   │   ├── ptfe3to1hub_v2.stl        # Hub PTFE 3-para-1
│   │   ├── ptfe3x1_full.stl          # Versão completa
│   │   └── ptfe3x1_half.stl          # Versão meia
│   ├── 📁 TurtleNeck-main/           # Sistema de Buffer TurtleNeck
│   │   ├── STLs/
│   │   │   ├── [a]_slide.stl
│   │   │   ├── frame.stl
│   │   │   ├── lid.stl
│   │   └── TN_horizontal mount.stl
│   └── 📄 FILAMENT_BUFFER_MOUNT.stl   # Montagem do buffer (JÁ IMPLEMENTADO)
└── 📄 README.md                       # Esta documentação
```

---

## ✅ Sistema Já Implementado

### 🎯 Componentes Funcionais

#### 🔄 **Sistema de Buffer Automático**
- **Status**: ✅ COMPLETAMENTE IMPLEMENTADO
- **Hardware**: Arduino + sensores + motor stepper
- **Software**: Firmware integrado com detecção automática
- **STLs**: `FILAMENT_BUFFER_MOUNT.stl` + **TurtleNeck buffer** para K1

#### 🐢 **Sistema TurtleNeck (Buffer)**
- **Status**: ✅ MODELOS DISPONÍVEIS
- **Uso**: Buffer de filamento TurtleNeck para reduzir tensão
- **Montagem**: Horizontal compatível com Creality K1
- **STLs**: `frame.stl`, `lid.stl`, `slide.stl`, `mount.stl`

#### 🔀 **Hub PTFE 3-para-1**
- **Status**: ✅ SISTEMA PRONTO
- **Função**: Distribuição de 3 filamentos para o seletor
- **Versões**: Completa e meia disponíveis
- **Integração**: Compatível com sistema de troca automática

#### 🤖 **Firmware Arduino**
- **Status**: ✅ FUNCIONAL
- **Comunicação**: Serial 9600 baud com Klipper
- **Motores**: Controle X (seletor) e Z (extrusora)
- **Sensores**: Filamento + buffer com lógica NC

#### 🖥️ **Integração Klipper**
- **Status**: ✅ COMPLETA
- **Macros**: T0-T2 com troca automática
- **Estado**: Persistente entre reinicializações
- **Shell Command**: Python bridge funcionando

---

## ⚙️ Instalação e Configuração

### 1. Preparação do Hardware

#### Conexões Arduino/CNC Shield:

```cpp
// Eixo Z - Extrusora (alimentação filamento)
#define extEnable 12   // Z_ENABLE
#define extStep 4      // Z_STEP
#define extDir 7       // Z_DIR

// Eixo X - Seletor (seleção ferramenta)
#define selEnable 8    // X_ENABLE
#define selStep 2      // X_STEP
#define selDir 5       // X_DIR

// Servo e Sensores
#define SERVO_PIN 11
#define FILAMENT_SENSOR_PIN A3
#define BUFFER_EMPTY_PIN A1
#define BUFFER_FULL_PIN A0
```

#### Impressão dos Componentes:

1. **Estrutura Principal**: Imprimir peças A e B do 3DChameleon
2. **Suporte Creality K1**: Usar `holder.stl` e `hopper.stl`
3. **Sistema Buffer**: Usar `FILAMENT_BUFFER_MOUNT.stl`
4. **Hub PTFE**: Usar `ptfe3to1hub_v2.stl` para distribuição de filamentos

### 2. Instalação do Firmware

1. **Carregar `3dchameleon_serial.ino`** no Arduino Uno
2. **Verificar comunicação serial** (9600 baud)
3. **Testar homing** do seletor

### 3. Configuração Klipper

1. **Copiar `3dchameleon_serial.cfg`** para pasta config do Klipper
2. **Instalar shell_command module** (se necessário)
3. **Copiar `control_arduino.py`** para `/usr/data/printer_data/config/`
4. **Dar permissões de execução**: `chmod +x control_arduino.py`
5. **Reinicializar Klipper**: `FIRMWARE_RESTART`

### 4. Calibração

#### Calibração de Passos por mm:
```cpp
const float STEPS_PER_MM = 151.0; // Ajustar conforme necessário
```

#### Calibração de Posições:
- Executar `CFS_HOME` para homing
- Testar seleção `T0`, `T1`, `T2` sequencialmente
- Verificar posições idle corretas

---

## 🎮 Como Usar

### Comandos Básicos

#### Troca de Filamentos:
```gcode
T0    ; Seleciona filamento 0 (completo com corte/purga)
T1    ; Seleciona filamento 1
T2    ; Seleciona filamento 2
```

#### Comandos de Controle:
```gcode
CFS_HOME          ; Home do seletor
CFS_IDLE          ; Move para posição idle
CFS_STATUS        ; Status do Arduino
```

#### Sistema de Purga:
```gcode
CFS_CUT           ; Posiciona para corte físico
CFS_PURGE_START   ; Ativa coletor + posiciona
CFS_PURGE_FINISH  ; Purga filamento + desativa
```

### Uso no Slicer

#### PrusaSlicer - Tool Change G-code:
```gcode
; Troca para próximo filamento
CFS_T{next_extruder}
M117 "Filamento {next_extruder} carregado"
```

### Monitoramento de Estado

```gcode
CURRENT_FILAMENT          ; Mostra filamento atual
SET_CURRENT_FILAMENT T=1  ; Define filamento atual
LOAD_CURRENT_FILAMENT      ; Carrega estado salvo
```

---

## 🔧 Sistema de Buffer Automático

### 🐢 **O que é TurtleNeck?**

O **TurtleNeck** é um sistema de buffer de filamento inteligente que:

- **Mantém um reservatório** de filamento entre o spool e a extrusora
- **Reduz a tensão** no filamento durante movimentos rápidos da cabeça
- **Previne emaranhamento** e quebra do filamento
- **Melhora a qualidade** da impressão com filamentos flexíveis
- **Compatível** com montagem horizontal na Creality K1

### 📦 **Componentes do Sistema de Buffer**

1. **FILAMENT_BUFFER_MOUNT.stl**: Montagem específica para Creality K1
2. **TurtleNeck Buffer**: Sistema de reservatório inteligente
3. **Sensores**: Microswitches NC para detecção automática de nível
4. **Firmware Arduino**: Controle automático de alimentação

### 🔄 **Funcionamento Automático**

- **Detecção**: Sensor identifica quando buffer está vazio
- **Alimentação**: Motor stepper alimenta filamento automaticamente
- **Monitoramento**: Verificação contínua durante impressão
- **Proteção**: Timeout de inatividade para economia de energia

### Funcionalidades

- **Alimentação Automática**: Buffer se recarrega quando vazio
- **Monitoramento Contínuo**: Verificação a cada 200ms
- **Estado Persistente**: Sobrevive a reinicializações
- **Proteções**: Timeout de inatividade (2 minutos)

### Lógica de Operação

1. **Detecção**: Sensor identifica buffer vazio
2. **Verificação**: Confirma filamento no hotend
3. **Posicionamento**: Move seletor para ferramenta ativa
4. **Alimentação**: Extrusora alimenta até sensor "full"
5. **Confirmação**: Log de operação bem-sucedida

### Configurações

```cpp
#define BUFFER_CHECK_INTERVAL 200   // Verificação a cada 200ms
#define IDLE_TIMEOUT 120000         // 2 minutos sem atividade
const float STEPS_PER_MM = 151.0;   // Calibração passos/mm
```

---

## 🛠️ Sistema de Purga Inteligente

### Sequência de Purga

1. **Corte**: Posicionamento seguro para corte físico
2. **Ativação**: Servo abre coletor de purga
3. **Posicionamento**: Bico vai para área de purga
4. **Extrusão**: Purga filamento residual
5. **Limpeza**: Movimentos de vai-e-vem
6. **Desativação**: Fecha coletor

### Posições Configuráveis

```python
# Posições de corte (sequência segura)
x_corte_1: 50.0, y_corte_1: 200.0    # Posição inicial
x_corte_2: 50.0, y_corte_2: 226.0    # Extensão Y
x_corte_3: 6.0, y_corte_3: 226.0     # Extensão X
x_corte_final: 50.0, y_corte_final: 226.0

# Posições de purga
x_purga: 160.0, y_purga: 226.0       # Área de purga
z_min_purga: 36.0                    # Altura segura
```

---

## 📊 Estado Persistente

### Arquivos de Estado

- **Klipper**: `/usr/data/printer_data/config/current_filament.cfg`
- **Python**: `/usr/data/printer_data/config/chameleon_state.json`

### Funcionalidades

- **Auto-carregamento**: Estado carregado automaticamente na inicialização
- **Backup duplo**: Tanto Klipper quanto script Python
- **Sobrevivência**: Mantém estado após crashes/reinicializações
- **Sincronização**: RAM e arquivo sempre sincronizados

### Comandos de Gerenciamento

```gcode
LOAD_CURRENT_FILAMENT      ; Carrega do arquivo para RAM
CURRENT_FILAMENT          ; Mostra estado atual
SET_CURRENT_FILAMENT T=1  ; Define e salva
```

---

## 🔧 Manutenção e Troubleshooting

### Problemas Comuns

#### 1. Seletor não move
- Verificar conexões motor X (seletor)
- Testar `CFS_HOME`
- Verificar calibração de passos

#### 2. Buffer não alimenta
- Verificar sensor de filamento (A3)
- Testar sensores de buffer (A0/A1)
- Verificar posição do seletor

#### 3. Comunicação serial falha
- Verificar cabo USB Arduino
- Confirmar baud rate (9600)
- Testar `CFS_STATUS`

#### 4. Estado não persiste
- Verificar permissões do arquivo JSON
- Confirmar caminho correto
- Testar script Python isoladamente

### Calibração

#### Passos por mm:
```cpp
// Testar com comando LOAD/UNLOAD conhecido
// Medir filamento real vs esperado
// Ajustar STEPS_PER_MM conforme necessário
```

#### Posições do Seletor:
```gcode
CFS_HOME          ; Estabelece posição 0
T0, T1, T2        ; Testa sequencialmente
CFS_IDLE          ; Verifica posições idle
```

---

## 📈 Logs e Debug

### Níveis de Log

- **Serial Arduino**: Status em tempo real
- **Klipper Console**: Comandos e respostas
- **Estado**: Persistência de variáveis

### Comandos de Debug

```gcode
CFS_TEST_LOAD     ; Testa sistema completo
M118 "Debug message"  ; Logs customizados
CFS_STATUS        ; Status do Arduino
```

---

## 🤝 Contribuição

### Baseado em

Este projeto é uma adaptação do **3DChameleon Mk4** original:

- **Autor Original**: William J. Steele
- **Licença**: MIT
- **Repositório**: [3DChameleon GitHub](https://github.com/3DChameleon/3DChameleon)
- **Documentação**: Incluída na pasta `3DChameleonMk4/`

### Modificações Implementadas

#### 🔧 **Adaptações de Hardware**
1. **Arduino Uno + CNC Shield**: Remoção dependência SX1509
2. **Motores Stepper**: Controle direto via drivers A4988
3. **Sensores NC**: Microswitches para detecção de filamento/buffer

#### 📦 **Integrações Já Prontas Utilizadas**
1. **Sistema de Buffer**: `FILAMENT_BUFFER_MOUNT.stl` adaptado + **TurtleNeck buffer**
2. **TurtleNeck**: Sistema de buffer de filamento para reduzir tensão na extrusora
3. **Hub PTFE 3-para-1**: Distribuição `ptfe3to1hub_v2.stl`
4. **3DChameleon Base**: Estrutura A/B Parts + componentes

#### 💻 **Desenvolvimento de Software**
1. **Integração Klipper**: Comunicação serial completa
2. **Sistema Buffer**: Automação de alimentação com sensores
3. **Estado Persistente**: JSON independente do Klipper
4. **Coletor de Purga**: Servo 360° para Creality K1
5. **Macros Automáticas**: T0-T2 com troca completa
6. **Configuração Creality K1**: Perfis específicos


## 📜 Licença

Este projeto mantém a **Licença MIT** do projeto original 3DChameleon.

```
Copyright 2024 William J. Steele (3DChameleon Original)
Adapted for Creality K1 by [Your Name]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```

---


### Documentação Adicional

- [Klipper G-code Shell Command](https://github.com/dw-0/kiauh/blob/master/docs/gcode_shell_command.md)
- [3DChameleon Original Documentation](https://github.com/3DChameleon/3DChameleon)
- [Creality K1 Integration](https://www.creality.com/products/creality-k1)
- [Turtle Neck](https://github.com/ArmoredTurtle/TurtleNeck)

---

## 🔄 Changelog

### v1.0 - Adaptação Completa para K1/K1C
- ✅ **Sistema de Buffer**: Integração completa com `FILAMENT_BUFFER_MOUNT.stl` + **TurtleNeck buffer**
- ✅ **TurtleNeck**: Sistema de buffer de filamento para reduzir tensão na extrusora
- ✅ **Hub PTFE 3-para-1**: Distribuição `ptfe3to1hub_v2.stl` implementada
- ✅ **Adaptação Arduino Uno + CNC Shield**: Sem dependência SX1509
- ✅ **Integração Klipper completa**: Comunicação serial 9600 baud
- ✅ **Estado persistente independente**: JSON + Klipper save_variables
- ✅ **Coletor de purga servo 360°**: Sequência automática
- ✅ **Macros T0-T2**: Troca automática com corte e purga
- ✅ **Configuração específica Creality K1/K1C**: Perfis otimizados

---

## 🚀 Status do Projeto

### ✅ **Pronto para Uso**

| Componente | Status | Descrição |
|------------|--------|-----------|
| **Sistema de Buffer** | ✅ Funcional | Alimentação automática com `FILAMENT_BUFFER_MOUNT.stl` |
| **TurtleNeck (Buffer)** | ✅ Modelos Disponíveis | Sistema de buffer TurtleNeck para filamento |
| **Hub PTFE 3-para-1** | ✅ Implementado | Distribuição de filamentos `ptfe3to1hub_v2.stl` |
| **Firmware Arduino** | ✅ Completo | Comunicação serial + controle de motores |
| **Configuração Klipper** | ✅ Funcional | Macros T0-T2 + estado persistente |
| **Estrutura 3DChameleon** | ✅ Adaptada | Peças A/B + componentes para K1 |

---

*Este projeto é uma adaptação independente mantendo compatibilidade com o ecossistema 3DChameleon original.*