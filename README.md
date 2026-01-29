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
- **Servo Pin 11**: Coletor de purga (PURGA_COLETOR_PIN)
- **Sensores de filamento por ferramenta** (NC): T0=D9 (X Endstop), T1=D10 (Y Endstop), T2=D13 (SpnDir)
- **Sensor de filamento no hotend**: A3 (CoolEnd)
- **Sensor de alimentação** (NO): T1=A2 (Resume) — monitora filamento pré-carregado
- **Sensores de buffer** (NC): A0 (HOLD), A1 (ABORT)
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
- **2x Microswitches NC** (sensores de buffer: A0, A1)
- **1x Microswitch NC** (sensor de filamento no hotend: A3)
- **3x Microswitches NC** (sensores de filamento por ferramenta: T0=D9, T1=D10, T2=D13)
- **1x Microswitch NO** (opcional, sensor de alimentação T1: A2)

### Motores
- **Motor Stepper 1**: Seletor de ferramentas (NEMA 17)
- **Motor Stepper 2**: Extrusora/buffer (NEMA 17)

### Sensores
- **Sensor de filamento no hotend**: Microswitch NC (A3 - CoolEnd)
- **Sensores de filamento por ferramenta** (NC): T0=D9 (X Endstop), T1=D10 (Y Endstop), T2=D13 (SpnDir)
- **Sensor de alimentação T1** (NO, opcional): A2 (Resume)
- **Buffer Empty**: Microswitch NC (A1 - ABORT)
- **Buffer Full**: Microswitch NC (A0 - HOLD)

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
- **Sensores**: Filamento no hotend (A3) + sensores por ferramenta (T0=D9, T1=D10, T2=D13) + buffer (A0, A1) + alimentação T1 (A2, opcional)
- **UNLOAD por sensor**: Comando `UNLOAD` sem distância — para quando sensor indica ausência de filamento
- **Load até sensor**: `loadUntilSensorAlimentacaoDeFilamento(toolNumber)` — carrega até sensor e retorna à ferramenta original

#### 🖥️ **Integração Klipper**
- **Status**: ✅ COMPLETA
- **Macros**: T0-T2 com troca automática
- **Estado**: Persistente entre reinicializações
- **Shell Command**: Python bridge funcionando

---

## 🛒 Lista de Materiais

### Eletrônica e Componentes

| Item | Quantidade | Valor Aproximado | Link de Compra |
|------|------------|------------------|----------------|
| **Arduino Uno** | 1 | R$36 | [Mercado Livre](https://www.mercadolivre.com.br/uno-r3-smd-chip-compativel-arduino/p/MLB38492604?pdp_filters=item_id:MLB5436735706#is_advertising=true&searchVariation=MLB38492604&backend_model=search-backend&position=1&search_layout=grid&type=pad&tracking_id=e2d4de71-5b96-4051-85b2-c6d9ee0eef8d&ad_domain=VQCATCORE_LST&ad_position=1&ad_click_id=MWZmZGRiZjQtM2VhNS00OWQ5LWI4MTYtZTVlYmFmYWI3N2Jj) |
| **CNC Shield v3** | 1 | R$ 27 | [Mercado Livre](https://www.mercadolivre.com.br/cnc-shield-v3-arduino-impressora-3d-reprap-grbl/p/MLB2039012117?pdp_filters=item_id%3AMLB2794264804#is_advertising=true&searchVariation=MLB2039012117&backend_model=search-backend&position=2&search_layout=grid&type=pad&tracking_id=e972754f-ce16-459b-8fdd-c8791484b1d4&ad_domain=VQCATCORE_LST&ad_position=2&ad_click_id=OWUzMjIzZDEtMWNiZi00N2U1LWI5OWEtM2ZkMjk5MzUyODUz) |
| **Driver A4988** | 2 | R$ 27 | [Mercado Livre](https://www.mercadolivre.com.br/2x-driver-motor-de-passo-a4988-com-dissipador-8-35v-2a/p/MLB2039737210#polycard_client=search-desktop&search_layout=grid&position=10&type=product&tracking_id=8593d4dd-4d70-4023-9198-078e31411894&wid=MLB2168520580&sid=search) |
| **Servo 360°** | 1 | R$ 38 | [Mercado Livre](https://www.mercadolivre.com.br/servo-digital-blutu-mg996r-360-metal-15kg-alto-torque-arduino/p/MLB40989790?pdp_filters=item_id:MLB5071204762#is_advertising=true&searchVariation=MLB40989790&backend_model=search-backend&position=1&search_layout=grid&type=pad&tracking_id=e40326b4-68c1-454e-90bc-0661325d6a58&ad_domain=VQCATCORE_LST&ad_position=1&ad_click_id=YTViOTQxNjYtN2FiZi00MzA5LWE3MDEtY2FhYWJkM2YwYTBh) |
| **Microswitch NC** | 5 | R$19 | [Mercado Livre](https://www.mercadolivre.com.br/chave-fim-de-curso-kw10-b-micro-switch-com-haste-10/p/MLB44351264?pdp_filters=item_id:MLB5857056256) — hotend + buffer (2) + por ferramenta (3) |
| **Microswitch NO** | 1 | opcional | Sensor de alimentação T1 (A2) |
| **NEMA 17 Stepper Motor** | 2 | R$ 60 cada | [Mercado Livre](https://www.mercadolivre.com.br/motor-de-passo-nema-17-42kgf-14a-ender-3-impressora-3d/p/MLB2045385578?pdp_filters=item_id%3AMLB5278821864#is_advertising=true&searchVariation=MLB2045385578&backend_model=search-backend&position=1&search_layout=grid&type=pad&tracking_id=491bc2e5-8554-4de2-a27e-176f5c5717d8&ad_domain=VQCATCORE_LST&ad_position=1&ad_click_id=MTM3NDRlOWEtZDA5OC00MjliLWEzOWItNTg2ODc3ODhhMWI3) |
| **PTFE Tubing 4mmx2,5mm OD** | 4m | R$ 70 | [Mercado Livre](https://www.mercadolivre.com.br/tubo-ptfe-para-bambu-lab-a1-mini-25mm-x-40mm/up/MLBU3464340835?pdp_filters=item_id:MLB4237039359) |
| **Conectores PTFE PC4-M10** | R$45 | [Mercado Livre](https://www.mercadolivre.com.br/kit-5-conectores-bowden-pc401-m10-pneumatico-hotend-ptfe/up/MLBU734237438?pdp_filters=item_id:MLB1653440903) |
| **Engrenagem MK8** | 2 | R$10 cada | [Mercado Livre](https://www.mercadolivre.com.br/1-peca-engrenagem-mk8-filamento-175mm-para-impressora-3d/up/MLBU1714610359?pdp_filters=item_id:MLB1953785178) |
| **Parafuso e Porcas M3x30mm** | 8 | R$23 | [Mercado Livre](https://www.mercadolivre.com.br/parafuso-com-cabeca-allen-m3-x-30mm-inox--20-pecas/up/MLBU1967887623#polycard_client=search-desktop&search_layout=grid&position=7&type=product&tracking_id=5a989f37-af0b-4e14-af35-d42bb9760db1&wid=MLB2797256477&sid=search) |
| **Parafuso e Porcas M3x12mm** | 8 | R$20 | [Mercado Livre](https://www.mercadolivre.com.br/kit-parafuso-maquina-allen-m3-x-12mm-cilindrico-20-unidades-ciser/p/MLB47070098?pdp_filters=item_id:MLB4000230637&matt_tool=48517109&matt_internal_campaign_id=301620185&matt_word=&matt_source=google&matt_campaign_id=22883155151&matt_ad_group_id=184598904365&matt_match_type=&matt_network=g&matt_device=c&matt_creative=776720678099&matt_keyword=&matt_ad_position=&matt_ad_type=pla&matt_merchant_id=211153465&matt_product_id=MLB4000230637&matt_product_partition_id=2442743515081&matt_target_id=pla-2442743515081&cq_src=google_ads&cq_cmp=22883155151&cq_net=g&cq_plt=gp&cq_med=pla&gad_source=1&gad_campaignid=22883155151&gbraid=0AAAAAD93qcC1BR4N9Z3Va3Myw2VpsMt7r&gclid=CjwKCAiAj8LLBhAkEiwAJjbY73Zq_wj2ePMLExqrYDkM1SymknsZygud99U0mP4LVqO__YWdUIlZLhoCyYIQAvD_BwE) |
| **Rolamento 625zz** | 4 | R$ 39 | [Mercado Livre](https://www.mercadolivre.com.br/kit-10-micro-rolamento-625-zz--625zz-5x16x5mm--aco-carbono/up/MLBU730803302?pdp_filters=item_id:MLB1131351895) |
| **Rolamento 6800zz** | 2 | R$ 23 | [Mercado Livre](https://www.mercadolivre.com.br/kit-5-rolamentos-6800-zz-medidas-10x19x5mm-cinza/p/MLB56414559?pdp_filters=item_id:MLB5709617128) |

---

## ⚙️ Instalação e Configuração

### 1. Preparação do Hardware

#### Conexões Arduino/CNC Shield:

```cpp
// Eixo Z - Extrusora (alimentação de filamento)
#define extEnable 12   // Z_ENABLE
#define extStep 4      // Z_STEP
#define extDir 7       // Z_DIR

// Eixo X - Seletor (seleção de ferramenta)
#define selEnable 8    // X_ENABLE
#define selStep 2      // X_STEP
#define selDir 5       // X_DIR

// Servo e sensores
#define PURGA_COLETOR_PIN 11       // Coletor de purga (Z+ Endstop)
#define FILAMENT_SENSOR_PIN A3     // Sensor no hotend (CoolEnd)

// Sensores de filamento por ferramenta (NC) - UNLOAD/Load até sensor
#define FILAMENT_SENSOR_T0_PIN 9   // X Endstop
#define FILAMENT_SENSOR_T1_PIN 10  // Y Endstop
#define FILAMENT_SENSOR_T2_PIN 13  // SpnDir

// Sensor de alimentação T1 (NO, opcional)
#define FILAMENT_ALIMENTATION_SENSOR_T1_PIN A2  // Resume

// Buffer (NC)
#define BUFFER_EMPTY_PIN A1   // ABORT - buffer vazio
#define BUFFER_FULL_PIN A0   // HOLD - buffer cheio
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

#### Load e Unload (comandos serial para o Arduino):
- **LOAD \<mm\>**: Carrega filamento até sensor do hotend, depois avança distância em mm
- **UNLOAD**: Descarrega filamento até sensor da ferramenta indicar ausência (parada automática) — **sem precisar informar distância**
- **UNLOAD \<mm\>**: Descarrega por distância fixa (modo legado)
- **UNLOAD_RETRACAO \<mm\>**: Descarrega com velocidade fixa (5 mm/s) para sincronização com retração do Klipper

A função **loadUntilSensorAlimentacaoDeFilamento(toolNumber)** carrega filamento até o sensor da ferramenta detectar presença; move o seletor para a ferramenta, carrega até o sensor e retorna para a posição original.

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
#define BUFFER_CHECK_INTERVAL 100   // Verificação a cada 100ms
#define IDLE_TIMEOUT 120000         // 2 minutos sem atividade
const float STEPS_PER_MM = 151.0;   // Calibração passos/mm
```

### Load/Unload com sensores por ferramenta

- **UNLOAD** (sem distância): usa o sensor da ferramenta ativa (T0→D9, T1→D10, T2→D13) para parar quando o filamento sair (sensor NC: LOW = sem filamento).
- **loadUntilSensorAlimentacaoDeFilamento(toolNumber)**: move para a ferramenta, carrega até o sensor detectar filamento (NC: HIGH = presente), depois retorna para a ferramenta original.
- Sensores NC: HIGH = filamento presente, LOW = sem filamento. Sensor de alimentação T1 (NO): HIGH = presente, LOW = ausente.

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
- [BOM Completa 3DChameleon Mk4](STLs/3DChameleonMk4/3DChameleon%20Mk4%20Electronics/BOM_3D-Chameleon-MK4_2024-12-13.csv)

---

## 🔄 Changelog

### v1.1 - Sensores por ferramenta e UNLOAD/Load por sensor
- ✅ **Sensores de filamento por ferramenta** (NC): T0=D9, T1=D10, T2=D13 (X/Y Endstop, SpnDir)
- ✅ **UNLOAD sem distância**: Comando `UNLOAD` para automaticamente até sensor indicar ausência de filamento
- ✅ **loadUntilSensorAlimentacaoDeFilamento(toolNumber)**: Carrega até sensor da ferramenta e retorna à posição original
- ✅ **Sensor de alimentação T1** (NO): A2 (Resume) para monitorar filamento pré-carregado
- ✅ **Documentação e comentários**: Código e README em português; pinos e lógica NC/NO documentados

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
| **Sensores por ferramenta** | ✅ Implementado | T0=D9, T1=D10, T2=D13 — UNLOAD/Load até sensor |
| **UNLOAD por sensor** | ✅ Implementado | Parada automática quando filamento sai (sem distância) |
| **Load até sensor** | ✅ Implementado | loadUntilSensorAlimentacaoDeFilamento + retorno à ferramenta |
| **TurtleNeck (Buffer)** | ✅ Modelos Disponíveis | Sistema de buffer TurtleNeck para filamento |
| **Hub PTFE 3-para-1** | ✅ Implementado | Distribuição de filamentos `ptfe3to1hub_v2.stl` |
| **Firmware Arduino** | ✅ Completo | Serial + motores + sensores NC/NO documentados |
| **Configuração Klipper** | ✅ Funcional | Macros T0-T2 + estado persistente |
| **Estrutura 3DChameleon** | ✅ Adaptada | Peças A/B + componentes para K1 |

---

*Este projeto é uma adaptação independente mantendo compatibilidade com o ecossistema 3DChameleon original.*