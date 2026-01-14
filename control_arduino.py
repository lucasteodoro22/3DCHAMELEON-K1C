#!/usr/bin/env python3
"""
Gerenciador de estado persistente para 3DChameleon
Salva/carrega variáveis em arquivo JSON independente do Klipper
"""

import sys
import os
import json
import time

# Arquivo de estado
STATE_FILE = "/usr/data/printer_data/config/chameleon_state.json"

def ensure_dir():
    """Garante que o diretório existe"""
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)

def load_state():
    """Carrega estado do arquivo JSON"""
    try:
        if os.path.exists(STATE_FILE):
            with open(STATE_FILE, 'r') as f:
                return json.load(f)
        return {}
    except Exception as e:
        print(f"Erro carregando estado: {e}")
        return {}

def save_state(state):
    """Salva estado no arquivo JSON"""
    try:
        ensure_dir()
        with open(STATE_FILE, 'w') as f:
            json.dump(state, f, indent=2)
        print(f"Estado salvo: {state}")
        return True
    except Exception as e:
        print(f"Erro salvando estado: {e}")
        return False

def set_variable(name, value):
    """Define uma variável"""
    state = load_state()

    # Converte valor para tipo apropriado
    try:
        if isinstance(value, str) and value.isdigit():
            value = int(value)
        elif isinstance(value, str) and value.replace('.', '').isdigit():
            value = float(value)
    except:
        pass

    state[name] = value

    if save_state(state):
        print(f"OK: {name} = {value}")
        return True
    else:
        print(f"ERRO salvando {name}")
        return False

def get_variable(name):
    """Obtém uma variável"""
    state = load_state()
    value = state.get(name)

    if value is not None:
        print(f"OK: {name} = {value}")
        return value
    else:
        print(f"NOT_FOUND: {name}")
        return None

def main():
    if len(sys.argv) < 3:
        print("Uso: python3 chameleon_state.py <acao> <nome_variavel> [valor]")
        print("Ações:")
        print("  SET <nome> <valor>  - Define variável")
        print("  GET <nome>          - Obtém variável")
        print("  SHOW                - Mostra todas")
        print("  CLEAR               - Limpa todas")
        sys.exit(1)

    action = sys.argv[1].upper()
    var_name = sys.argv[2]

    if action == "SET":
        if len(sys.argv) < 4:
            print("ERRO: SET precisa de valor")
            sys.exit(1)
        value = sys.argv[3]
        if not set_variable(var_name, value):
            sys.exit(1)

    elif action == "GET":
        if get_variable(var_name) is None:
            sys.exit(1)

    elif action == "SHOW":
        state = load_state()
        if state:
            print("Estado atual:")
            for k, v in state.items():
                print(f"  {k}: {v}")
        else:
            print("Estado vazio")

    elif action == "CLEAR":
        if save_state({}):
            print("Estado limpo")
        else:
            print("Erro limpando estado")
            sys.exit(1)

    else:
        print(f"Ação desconhecida: {action}")
        sys.exit(1)

if __name__ == "__main__":
    main()