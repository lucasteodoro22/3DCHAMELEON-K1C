import serial
import sys
import time

# Porta que encontramos no seu teste
device = '/dev/ttyACM0'
baud = 9600

def enviar():
    if len(sys.argv) < 2:
        print("Uso: python control_arduino.py <comando>")
        print("Exemplos:")
        print("  python control_arduino.py T0")
        print("  python control_arduino.py LOAD 100")
        print("  python control_arduino.py STATUS")
        return

    comando = sys.argv[1] + "\n"

    # Adicionar argumentos adicionais se houver (para LOAD/UNLOAD com distancia)
    if len(sys.argv) > 2:
        comando = sys.argv[1] + " " + sys.argv[2] + "\n"

    try:
        # Verificar se a porta existe
        import os
        if not os.path.exists(device):
            print(f"ERRO: Porta {device} não encontrada")
            return

        # Abrir porta, enviar e fechar
        ser = serial.Serial(device, baud, timeout=1)
        time.sleep(0.1) # Pequena pausa para estabilidade

        # Limpar buffer de entrada
        ser.reset_input_buffer()

        # Enviar comando
        ser.write(comando.encode())
        print(f"Enviado: {comando.strip()}")

        # Aguardar e ler resposta (até 2 segundos)
        time.sleep(0.5)
        if ser.in_waiting > 0:
            resposta = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print("Resposta do Arduino:")
            print(resposta.strip())

        ser.close()

    except serial.SerialException as e:
        print(f"ERRO de comunicação serial: {e}")
        with open("/tmp/arduino_error.log", "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Serial error: {e}\n")
    except Exception as e:
        print(f"ERRO geral: {e}")
        with open("/tmp/arduino_error.log", "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - General error: {e}\n")

if __name__ == "__main__":
    enviar()
