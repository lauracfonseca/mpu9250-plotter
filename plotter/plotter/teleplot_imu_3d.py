"""Recebe um MPU9250 pela USB Serial e envia seu cubo 3D ao Teleplot."""

import math
import socket
import time

try:
    import serial  # pacote pyserial: py -m pip install pyserial
except ImportError as erro:
    raise SystemExit("Instale a dependência: py -m pip install pyserial") from erro

# A porta precisa corresponder à porta USB do ESP32 no Windows.
PORTA_SERIAL = "COM17"
BAUDRATE = 115200
DESTINO_TELEPLOT = ("127.0.0.1", 47269)


def euler_para_quaternion(roll_graus: float, pitch_graus: float,
                          yaw_graus: float):
    """Converte roll/pitch/yaw, em graus, para quaternion x:y:z:w."""
    roll = math.radians(roll_graus) / 2.0
    pitch = math.radians(pitch_graus) / 2.0
    yaw = math.radians(yaw_graus) / 2.0

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def diferenca_angular(valor: float, referencia: float) -> float:
    """Retorna a diferença entre ângulos no intervalo [-180, 180)."""
    return (valor - referencia + 180.0) % 360.0 - 180.0


def ler_imu(linha: str):
    """Lê uma linha `IMU,roll,pitch,yaw`; retorna None para outras linhas."""
    partes = linha.strip().split(",")
    if len(partes) != 4 or partes[0] != "IMU":
        return None

    try:
        return float(partes[1]), float(partes[2]), float(partes[3])
    except ValueError:
        return None


def main():
    # timeout curto mantém o programa responsivo caso o sensor seja desconectado.
    usb = serial.Serial()
    usb.port = PORTA_SERIAL
    usb.baudrate = BAUDRATE
    usb.timeout = 0.1
    # Evita que o adaptador USB-serial reinicie o ESP32 ao abrir a COM.
    usb.dtr = False
    usb.rts = False
    usb.open()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Lendo {PORTA_SERIAL} a {BAUDRATE} baud; envie Ctrl+C para encerrar.")
    ultimo_status = 0.0
    orientacao_zero = None

    try:
        while True:
            # O ESP32 envia: IMU,roll,pitch,yaw
            linha = usb.readline().decode("utf-8", errors="replace").strip()
            if linha.startswith("#"):
                print(linha)
                continue

            leitura = ler_imu(linha)
            if leitura is None:
                continue

            roll, pitch, yaw = leitura
            if orientacao_zero is None:
                orientacao_zero = (roll, pitch, yaw)
                print("Referência zero definida; mantenha o sensor nivelado ao iniciar.")

            roll = diferenca_angular(roll, orientacao_zero[0])
            pitch = diferenca_angular(pitch, orientacao_zero[1])
            yaw = diferenca_angular(yaw, orientacao_zero[2])
            qx, qy, qz, qw = euler_para_quaternion(roll, pitch, yaw)
            timestamp = int(time.time() * 1000)

            # Sintaxe Teleplot: 3D|NOME:TIMESTAMP:PROPRIEDADES|MARCADORES.
            mensagem = (
                f"3D|imu_cube:{timestamp}:shape:cube:position:1:2:3:"
                f"quaternion:{qx:.6f}:{qy:.6f}:{qz:.6f}:{qw:.6f}:"
                f"color:green|g"
            )
            udp.sendto(mensagem.encode("utf-8"), DESTINO_TELEPLOT)

            # Confirma no terminal que os dados recebidos e enviados mudam.
            # Isso facilita separar um problema do sensor de um problema visual.
            agora = time.monotonic()
            if agora - ultimo_status >= 1.0:
                print(
                    f"IMU: roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f} | "
                    f"Q=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
                )
                ultimo_status = agora
    except KeyboardInterrupt:
        print("Envio interrompido.")
    finally:
        usb.close()
        udp.close()


if __name__ == "__main__":
    main()
