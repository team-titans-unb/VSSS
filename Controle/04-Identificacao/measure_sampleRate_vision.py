import socket
import struct
import time
import wrapper_pb2 as wr

# Configuração do sistema de visão
VISION_IP = "224.5.23.2"  # IP multicast do sistema de visão
VISION_PORT = 10011  # Porta do sistema de visão

def measure_intervals():
    """
    Mede o tempo entre as amostras de dados recebidas do sistema de visão computacional.
    """
    try:
        # Inicializa o socket UDP para receber pacotes do sistema de visão
        vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        vision_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                               struct.pack("=4sl", socket.inet_aton(VISION_IP), socket.INADDR_ANY))
        vision_sock.bind((VISION_IP, VISION_PORT))
        print("Listening for vision data...")

        # Variáveis para medir o intervalo entre pacotes
        last_time = None
        intervals = []

        # Loop para receber pacotes
        while True:
            try:
                # Recebe um pacote de dados
                data, _ = vision_sock.recvfrom(1024)
                current_time = time.time()  # Marca o tempo de recebimento do pacote

                # Calcula o intervalo entre pacotes
                if last_time is not None:
                    interval = current_time - last_time
                    intervals.append(interval)
                    print(f"Interval: {interval:.6f} seconds")

                last_time = current_time  # Atualiza o tempo do último pacote recebido

            except KeyboardInterrupt:
                # Interrompe o loop com Ctrl+C
                print("\nMeasurement stopped by user.")
                break

            except Exception as e:
                print(f"Error receiving vision data: {e}")
                break

        # Exibe estatísticas dos intervalos
        if intervals:
            print("\n--- Statistics ---")
            print(f"Total packets: {len(intervals)}")
            print(f"Average interval: {sum(intervals) / len(intervals):.6f} seconds")
            print(f"Minimum interval: {min(intervals):.6f} seconds")
            print(f"Maximum interval: {max(intervals):.6f} seconds")
        else:
            print("No intervals measured.")

    except Exception as e:
        print(f"Error setting up vision system: {e}")

if __name__ == "__main__":
    measure_intervals()