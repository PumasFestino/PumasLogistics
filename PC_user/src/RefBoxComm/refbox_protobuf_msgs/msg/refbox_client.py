import socket
import struct
from GameState_pb2 import GameState

def connect_to_refbox(host, port):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        print(f"Conectado a {host}:{port}")
        return s
    except Exception as e:
        print(f"Error de conexión: {str(e)}")
        return None

def main():
    HOST = "localhost"
    PORT = 4444  # GameState
    
    sock = connect_to_refbox(HOST, PORT)
    if not sock:
        return
    
    try:
        while True:
            # Recibir longitud del mensaje (4 bytes big-endian)
            len_bytes = sock.recv(4)
            if not len_bytes:
                print("Conexión cerrada por el servidor")
                break
                
            msg_len = struct.unpack(">I", len_bytes)[0]
            print(f"Esperando mensaje de {msg_len} bytes...")
            
            # Recibir datos completos
            data = b""
            while len(data) < msg_len:
                packet = sock.recv(msg_len - len(data))
                if not packet:
                    break
                data += packet
            
            # Decodificar protobuf
            game_state = GameState()
            game_state.ParseFromString(data)
            
            # Mostrar información básica
            print(f"\n--- Estado del juego ---")
            print(f"Fase: {game_state.game_phase}")
            print(f"Tiempo: {game_state.game_time}")
            print(f"Puntos Cyan: {game_state.points_cyan}")
            print(f"Puntos Magenta: {game_state.points_magenta}")
            
    except KeyboardInterrupt:
        print("\nDesconectado")
    finally:
        sock.close()

if __name__ == "__main__":
    main()