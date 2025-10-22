#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

listen_topic: str = "robot_voice"
queue_size: int = 50 

class RoboListener(Node):

    def __init__(self) -> None:
        super().__init__("robo_listener")

        #TODO: DECLARAR PARAMETROS CONFIGURABLES
        self.declare_parameter("enable_tts", True)
        self.declare_parameter("tts_voice", "en-us+f2") #VOZ DE MUJER
        self.declare_parameter("tts_speed", 190) # VELOCIDAD DE VOZ

        #TODO: AQUI CARGAMOS LOS PARAMETROS
        self._enable: bool = bool(self.get_parameter("enable_tts").value)
        self._voice: str = str(self.get_parameter("tts_voice").value)
        self._speed: int = int(self.get_parameter("tts_speed").value)
        self.get_logger().info(f"TTS Listener iniciado. Voz: {self._voice}, velocidad: {self._speed}")

        #TODO: SUSCRIPCION AL TOPICO
        self.subscription = self.create_subscription(String, listen_topic, self._on_message, queue_size)

    def _on_message(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("Mensaje vacío recibido, se ignora")
            return
        self.get_logger().info(f"Recibido: '{text}'")
        if self._enable:
            self._speak(text)

    def _speak(self, text: str) -> None:
        try:
            subprocess.run([
                "espeak-ng",
                "-v", self._voice,
                "-s", str(self._speed),
                text
            ], check=True)
        except FileNotFoundError:
            self.get_logger().error("No se encontró 'espeak-ng'. Instálalo para usar TTS.")
        except subprocess.CalledProcessError as exc:
            self.get_logger().error(f"Error ejecutando espeak-ng: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoboListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RoboListener detenido por teclado")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()