#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

publish_topic: str = "robot_voice"
queue_size: int = 50

class RoboTalker(Node):
    
    #TODO: NODO QUE PUBLICA EL MENSAJE
    def __init__(self) -> None:
        super().__init__("robo_talker")

        self.publisher_ = self.create_publisher(String, publish_topic, queue_size)
        self.get_logger().info("Justina lista, Write your message and press ENTER:")

    def publish_text(self, text: str) -> None:
        #TODO: PUBLICA EL MENSAJE DEL TOPICO
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publicado: '{text}'")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoboTalker()
    try:
        while rclpy.ok():
            text = input("> ")
            if text.strip() == "":
                node.get_logger().warn("Mensaje vacío")
                continue
            node.publish_text(text)
    except KeyboardInterrupt:
        node.get_logger().info("RoboTalker detenido por teclado")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
