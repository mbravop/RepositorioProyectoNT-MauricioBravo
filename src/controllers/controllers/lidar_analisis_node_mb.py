import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarAnalisis(Node):
    def __init__(self):
        super().__init__('analizador_lidar_node') 
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.analyzer, 10)
        self.last_time_sec = None
        self.last_time_nanosec = None
        self.message_count = 0
        self.publication_frequency = 0.0
    
    def analyzer(self, msg):
        self.message_count += 1

        current_time_sec = msg.header.stamp.sec
        current_time_nanosec = msg.header.stamp.nanosec
        
        if self.last_time_sec is not None:
            time_interval = (current_time_sec - self.last_time_sec) + (current_time_nanosec - self.last_time_nanosec) / 1e9
            
            if time_interval > 0:
                self.publication_frequency = 1.0 / time_interval

        self.last_time_sec = current_time_sec
        self.last_time_nanosec = current_time_nanosec

        angular_resolution_deg = math.degrees(msg.angle_increment)
        ranges = msg.ranges

        index_menor90 = self.indiceAngulo(msg, -math.pi / 2.0)
        index_mayor90 = self.indiceAngulo(msg, math.pi / 2.0)
        front_ranges = ranges[index_menor90 : index_mayor90 + 1]

        rear_ranges_antes = ranges[0 : index_menor90]
        rear_ranges_despues = ranges[index_mayor90 + 1 :]
        rear_ranges = rear_ranges_antes + rear_ranges_despues
            
        self.get_logger().info("\n"
            "Análisis de Datos LiDAR:\n"
            f"Resolución Angular: {angular_resolution_deg:.4f} grados\n"
            f"Frecuencia Estimada: {self.publication_frequency:.2f} Hz\n"
            "--------------------\n"
            "División de Mediciones:\n"
            f"Mediciones Frontales: {len(front_ranges)} puntos\n"
            f"Ejemplo: {[round(r, 2) for r in front_ranges[:5]]}...\n"
            f"Mediciones Traseras: {len(rear_ranges)} puntos\n"
            f"Ejemplo: {[round(r, 2) for r in rear_ranges[:5]]}...\n"
            + "="*50
            )
        
    def indiceAngulo(self, msg, angle_rad):
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)
        return max(0, min(index, len(msg.ranges) - 1))


def main(args=None):
    rclpy.init(args=args)
    lap_logger_node = LidarAnalisis()
    rclpy.spin(lap_logger_node)
    lap_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()