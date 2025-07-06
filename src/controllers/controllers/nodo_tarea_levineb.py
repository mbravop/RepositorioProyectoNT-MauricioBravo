import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math

class NodoTareaLevineB(Node):
    def __init__(self):
        super().__init__('nodo_levineb')
        
        # Definidos previamente al utilizar odom al pasar por el mapa.
        self.waypoints = [
            [8.39, 0.02], #primer inicio de giro
            [9.42, 0.83], #primera salida de giro
            [9.42, 7.92], #segundo...
            [8.61, 8.77],
            [-12.82, 8.82], 
            [-13.95, 7.79], 
            [-13.85, 1.02], 
            [-12.98, 0.02]
        ]
        self.current_waypoint_index = 0

        self.distance_threshold = 0.2
        self.speed = 1.5
        self.max_steering = 0.4
        self.steering_gain = 1.2
        
        self.log_counter = 0 
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        num = 2.0 * (q.w * q.z + q.x * q.y)#sen yaw cos pitch
        den = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)#cos yaw cos pitch

        current_yaw = math.atan2(num, den)
        
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        if distance < self.distance_threshold:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            return

        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        angle_error = angle_to_target - current_yaw
        
        while angle_error > math.pi: 
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: 
            angle_error += 2.0 * math.pi

        steering_angle = self.steering_gain * angle_error
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))

        self.log_counter += 1
        if self.log_counter % 25 == 0:
            estado_movimiento = ""
            if abs(steering_angle) < math.radians(1):
                estado_movimiento = "Avanzando recto"
            elif steering_angle > 0:
                estado_movimiento = f"Girando a la izquierda (ángulo: {steering_angle:.2f})"
            else:
                estado_movimiento = f"Girando a la derecha (ángulo: {steering_angle:.2f})"
            
            self.get_logger().info(f"-> Waypoint: {self.current_waypoint_index} | Dist: {distance:.1f}m | {estado_movimiento}")
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NodoTareaLevineB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()