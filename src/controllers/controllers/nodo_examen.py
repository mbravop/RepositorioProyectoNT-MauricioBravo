import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class NodoExamen(Node):
    def __init__(self):
        super().__init__('nodo_examen')

        #Publisher y suscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.listener_lidar, 10)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.listener_odom, 10)
        self.sub_teleop = self.create_subscription(Twist,'/teleop_cmd_vel',self.listener_teleop,10)

        self.velocidad_normal = 3.0
        self.velocidad_frenando = 0.8
        self.distancia_freno = 0.5
        self.distancia_min = 1.5

        self.ultimo_twist = Twist()
        self.distancia_cercana = 100000

        self.control = self.create_timer(0.02, self.loop_principal)

    #Guardo el ultimo mensaje recibido en el teleop_cmd_vel ya que hago un remapping del teleop.
    def listener_teleop(self, msg:Twist):
        self.ultimo_twist = msg

    #Defino el rango de frente que quiero tener en cuenta (indices) y guardo la distancia minima del lidar
    def listener_lidar(self, msg:LaserScan):
        #Rango para observar solo al frente en un foco. Los indices los supe de la tarea anterior de analizar lidar.
        start_index = 540 - 180
        end_index = 540 + 180

        frontal_scan = msg.ranges[start_index:end_index]

        scan_valido = [d for d in frontal_scan if d>0 and np.isfinite(d)]

        if scan_valido:
            self.distancia_cercana = min(scan_valido)
        else:
            self.distancia_cercana = 100000


    #Funcion que se encarga de definir la velocidad del carro segun las distancias
    def loop_principal(self):
        mensaje_nuevo = Twist()
        #Si la minima distancia del lidar es menor a 0.5 se frena por completo
        if self.distancia_cercana < self.distancia_freno:
            mensaje_nuevo.linear.x = 0.00
            mensaje_nuevo.angular.z = 0.0
            self.get_logger().warn(f'Freno de emergencia! Distancia: {self.distancia_cercana:.2f}')
        #Si la minima distancia del lidar es menor a 1.5 cambia la velocidad a 0.8
        elif self.distancia_cercana < self.distancia_min:
            mensaje_nuevo.linear.x = self.velocidad_frenando
            mensaje_nuevo.angular.z = self.ultimo_twist.angular.z
            self.get_logger().warn(f'Frenando... Distancia: {self.distancia_cercana:.2f}')
        
        #Si no entra en las condiciones para el frenado, se mantienen los valores previos.
        else:
            if self.ultimo_twist.linear.x > 0:
                mensaje_nuevo.linear.x = self.velocidad_normal
            else:
                mensaje_nuevo.linear.x = self.ultimo_twist.linear.x

            mensaje_nuevo.angular.z = self.ultimo_twist.angular.z
        
        self.publisher.publish(mensaje_nuevo)       
    
    #Para imprimir la posicion actual del carro en todo momento
    def listener_odom(self,msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        self.get_logger().info(f'Posicion actual: x: {current_x:.2f} y: {current_y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = NodoExamen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
