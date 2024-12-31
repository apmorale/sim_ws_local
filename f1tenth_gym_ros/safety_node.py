import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import numpy as np

class SafetyNode (Node):
    def __init__(self):
        super().__init__('safety_node')
        # Suscripcion a /scan (LaserScan) y /ego_racecar/odom (Odometry)
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10) 
        self.odom_sub = self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)

        # Publicar en /dcmd (AckermannDriveStamped)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Dimensiones del robot - Largo y ancho en metros
        self.robot_length = 0.5
        self.robot_width = 0.3 

        # Ubicacion del sensor con respecto al robot
        self.sensor_offset_x1 = 0.23
        self.sensor_offset_x2 = 0.27
        self.sensor_offset_y1 = 0.15
        self.sensor_offset_y2 = 0.15

        self.velocity = 0.0 # Velocidad de paro de emergencia, se detiene por completo
        self.ttc_threshold = 1.1 # Umbral de tiempo hasta la colision en segundos
    
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        for i, r in enumerate(ranges):
            angle = angles[i]
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            ttc = self.calculate_ttc(r)

            if ttc< self.ttc_threshold and self.is_within_robot_zone(x,y):
                self.emergency_brake()
                break
    
    def odom_callback(self, msg: Odometry):
        self.velocity = msg.twist.twist.linear.x
        self.get_logger().info(f"Vehicle velocity: {self.velocity}")

    def calculate_ttc(self, distance):
        if self.velocity > 0:
            return distance / self.velocity
        return float('inf')
    
    def is_within_robot_zone(self, x, y):
        sensor_radius_1 = np.sqrt(self.sensor_offset_x1**2 + self.sensor_offset_y1**2)
        sensor_radius_2 = np.sqrt(self.sensor_offset_x2**2 + self.sensor_offset_y2**2)

        distance_1 = np.sqrt(x**2 + y**2)
        distance_2 = np.sqrt((x - self.sensor_offset_x1)**2 + (y - self.sensor_offset_y1**2))
        distance_3 = np.sqrt((x - self.sensor_offset_x2)**2 + (y - self.sensor_offset_y2**2))

        return distance_1 < sensor_radius_1 or distance_2 < sensor_radius_2 or distance_3 < sensor_radius_2
    
    def emergency_brake(self):
        # Crear un mensaje Twist con velocidades lineal y angular en 0
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        
        # Publicar el mensaje en /cmd_vel
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('¡Freno de emergencia activado!')
    
def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()