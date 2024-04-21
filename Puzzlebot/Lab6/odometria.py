import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

class My_Process(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.sub_velL = self.create_subscription(Float32, 'VelocityEncL', self.process_callback_velL, rclpy.qos.qos_profile_sensor_data)
        self.sub_velR = self.create_subscription(Float32, 'VelocityEncR', self.process_callback_velR, rclpy.qos.qos_profile_sensor_data)

        self.distancePublisher = self.create_publisher(Float32, 'distance', rclpy.qos.qos_profile_sensor_data)
        self.linearPublisher= self.create_publisher(Float32, 'linear_speed', rclpy.qos.qos_profile_sensor_data)
        self.orientationPublisher = self.create_publisher(Float32, 'orientation', rclpy.qos.qos_profile_sensor_data)
        self.angularPublisher = self.create_publisher(Float32, 'angular_speed', rclpy.qos.qos_profile_sensor_data)
        self.xPublisher = self.create_publisher(Float32, 'positionX', rclpy.qos.qos_profile_sensor_data)
        self.yPublisher = self.create_publisher(Float32, 'positionY', rclpy.qos.qos_profile_sensor_data)

        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Process node successfully initialized!')

        self.msg_x = Float32()  # Posición en el eje x
        self.msg_y = Float32()  # Posición en el eje y
        self.msg_theta = Float32()  # Orientación del robot
        self.msg_distance = Float32() # Distancia del robot
        self.msg_linear = Float32()
        self.msg_angular = Float32()

        self.radius = 0.05
        self.base = 0.18

        self.old_distance = 0.0
        self.old_theta = 0.0
        self.old_x = 0.0
        self.old_y = 0.0

        self.last_velL = 0.0
        self.last_velR = 0.0

    
    def process_callback_velL(self, msg):
        self.last_velL = msg.data

    def process_callback_velR(self, msg):
        self.last_velR = msg.data

    def timer_callback(self):
        # Cálculo de la velocidad lineal
        linear_speed = (self.radius * self.last_velR + self.radius * self.last_velL) / 2.0
        self.msg_linear.data = linear_speed

        # Cálculo de la velocidad angular
        angular_speed = (self.radius * self.last_velR - self.radius * self.last_velL) / self.base
        self.msg_angular.data = angular_speed

        # Cálculo de distancia y orientación
        if (linear_speed < 0):
            linear_speed_distance = linear_speed * -1
        else:
            linear_speed_distance = linear_speed
        distance = self.old_distance + (linear_speed_distance * self.timer_period)
        self.msg_distance.data = distance
        theta = (self.old_theta + (angular_speed * self.timer_period)) % 6.28318530718
        self.msg_theta.data = theta
    
        # Cálculo de posición global en coordenadas(x,y)
        speed_x = linear_speed * np.cos(theta)
        speed_y = linear_speed * np.sin(theta)
        x = self.old_x + (speed_x * self.timer_period)
        self.msg_x.data = x
        y = self.old_y + (speed_y * self.timer_period)
        self.msg_y.data = y

        # Actualización de datos anteriores
        self.old_distance = distance
        self.old_theta = theta
        self.old_x = x
        self.old_y = y

        # Publicación de mensajes
        self.distancePublisher.publish(self.msg_distance)
        self.linearPublisher.publish(self.msg_linear)
        self.orientationPublisher.publish(self.msg_theta)
        self.angularPublisher.publish(self.msg_angular)
        self.xPublisher.publish(self.msg_x)
        self.yPublisher.publish(self.msg_y)

        self.get_logger().info(f'Distancia: {self.msg_distance.data} Orientacion: {self.msg_theta.data} Vel lineal: {self.msg_linear.data} Vel angular: {self.msg_angular.data} Pos x: {self.msg_x.data} Pos y: {self.msg_y.data}')


def main(args = None):
    rclpy.init(args = args)
    m_p = My_Process()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
