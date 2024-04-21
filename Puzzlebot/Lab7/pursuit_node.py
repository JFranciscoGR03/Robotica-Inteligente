import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class PurePursuitControllerNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Suscripción a tópicos de x, y, theta
        self.sub_posX = self.create_subscription(Float32, 'positionX', self.callback_positionX, rclpy.qos.qos_profile_sensor_data)
        self.sub_posY = self.create_subscription(Float32, 'positionY', self.callback_positionY, rclpy.qos.qos_profile_sensor_data)
        self.sub_orientation = self.create_subscription(Float32, 'orientation', self.callback_orientation, rclpy.qos.qos_profile_sensor_data)

        # Publicación de velocidades lineales y angulares
        self.velPublisher = self.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_sensor_data)

        # Trayectoria deseada
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypointsX', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('waypointsY', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ]
        )

        #Parámetros de trayectoria deseada
        self.waypointsX = self.get_parameter('waypointsX').get_parameter_value().double_array_value
        self.waypointsY = self.get_parameter('waypointsY').get_parameter_value().double_array_value
        self.pathSize = len(self.waypointsX)
        self.path = []

        for i in range(self.pathSize):
            self.path.append([self.waypointsX[i], self.waypointsY[i]])

        # Parámetros del controlador
        self.lookahead_distance = 0.35
        self.vehicle_wheelbase = 0.18
        self.x = 0
        self.y = 0
        self.theta = 0

        # Función con timer para publicar
        self.timer_period = 0.015
        self.timer = self.create_timer(self.timer_period, self.odometry_callback)
        self.get_logger().info('Process node successfully initialized!')

        self.twist_msg = Twist()
        self.last_index = 0

    def callback_positionX(self, msg):
        self.x = msg.data

    def callback_positionY(self, msg):
        self.y = msg.data

    def callback_orientation(self, msg):
        self.theta = msg.data

    def find_target_index(self, pose):
        # Encuentra el punto en la trayectoria lo suficientemente adelante
        lookahead_index = self.last_index
        while lookahead_index < len(self.path) - 1:
            if np.sqrt((pose[0] - self.path[lookahead_index][0])**2 + (pose[1] - self.path[lookahead_index][1])**2) > self.lookahead_distance:
                break
            lookahead_index += 1

        self.last_index = lookahead_index

        return lookahead_index

    def pure_pursuit_control(self, pose):
        # Encuentra el índice del punto objetivo en la trayectoria
        target_index = self.find_target_index(pose)

        # Coordenadas del punto objetivo
        target_point = self.path[target_index]

        # Calcula el ángulo de dirección
        alpha = np.arctan2(target_point[1] - pose[1], target_point[0] - pose[0]) - pose[2]
        delta = np.arctan2(2.0 * self.vehicle_wheelbase * np.sin(alpha) / self.lookahead_distance, 1.0)

        return delta

    def odometry_callback(self):
        # Calcular el comando de dirección utilizando el algoritmo Pure Pursuit
        delta = self.pure_pursuit_control([self.x, self.y, self.theta])

        # Dirección
        self.twist_msg.angular.z = delta

        # Distancia al punto final
        distance_to_goal = np.sqrt((self.path[-1][0] - self.x)**2 + (self.path[-1][1] - self.y)**2)

        # Detener el robot si está cerca del punto final
        if (distance_to_goal <= 0.05) and (self.last_index == len(self.path) - 1):
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
        else:
            # Velocidad lineal
            self.twist_msg.linear.x = 0.08

        self.velPublisher.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
