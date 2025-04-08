import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class QubeController(Node):
    def __init__(self):
        super().__init__('qube_controller')
        
        # PID-parametre
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 1.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.target_position = 0.0  # Ønsket posisjon (radianer)

        # Abonner på joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        
        # Publiserer for hastighetskommandoer
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controller/commands', 
            10)
        
        self.get_logger().info('Qube Controller Node startet')

    def joint_states_callback(self, msg):
        try:
            # Finn indeksen for rotating_disk-leddet (motor_joint)
            index = msg.name.index('motor_joint')
            current_pos = msg.position[index]
            current_vel = msg.velocity[index]

            # Beregn PID
            error = self.target_position - current_pos
            self.integral += error * 0.01  # Antar 100Hz oppdatering
            derivative = (error - self.prev_error) / 0.01
            self.prev_error = error

            # Beregn hastighetskommando
            velocity_command = (
                self.kp * error + 
                self.ki * self.integral + 
                self.kd * derivative
            )

            # Opprett og publiser melding
            command_msg = Float64MultiArray()
            command_msg.data = [velocity_command]
            self.publisher.publish(command_msg)

        # Feilmelding til det Adam skrev om feil navn på disk_joint, se github hans
        except ValueError:
            self.get_logger().warn('disk_joint ikke funnet i joint_states')

def main(args=None):
    rclpy.init(args=args)
    controller = QubeController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
