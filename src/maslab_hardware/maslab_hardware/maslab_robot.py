from typing import Sequence
from icm42688.icm42688 import busio
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from hardware_interfaces.msg import RobotVelocity


from raven import Raven
from icm42688 import ICM42688
import board


def diff_drive_ik(
    wheel_radius: float,
    track_width: float,
    translational_velocity: float,
    rotational_velocity: float,
) -> tuple[float, float]:
    avg_rot_vel = translational_velocity / wheel_radius
    chassis_rot_vel = rotational_velocity * track_width / wheel_radius / 2

    left_rot_vel = avg_rot_vel + chassis_rot_vel
    right_rot_vel = avg_rot_vel - chassis_rot_vel

    return (left_rot_vel, right_rot_vel)


def sequence_to_vector3(sequence: Sequence) -> Vector3:
    return Vector3(x=sequence[0], y=sequence[1], z=sequence[2])


class Hardware(Node):

    left_wheel = Raven.MotorChannel.CH2
    right_wheel = Raven.MotorChannel.CH1

    in_servo = Raven.ServoChannel.CH1

    def __init__(self):
        super().__init__("maslab_hardware")

        # set up node parameters
        self.track_width_param = self.declare_parameter("track_width", 0.0)
        self.wheel_radius_param = self.declare_parameter("wheel_radius", 0.0)
        self.p_param = self.declare_parameter("p_gain", 0.0)
        self.i_param = self.declare_parameter("i_gain", 0.0)
        self.d_param = self.declare_parameter("d_gain", 0.0)

        self.p_value = 0
        self.i_value = 0
        self.d_value = 0

        self.controller = Raven()

        # start from scratch
        self.controller.reset()

        # set intake servo to the left side
        #assuming 0 means we are the red side
        self.controller.set_servo_position(self.in_servo, 0)

        # configure motor control

        self.controller.set_motor_mode(self.left_wheel, Raven.MotorMode.VELOCITY, 5)
        self.controller.set_motor_pid(
            self.left_wheel, p_gain=0, i_gain=0, d_gain=0, retry=5
        )

        self.controller.set_motor_mode(self.right_wheel, Raven.MotorMode.VELOCITY, 5)
        self.controller.set_motor_pid(
            self.right_wheel, p_gain=0, i_gain=0, d_gain=0, retry=5
        )

        # configure imu
        self.spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        while not self.spi.try_lock():
            pass

        self.spi.configure(baudrate=5000000)

        self.imu = ICM42688(self.spi)
        self.imu.begin()

        # subscribe to motor control topics
        self.commanded_twist = self.create_subscription(
            RobotVelocity, "drive_twist", self.drive_callback, 10
        )

        #We would then need a system to call this when
        #the blocks are being approached 
        #the blocks are in the middle
        #not sure when we'd know the blocks are in the middle
        self.servo_turn = self.create_subscription(
            Bool, "servo_rot", self.move_servo_to, 10
        )


        # publish imu data
        self.imu_trn_pub = self.create_publisher(Vector3, "linear_accel", 10)
        self.imu_rot_pub = self.create_publisher(Vector3, "angular_velocity", 10)

        # publish encoder data
        self.right_distance_pub = self.create_publisher(Float32, "right_distance", 10)
        self.left_distance_pub = self.create_publisher(Float32, "left_distance", 10)

        self.sensor_timer = self.create_timer(
            timer_period_sec=0.02, callback=self.update_sensor_callback
        )
        self.slow_timer = self.create_timer(
            timer_period_sec=0.5, callback=self.slow_callback
        )

    #msg = False, move to red
    #msg = True, move to green
    def move_servo_to(self, msg: Bool) -> None:
        if msg is False:
            self.controller.set_servo_position(self.in_servo, 0)

        if msg is True:
            self.controller.set_servo_position(self.in_servo, 180)
        
        

    def drive_callback(self, msg: RobotVelocity) -> None:
        left_vel, right_vel = diff_drive_ik(
            self.wheel_radius,
            self.track_width,
            msg.translational_velocity,
            msg.rotational_velocity,
        )

        self.get_logger().info(f"commanded left: {left_vel}, {right_vel}")

        left_success = self.controller.set_motor_target(self.left_wheel, left_vel, 5)
        right_success = self.controller.set_motor_target(self.right_wheel, right_vel, 5)

        if not left_success:
            self.get_logger().warning("setting left drive motor failed")

        if not right_success:
            self.get_logger().warning("setting right drive motor failed")

    def update_sensor_callback(self) -> None:
        angular_accel, rot_vel = self.imu.get_data()
        self.imu_trn_pub.publish(sequence_to_vector3(angular_accel))
        self.imu_rot_pub.publish(sequence_to_vector3(rot_vel))

        left_distance = self.controller.get_motor_encoder(self.left_wheel, 5)
        right_distance = self.controller.get_motor_encoder(self.right_wheel, 5)

        if left_distance is not None:
            self.left_distance_pub.publish(Float32(data=float(left_distance)))
        else:
            self.get_logger().error("could not get left motor distance")

        if right_distance is not None:
            self.right_distance_pub.publish(Float32(data=float(right_distance)))
        else:
            self.get_logger().error("could not get right motor distance")

    def slow_callback(self) -> None:
        self.track_width = self.track_width_param.get_parameter_value().double_value
        self.wheel_radius = self.track_width_param.get_parameter_value().double_value

        # update pid values
        p_value = self.p_param.get_parameter_value().double_value
        i_value = self.i_param.get_parameter_value().double_value
        d_value = self.d_param.get_parameter_value().double_value

        if not (
            self.p_value == p_value
            and self.i_value == i_value
            and self.d_value == d_value
        ):
            self.p_value = p_value
            self.i_value = i_value
            self.d_value = d_value
            self.controller.set_motor_pid(self.left_wheel, p_value, i_value, d_value, 5)
            self.controller.set_motor_pid(
                self.right_wheel, p_value, i_value, d_value, 5
            )


def main(args=None):
    rclpy.init(args=args)

    hardware = Hardware()

    rclpy.spin(hardware)

    hardware.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
