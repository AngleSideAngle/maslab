from typing import Sequence
from icm42688.icm42688 import busio
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from hardware_interfaces.msg import RobotVelocity, PID


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

    left_wheel = Raven.MotorChannel.CH1
    right_wheel = Raven.MotorChannel.CH2

    def __init__(self):
        super().__init__("maslab_hardware")

        # set up node parameters
        self.track_width_param = self.declare_parameter("track_width", 0)
        self.wheel_radius_param = self.declare_parameter("wheel_radius", 0)
        self.p_param = self.declare_parameter("p_gain", 0)
        self.i_param = self.declare_parameter("i_gain", 0)
        self.d_param = self.declare_parameter("d_gain", 0)

        self.p_value = 0
        self.i_value = 0
        self.d_value = 0

        self.controller = Raven()

        # start from scratch
        self.controller.reset()

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

        # publish imu data
        self.imu_trn_pub = self.create_publisher(Vector3, "linear_accel", 10)
        self.imu_rot_pub = self.create_publisher(Vector3, "angular_velocity", 10)

        # publish encoder data
        self.right_distance_pub = self.create_publisher(float, "right_distance", 10)
        self.left_distance_pub = self.create_publisher(float, "left_distance", 10)

        self.sensor_timer = self.create_timer(
            timer_period_sec=0.02, callback=self.update_sensor_callback
        )
        self.slow_timer = self.create_timer(
            timer_period_sec=0.5, callback=self.slow_callback
        )

    def drive_callback(self, msg: RobotVelocity) -> None:
        left_vel, right_vel = diff_drive_ik(
            self.wheel_radius,
            self.track_width,
            msg.translational_velocity,
            msg.rotational_velocity,
        )

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

        self.left_distance_pub.publish(left_distance)
        self.right_distance_pub.publish(right_distance)

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
