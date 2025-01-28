from typing import Callable, Sequence
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from hardware_interfaces.msg import RobotVelocity


from raven import Raven
from icm42688 import ICM42688
from icm42688.icm42688 import busio
import board

MAX_MOTOR_CURRENT_AMPS = 6.0


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

    LEFT_WHEEL = Raven.MotorChannel.CH1
    RIGHT_WHEEL = Raven.MotorChannel.CH2

    WALL_SERVO = Raven.ServoChannel.CH2
    BALL_FLICK_SERVO = Raven.ServoChannel.CH1
    GREEN_RAISE_SERVO = Raven.ServoChannel.CH4
    RED_LAUNCH_DC = Raven.ServoChannel.CH3

    RED_WALL = 90
    GREEN_WALL = -50

    GREEN_BOTTOM = -90
    GREEN_TOP = 90

    def __init__(self):
        super().__init__("maslab_hardware")

        # set up node parameters
        self.track_width_param = self.declare_parameter("track_width", 0.0)
        self.wheel_radius_param = self.declare_parameter("wheel_radius", 0.0)
        self.p_param = self.declare_parameter("p_gain", 0.0)
        self.i_param = self.declare_parameter("i_gain", 0.0)
        self.d_param = self.declare_parameter("d_gain", 0.0)
        self.cpr_param = self.declare_parameter("cpr", 1.0)

        self.track_width = 1
        self.wheel_radius = 1
        self.p_value = 0.0
        self.i_value = 0.0
        self.d_value = 0.0
        self.cpr_value = 1.0

        self.controller = Raven()

        # start from scratch
        self.controller.reset()

        # set intake servo to the left side
        # assuming 0 means we are the red side
        self.controller.set_servo_position(self.WALL_SERVO, 0)

        # configure motor control

        self.controller.set_motor_mode(self.LEFT_WHEEL, Raven.MotorMode.VELOCITY, 5)
        self.controller.set_motor_pid(
            self.LEFT_WHEEL, p_gain=0, i_gain=0, d_gain=0, retry=5
        )
        self.controller.set_motor_max_current(
            self.LEFT_WHEEL, MAX_MOTOR_CURRENT_AMPS, 5
        )

        self.controller.set_motor_mode(self.RIGHT_WHEEL, Raven.MotorMode.VELOCITY, 5)
        self.controller.set_motor_pid(
            self.RIGHT_WHEEL, p_gain=0, i_gain=0, d_gain=0, retry=5
        )
        self.controller.set_motor_max_current(
            self.RIGHT_WHEEL, MAX_MOTOR_CURRENT_AMPS, 5
        )

        # configure imu
        self.spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        while not self.spi.try_lock():
            pass

        self.spi.configure(baudrate=5000000)

        self.imu = ICM42688(self.spi)
        self.imu.begin()

        # subscribe to motor control topics
        self.create_subscription(RobotVelocity, "drive_twist", self.drive_callback, 10)

        # subscribe to servo commands
        self.create_subscription(
            Bool,
            "extend_wall",
            self.create_servo_callback(self.WALL_SERVO, self.RED_WALL, self.GREEN_WALL),
            10,
        )
        self.create_subscription(
            Bool,
            "extend_ball_catcher",
            self.create_servo_callback(self.BALL_FLICK_SERVO, 0, 0),
            10,
        )
        self.create_subscription(
            Bool,
            "extend_green_lift",
            self.create_servo_callback(
                self.GREEN_RAISE_SERVO, self.GREEN_BOTTOM, self.GREEN_TOP
            ),
            10,
        )

        # publish imu data
        self.imu_trn_pub = self.create_publisher(Vector3, "linear_accel", 10)
        self.imu_rot_pub = self.create_publisher(Vector3, "angular_velocity", 10)

        # publish encoder data
        self.right_distance_pub = self.create_publisher(Float32, "right_distance", 10)
        self.left_distance_pub = self.create_publisher(Float32, "left_distance", 10)
        self.right_vel_pub = self.create_publisher(Float32, "right_velocity", 10)
        self.left_vel_pub = self.create_publisher(Float32, "left_velocity", 10)

        self.sensor_timer = self.create_timer(
            timer_period_sec=0.02, callback=self.update_sensor_callback
        )
        self.slow_timer = self.create_timer(
            timer_period_sec=0.5, callback=self.slow_callback
        )

    def create_servo_callback(
        self, channel: Raven.ServoChannel, false_deg: float, true_deg: float
    ) -> Callable[[Bool], None]:
        def fn(msg: Bool) -> None:
            self.controller.set_servo_position(
                channel, true_deg if msg.data else false_deg
            )

        return fn

    def drive_callback(self, msg: RobotVelocity) -> None:
        left_vel, right_vel = diff_drive_ik(
            self.wheel_radius,
            self.track_width,
            msg.translational_velocity,
            msg.rotational_velocity,
        )

        self.get_logger().info(f"commanded velocities: {left_vel} : {right_vel}")

        # NOTE: THE LEFT WHEEL IS PHYSICALLY INVERTED. TO SAVE TIME IT IS INVERTED IN SOFTWARE
        left_success = self.controller.set_motor_target(
            self.LEFT_WHEEL, -left_vel * self.cpr_value, 5
        )
        right_success = self.controller.set_motor_target(
            self.RIGHT_WHEEL, right_vel * self.cpr_value, 5
        )

        if not left_success:
            self.get_logger().warning("setting left drive motor failed")

        if not right_success:
            self.get_logger().warning("setting right drive motor failed")

    def update_sensor_callback(self) -> None:
        angular_accel, rot_vel = self.imu.get_data()
        self.imu_trn_pub.publish(sequence_to_vector3(angular_accel))
        self.imu_rot_pub.publish(sequence_to_vector3(rot_vel))

        left_distance = self.controller.get_motor_encoder(self.LEFT_WHEEL, 5)
        right_distance = self.controller.get_motor_encoder(self.RIGHT_WHEEL, 5)

        left_velocity = self.controller.get_motor_velocity(self.LEFT_WHEEL, 5)
        right_velocity = self.controller.get_motor_velocity(self.RIGHT_WHEEL, 5)

        if left_distance is not None:
            self.left_distance_pub.publish(Float32(data=float(left_distance)))
        else:
            self.get_logger().error("could not get left motor distance")

        if right_distance is not None:
            self.right_distance_pub.publish(Float32(data=float(right_distance)))
        else:
            self.get_logger().error("could not get right motor distance")

        if left_velocity is not None:
            self.left_vel_pub.publish(Float32(data=float(left_velocity)))
        else:
            self.get_logger().error("could not get left motor velocity")

        if right_velocity is not None:
            self.right_vel_pub.publish(Float32(data=float(right_velocity)))
        else:
            self.get_logger().error("could not get right motor velocity")

    def slow_callback(self) -> None:
        self.track_width = self.track_width_param.get_parameter_value().double_value
        self.wheel_radius = self.track_width_param.get_parameter_value().double_value

        # update pid values
        p_value = self.p_param.get_parameter_value().double_value
        i_value = self.i_param.get_parameter_value().double_value
        d_value = self.d_param.get_parameter_value().double_value
        cpr_value = self.cpr_param.get_parameter_value().double_value

        if not (
            self.p_value == p_value
            and self.i_value == i_value
            and self.d_value == d_value
            and self.cpr_value == cpr_value
        ):
            self.p_value = p_value
            self.i_value = i_value
            self.d_value = d_value
            self.cpr_value = cpr_value
            self.controller.set_motor_pid(
                self.LEFT_WHEEL,
                p_value,
                i_value,
                d_value,
                percent=100,
                retry=5,
            )
            self.controller.set_motor_pid(
                self.RIGHT_WHEEL,
                p_value,
                i_value,
                d_value,
                percent=100,
                retry=5,
            )

            self.get_logger().warn("pid values changed, encoders reset")


def main(args=None):
    rclpy.init(args=args)

    hardware = Hardware()

    rclpy.spin(hardware)

    hardware.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
