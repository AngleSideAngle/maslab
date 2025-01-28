import time

from enum import Enum
from typing import Callable, Optional
import rclpy
from rclpy.node import Node

from hardware_interfaces.msg import DetectedCube
from hardware_interfaces.msg import RobotVelocity
from std_msgs.msg import Bool, String


type State = Callable[[], State]


class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        self.state: State = self.searching_state
        self.wall_extended_to_green = False

        self.red_cube: Optional[DetectedCube] = None
        self.create_subscription(DetectedCube, "red_cube", self.set_red_cube, 10)

        self.green_cube: Optional[DetectedCube] = None
        self.create_subscription(DetectedCube, "green_cube", self.set_green_cube, 10)

        self.drive_vel_pub = self.create_publisher(RobotVelocity, "drive_twist", 10)
        self.extend_wall_pub = self.create_publisher(Bool, "extend_wall", 10)
        self.extend_ball_catcher_pub = self.create_publisher(
            Bool, "extend_ball_catcher", 10
        )
        self.extend_green_lift_pub = self.create_publisher(
            Bool, "extend_green_lift", 10
        )
        self.current_state = self.create_publisher(String, "current_state", 10)

        self.create_timer(0.08, self.timer_callback)

    def timer_callback(self) -> None:
        self.state = self.state()
        self.current_state.publish(String(data=self.state.__name__))

    def set_red_cube(self, msg: DetectedCube) -> None:
        self.red_cube = msg

    def set_green_cube(self, msg: DetectedCube) -> None:
        self.green_cube = msg

    def take_red_cube(self) -> Optional[DetectedCube]:
        prev = self.red_cube
        self.red_cube = None
        return prev

    def take_green_cube(self) -> Optional[DetectedCube]:
        prev = self.green_cube
        self.green_cube = None
        return prev

    def create_perpetual_state(self, callback: Callable[[], State]) -> State:
        def state() -> State:
            callback()
            return callback

        return state

    def create_timed_state(
        self, duration: float, next_state: State
    ) -> Callable[[Callable[[], None]], State]:
        def create_timeout_state(callback: Callable[[]]) -> State:

            def timed_state(end_time: float):
                if time.time() > end_time:
                    return next_state

                callback()

                return timed_state(end_time)

            def start_timer():
                return timed_state(time.time() + duration)

            return start_timer

        return create_timeout_state

    def set_drive_vel(
        self, translational_velocity: float, rotational_velocity: float
    ) -> None:
        self.drive_vel_pub.publish(
            RobotVelocity(
                translational_velocity=translational_velocity,
                rotational_velocity=rotational_velocity,
            )
        )

    def set_wall_extension(self, extend: bool) -> None:
        self.wall_extended_to_green = extend
        self.extend_wall_pub.publish(Bool(data=extend))

    def set_ball_catcher_extension(self, extend: bool) -> None:
        self.extend_ball_catcher_pub.publish(Bool(data=extend))

    def set_green_lift_extension(self, extend: bool) -> None:
        self.extend_green_lift_pub.publish(Bool(data=extend))

    def set_red_lift_extension(self, extend: bool) -> None:
        pass

    def searching_state(self) -> State:
        red = self.take_red_cube()
        green = self.take_green_cube()

        if red is not None or green is not None:
            return self.following_state

        self.set_drive_vel(0.0, 0.4)

        return self.searching_state

    def following_state(self) -> State:
        red = self.take_red_cube()
        green = self.take_green_cube()

        if red is not None and green is not None:
            red_on_top = red.y > green.y
            self.set_wall_extension(red_on_top)
            center = (red.x + red.width / 2 + green.x + green.width / 2) / 2
        elif red is not None:
            center = red.x + red.width / 2
        elif green is not None:
            center = green.x + green.width / 2
        else:
            return self.intake_state_sequence

        self.set_drive_vel(0.5, center / 10)

        return self.following_state

    def either(
        self, cond: Callable[[], bool], true_state: State, false_state: State
    ) -> State:
        return true_state if cond() else false_state

    def intake_state_sequence(self) -> State:
        def lower_raise_red(next: State):

            @self.create_timed_state(0.5, next_state=next)
            def red_up():
                self.set_red_lift_extension(True)

            @self.create_timed_state(0.5, next_state=red_up)
            def red_down():
                self.set_red_lift_extension(False)

            return red_down

        def lower_raise_green(next: State):

            @self.create_timed_state(0.5, next_state=next)
            def green_up():
                self.set_green_lift_extension(True)

            @self.create_timed_state(0.5, next_state=green_up)
            def green_down():
                self.set_green_lift_extension(False)

            return green_down

        def wall_switch(next: State):

            @self.create_timed_state(0.5, next_state=next)
            def state():
                self.set_wall_extension(not self.wall_extended_to_green)

            return state

        red_first = lower_raise_red(
            wall_switch(lower_raise_green(self.searching_state()))
        )
        green_first = lower_raise_green(
            wall_switch(lower_raise_red(self.searching_state()))
        )

        @self.create_timed_state(
            1.5,
            next_state=self.either(
                lambda: self.wall_extended_to_green, red_first, green_first
            ),
        )
        def forward():
            self.set_drive_vel(0.5, 0)

        return forward

    def disabled(self) -> State:
        return self.disabled


def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
