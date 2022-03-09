import math
import rclpy  # Import the ROS client library for Python
from rclpy.node import Node  # Enables the use of rclpy's Node class

from geometry_msgs.msg import Twist
from std_msgs.msg import (
    Float64MultiArray,
)  # Enable use of the std_msgs/Float64MultiArray message type
from ackermann_msgs.msg import AckermannDriveStamped

# Twist Sub => Float64MultiArray Pub
class RacecarController(Node):
    def __init__(
        self,
        car_wheel_base,
        car_wheel_threat,
        max_abs_steer,
        wheel_radius,
        max_wheel_turn_speed,
    ):
        super().__init__("racecar_controller")

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        self.L = car_wheel_base
        self.T = car_wheel_threat
        self.wheel_radius = wheel_radius
        self._max_wheel_turn_speed = max_wheel_turn_speed

        # rad / sec
        self.max_steering_speed = 2.0
        self.acceptable_steer_error = 0.1

        # we want maxsteer to be that of the "inside" tire, and since it is 0.6 in gazebo, we
        # set our ideal steering angle max to be less than that, based on geometry
        self.maxsteerInside = max_abs_steer
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        # radius of inside tire is rMax, so radius of the ideal middle tire (R_MIN) is rMax+treadwidth/2
        R_Min_interior = self.L / math.tan(self.maxsteerInside)
        self.R_Min_baselink = R_Min_interior + (self.T / 2.0)
        self.get_logger().info(
            "################ MINIMUM TURNING RADIUS ACKERMAN==="
            + str(self.R_Min_baselink)
        )

        # TODO
        # self._check_cmd_vel_ready()

        # ROS 2 Parts
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        self.throttling_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )

        self.lastMsg = self.get_clock().now().to_msg()

        self.steering_msg = Float64MultiArray()
        self.throttling_msg = Float64MultiArray()

    # def _check_cmd_vel_ready(self):
    #     data = None
    #     while data is None and rclpy.ok():
    #         try:
    #             data = rclpy.wait_for_message('/cmd_vel', Twist, timeout=1.0)
    #             self.process_cmd_vel_data(data)
    #             rclpy.get_logger().info("Current cmd_vel READY=>")
    #         except:
    #             rclpy.get_logger().info("Current cmd_vel not ready yet, retrying...")

    def cmd_vel_callback(self, data):
        """
        We get the linear velocity and the desired Turning Angular velocity.
        We have to convert it to Turning Radius
        """
        self.process_cmd_vel_data(data)
        self.publish()

    def process_cmd_vel_data(self, data):
        self.linear_velocity = data.linear.x
        # We limit the minimum value of the Steering Radius
        # Todo Process negatives

        self.get_logger().info("self.linear_velocity=" + str(self.linear_velocity))
        self.get_logger().info("data.angular.z=" + str(data.angular.z))

        if data.angular.z != 0.0:
            steering_radius_raw = abs(self.linear_velocity / data.angular.z)
            self.get_logger().info("steering_radius_raw=" + str(steering_radius_raw))
            self.get_logger().info("R_Min_baselink=" + str(self.R_Min_baselink))

            self.steering_radius = max(abs(steering_radius_raw), self.R_Min_baselink)
            # We consider that turning left should be positive
            self.turning_sign = -1 * math.copysign(1, data.angular.z)
            # Going Fowards is positive
            self.linear_sign = math.copysign(1, self.linear_velocity)
            self.omega_turning_speed = self.linear_velocity / self.steering_radius
        else:
            self.steering_radius = -1
            self.turning_sign = 0.0
            self.linear_sign = 0.0
            self.omega_turning_speed = 0.0

        self.lastMsg = self.get_clock().now().to_msg()

    def limit_wheel_speed(self, in_speed):

        if in_speed > self._max_wheel_turn_speed:
            self.get_logger().warn("MAX Wheel Speed!")
            in_speed = self._max_wheel_turn_speed
        elif in_speed < -1.0 * self._max_wheel_turn_speed:
            self.get_logger().warn("MAX Wheel Speed!")
            in_speed = -1.0 * self._max_wheel_turn_speed

        return in_speed

    def publish(self):

        # Step 1 Calculate the Wheel Turning Speed for the Rear Wheels
        # For that we have the following input data of the base_link
        # Filtered if it was impossible for the system to do it
        # self.turning_sign states which side system want to turn
        # and who is the exterior interior wheel ( default interior = right wheel. exterior = left wheel )
        vel_base_link = self.linear_velocity
        omega_base_link = self.omega_turning_speed
        turning_radius_base_link = self.steering_radius

        turning_radius_right_rear_wheel = None
        turning_radius_left_rear_wheel = None

        if self.steering_radius >= 0:
            # Default Interior = Right WHeel
            # Make this sign multiplication because when going backwards angular is inverted, so it ha sto invert the sign
            turning_radius_right_rear_wheel = turning_radius_base_link + (
                -1 * self.turning_sign * self.linear_sign
            ) * (self.T / 2.0)
            vel_right_rear_wheel = omega_base_link * turning_radius_right_rear_wheel
            wheel_turnig_speed_right_rear_wheel = self.limit_wheel_speed(
                vel_right_rear_wheel / self.wheel_radius
            )

            # Default Interior = Left WHeel
            # Make this sign multiplication because when going backwards angular is inverted, so it ha sto invert the sign
            turning_radius_left_rear_wheel = turning_radius_base_link + (
                1 * self.turning_sign * self.linear_sign
            ) * (self.T / 2.0)
            vel_left_rear_wheel = omega_base_link * turning_radius_left_rear_wheel
            wheel_turnig_speed_left_rear_wheel = self.limit_wheel_speed(
                vel_left_rear_wheel / self.wheel_radius
            )
        else:
            # Not turning , there fore they are all the same
            # Default Interior = Right WHeel
            wheel_turnig_speed_right_rear_wheel = self.limit_wheel_speed(
                vel_base_link / self.wheel_radius
            )
            # Default Interior = Left WHeel
            wheel_turnig_speed_left_rear_wheel = self.limit_wheel_speed(
                vel_base_link / self.wheel_radius
            )

        #### END REAR WHeel Calculations

        # Step 2: Calculate the Wheel Turning Speed for the Front wheels and the STeering angle
        if self.steering_radius >= 0:
            turning_radius_right_front_wheel = turning_radius_right_rear_wheel
            distance_to_turning_point_right_front_wheel = math.sqrt(
                pow(self.L, 2) + pow(turning_radius_right_front_wheel, 2)
            )
            vel_right_front_wheel = (
                omega_base_link * distance_to_turning_point_right_front_wheel
            )

            wheel_turnig_speed_right_front_wheel = self.limit_wheel_speed(
                vel_right_front_wheel / self.wheel_radius
            )
            alfa_right_front_wheel = math.atan(
                self.L / turning_radius_right_front_wheel
            )

            turning_radius_left_front_wheel = turning_radius_left_rear_wheel
            distance_to_turning_point_left_front_wheel = math.sqrt(
                pow(self.L, 2) + pow(turning_radius_left_front_wheel, 2)
            )
            vel_left_front_wheel = (
                omega_base_link * distance_to_turning_point_left_front_wheel
            )
            wheel_turnig_speed_left_front_wheel = self.limit_wheel_speed(
                vel_left_front_wheel / self.wheel_radius
            )
            alfa_left_front_wheel = math.atan(self.L / turning_radius_left_front_wheel)
        else:
            wheel_turnig_speed_right_front_wheel = self.limit_wheel_speed(
                vel_base_link / self.wheel_radius
            )
            alfa_right_front_wheel = 0.0

            wheel_turnig_speed_left_front_wheel = self.limit_wheel_speed(
                vel_base_link / self.wheel_radius
            )
            alfa_left_front_wheel = 0.0
        #### END FRONT WHeel Calculations

        # os.system('clear')
        print("#####################")
        print("@ INPUT VALUES @")
        print("vel_base_link=" + str(vel_base_link))
        print("omega_base_link=" + str(omega_base_link))
        print("turning_radius_base_link=" + str(turning_radius_base_link))
        print("@ TURNING SPEEDS @")
        print(
            "wheel_turnig_speed_right_rear_wheel="
            + str(wheel_turnig_speed_right_rear_wheel)
        )
        print(
            "wheel_turnig_speed_left_rear_wheel="
            + str(wheel_turnig_speed_left_rear_wheel)
        )
        print(
            "wheel_turnig_speed_right_front_wheel="
            + str(wheel_turnig_speed_right_front_wheel)
        )
        print(
            "wheel_turnig_speed_left_front_wheel="
            + str(wheel_turnig_speed_left_front_wheel)
        )
        print("@ ANGLES @")
        print("alfa_right_front_wheel=" + str(alfa_right_front_wheel))
        print("alfa_left_front_wheel=" + str(alfa_left_front_wheel))
        print("####### END #########")

        # Step 3 Publish all the data in the corresponding topics
        self.steering_msg.data = [
            -1 * self.turning_sign * self.linear_sign * alfa_right_front_wheel,
            -1 * self.turning_sign * self.linear_sign * alfa_left_front_wheel,
        ]
        self.throttling_msg.data = [
            wheel_turnig_speed_left_rear_wheel,
            wheel_turnig_speed_right_rear_wheel,
            wheel_turnig_speed_left_front_wheel,
            wheel_turnig_speed_right_front_wheel,
        ]

        self.steering_pub.publish(self.steering_msg)
        self.throttling_pub.publish(self.throttling_msg)


def main(args=None):

    rclpy.init(args=args)

    # Distance from Front to Rear axel
    car_wheel_base = 0.325

    # Distance from left to right wheels
    car_wheel_threat = 0.2

    # Calculated as the  maximum steering angle the inner wheel can do
    max_abs_steer = 0.7853

    # Wheel Radius (from urdf)
    wheel_radius = 0.05

    # Radians per second, that with the current wheel radius would make 44 Km/h max linear vel
    # 최대속도 30km/h 기준
    # 30 * 1000 / 3600 / 0.05 = 167
    max_wheel_turn_speed = 167

    racecar_controller = RacecarController(
        car_wheel_base,
        car_wheel_threat,
        max_abs_steer,
        wheel_radius,
        max_wheel_turn_speed,
    )

    rclpy.spin(racecar_controller)
    racecar_controller.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
