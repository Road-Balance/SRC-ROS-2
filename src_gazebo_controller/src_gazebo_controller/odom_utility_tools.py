import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from std_srvs.srv import Trigger
from gazebo_msgs.srv import SetEntityState, GetEntityState

import numpy as np


# truth_pub=rospy.Publisher('/ground_truth_x', Float64, queue_size=1)
# noisy_odom_pub = rospy.Publisher('/noisy_odom_x', Float64, queue_size=1)

# get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
# teleport_service = rospy.ServiceProxy('/reset_model_pose', Trigger)

"""
OdomUtilNode에는 두 Class가 포함되어 있음
- TeleportClient
    reset_model_pose service call을 통해 로봇 위치 재설정 
    teleport_service.py 참고
- EntityStateClient
    Gazebo에서 neuronbot2의 x 위치 즉, pose.position.x를 주기적으로 받는다.
    이것이 ground_truth_x가 됨
    ground_truth_x > 10 이면 로봇 위치 재설정
x 위치 변화량에 따라 noisy_odom을 만든 뒤 noisy_odom_x topic publish를 한다.
"""

class EntityStateClient(Node):

    def __init__(self, model_name, verbose):
        super().__init__('reset_model_client')

        self._verbose = verbose
        self._entity_state_client = self.create_client(
            GetEntityState, 'get_entity_state'
        )

        while not self._entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' [gazebo/get_entity_state] service not available, waiting again...')

        self._entity_state_req = GetEntityState.Request()
        self._entity_state_req.name = model_name

        self.get_logger().info('==== Entity State Service Client Ready ====')

    def send_request(self):
        
        future = self._entity_state_client.call_async(self._entity_state_req)

        if self._verbose:
            self.get_logger().info('=== Request Sended ===')
        
        return future

class OdomUtilNode(Node):

    def __init__(self):
        super().__init__('odometry_util_node')

        self.declare_parameter('model_name', 'racecar')
        self.declare_parameter('alpha', 0.4)
        self.declare_parameter('verbose', True)

        self._model_name = self.get_parameter('model_name').value
        self._alpha = self.get_parameter('alpha').value
        self._verbose = self.get_parameter('verbose').value

        self._entity_state_client = EntityStateClient(self._model_name, self._verbose)

        self._ground_truth_publisher_x = self.create_publisher(
            Float64, 'ground_truth_x', 1
        )

        self._ground_truth_publisher_y = self.create_publisher(
            Float64, 'ground_truth_y', 1
        )

        self._noisy_odom_publisher = self.create_publisher(
            Float64, 'noisy_odom_x', 1
        )

        self.gt_x = Float64()
        self.gt_y = Float64()

        self.gt_list = [1.0]

        self._timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):

        state_client_future = self._entity_state_client.send_request()
        rclpy.spin_until_future_complete(self._entity_state_client, state_client_future)

        if state_client_future.done():
            try:
                state_response = state_client_future.result()
            except Exception:
                raise RuntimeError(
                    'exception while calling entity state service: %r' % state_client_future.exception()
                )
            else:
                ground_truth_x = state_response.state.pose.position.x
                ground_truth_y = state_response.state.pose.position.y
            finally:
                if self._verbose:
                    self.get_logger().warn('==== Entity Client Execution Done ====')

        if self._verbose:
            self.get_logger().info(f"Got ground truth pose.position.x: {ground_truth_x}" )

        self.gt_x.data = ground_truth_x
        self.gt_y.data = ground_truth_y
        
        max_gt = max(self.gt_list)
        min_gt = min(self.gt_list)

        if (self.gt_x.data < min_gt) | (self.gt_x.data > max_gt):
            self.gt_list.append(self.gt_x.data)

        max_gt = max(self.gt_list)
        min_gt = min(self.gt_list)

        print(max_gt, min_gt, max_gt - min_gt)

        self._ground_truth_publisher_x.publish(self.gt_x)
        self._ground_truth_publisher_y.publish(self.gt_y)

def main(args=None):
    rclpy.init(args=args)

    odom_util_node = OdomUtilNode()

    rclpy.spin(odom_util_node)

    odom_util_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()