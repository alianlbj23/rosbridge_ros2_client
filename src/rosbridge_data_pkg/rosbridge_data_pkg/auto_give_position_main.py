# auto_give_position_main.py
import roslibpy
import time


class IKPoseSenderNode:
    def __init__(self, host='localhost', port=9090):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()

        if not self.ros.is_connected:
            raise RuntimeError('❌ Failed to connect to ROS bridge.')

        print('✅ Connected to ROS bridge.')

        # 設定 publishers
        self.pose_pub = roslibpy.Topic(self.ros, '/target_pose', 'geometry_msgs/PoseStamped')
        self.trigger_pub = roslibpy.Topic(self.ros, '/execute_plan_signal', 'std_msgs/Empty')

    def send_pose_sequence(self, poses):
        for i, pos in enumerate(poses):
            msg = {
                'header': {'frame_id': 'base_link'},
                'pose': {
                    'position': pos,
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
            self.pose_pub.publish(roslibpy.Message(msg))
            print(f'📤 Sent pose {i+1}: {pos}')
            time.sleep(0.2)

    def send_trigger(self):
        self.trigger_pub.publish(roslibpy.Message({}))
        print('✅ Trigger sent to execute trajectory.')

    def shutdown(self):
        self.pose_pub.unadvertise()
        self.trigger_pub.unadvertise()
        self.ros.terminate()
        print('🔌 Disconnected from ROS bridge.')


def main():
    poses = [
        {'x': 0.3, 'y': 0.1, 'z': 0.4},
        {'x': 0.4, 'y': -0.2, 'z': 0.35},
        {'x': 0.5, 'y': 0.0, 'z': 0.3}
    ]

    try:
        node = IKPoseSenderNode(host='172.27.188.157', port=9090)
        node.send_pose_sequence(poses)
        node.send_trigger()
    finally:
        node.shutdown()
