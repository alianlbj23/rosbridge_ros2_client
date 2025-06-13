#!/usr/bin/env python3
import argparse
import os
import time

import rclpy
from rclpy.node import Node

import roslibpy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as BuiltinTime

def wait_for_connection(client, timeout=5.0, check_interval=0.1):
    """
    等待 roslibpy client 連線成功或超時
    """
    start = time.time()
    while not getattr(client, 'connected', False) and (time.time() - start) < timeout:
        time.sleep(check_interval)
    return getattr(client, 'connected', False)

class RosbridgeRelay(Node):
    def __init__(self, host: str, port: int):
        super().__init__('rosbridge_relay')

        # 1. 建立並啟動 roslibpy client
        self._roslib = roslibpy.Ros(host=host, port=port)
        self._roslib.run()
        if wait_for_connection(self._roslib):
            self.get_logger().info(f'✅ Connected to rosbridge at {host}:{port}')
        else:
            self.get_logger().error(f'❌ Failed to connect to rosbridge at {host}:{port}')

        # 2. 定义要转发的 topics:
        #    key = topic name
        #    value = (rosbridge_type, ros2_type)
        topics = {
            '/joint_states':        ('sensor_msgs/JointState',         'sensor_msgs/msg/JointState'),
            '/move_group/status':   ('actionlib_msgs/GoalStatusArray','actionlib_msgs/msg/GoalStatusArray'),
        }

        # 动态加载 GoalStatus，用于 status_list 里的 submessage
        self._status_cls = get_message('actionlib_msgs/msg/GoalStatus')

        # 3. 为每个 topic 建 publisher & subscription
        self._pubs = {}
        for topic, (rb_type, ros2_type) in topics.items():
            # 3.1 动态加载 ROS2 message class
            MsgClass = get_message(ros2_type)
            pub = self.create_publisher(MsgClass, topic, 10)
            self._pubs[topic] = pub

            # 3.2 在 rosbridge 上 subscribe
            rt = roslibpy.Topic(self._roslib, topic, rb_type)
            rt.subscribe(self._make_cb(topic, MsgClass))
            self.get_logger().info(f'Subscribed → {topic} as {rb_type}')

    def _make_cb(self, topic, MsgClass):
        is_status = (topic == '/move_group/status')
        def callback(msg):
            data = dict(msg)

            # 处理 header（JointState 也有 header）
            if 'header' in data and isinstance(data['header'], dict):
                h = data['header']
                hdr = Header()
                stamp = h.get('stamp', {})
                hdr.stamp = BuiltinTime(sec=stamp.get('secs',0), nanosec=stamp.get('nsecs',0))
                hdr.frame_id = h.get('frame_id','')
                data['header'] = hdr

            # 处理 status_list
            if is_status and 'status_list' in data:
                lst = []
                for s in data['status_list']:
                    gs = self._status_cls()
                    gid = s.get('goal_id', {})
                    gs.goal_id.id = gid.get('id','')
                    st = gid.get('stamp', {})
                    gs.goal_id.stamp = BuiltinTime(sec=st.get('secs',0), nanosec=st.get('nsecs',0))
                    gs.status = s.get('status',0)
                    gs.text = s.get('text','')
                    lst.append(gs)
                data['status_list'] = lst

            try:
                m = MsgClass(**data)
                self._pubs[topic].publish(m)
            except Exception as e:
                self.get_logger().error(f'Failed to relay {topic}: {e}')
        return callback

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host',
        default=os.getenv('ROSBRIDGE_HOST','host.docker.internal'),
        help='rosbridge host')
    parser.add_argument('--port', type=int,
        default=int(os.getenv('ROSBRIDGE_PORT','9090')),
        help='rosbridge port')
    args = parser.parse_args()

    rclpy.init()
    node = RosbridgeRelay(args.host, args.port)
    try:
        rclpy.spin(node)
    finally:
        node._roslib.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
