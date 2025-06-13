#!/usr/bin/env python3
import argparse
import os
import time

import rclpy
from rclpy.node import Node

import roslibpy


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
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()

        # 等待連線結果
        if wait_for_connection(self.client):
            self.get_logger().info(f'✅ Connected to rosbridge at {host}:{port}')
        else:
            self.get_logger().error(f'❌ Failed to connect to rosbridge at {host}:{port}')

        # 目前什麼都不做，之後再加功能
        # e.g. 可以在這裡保留 client, 再做 service call、subscribe 等


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--host',
        default=os.getenv('ROSBRIDGE_HOST', 'host.docker.internal'),
        help='rosbridge host (預設: host.docker.internal 或 $ROSBRIDGE_HOST)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=int(os.getenv('ROSBRIDGE_PORT', '9090')),
        help='rosbridge port (預設: 9090 或 $ROSBRIDGE_PORT)'
    )
    args = parser.parse_args()

    rclpy.init()
    node = RosbridgeRelay(args.host, args.port)
    try:
        rclpy.spin(node)
    finally:
        node.client.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
