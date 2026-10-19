#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Helper node tự động gửi lệnh Undock tới TurtleBot 4
Giúp robot lùi an toàn khỏi trạm sạc khi khởi chạy simulation.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock

def main():
    rclpy.init(args=sys.argv + ['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('auto_undocker')
    client = ActionClient(node, Undock, '/undock')

    node.get_logger().info('Đang chờ action server /undock sẵn sàng...')
    if client.wait_for_server(timeout_sec=25.0):
        node.get_logger().info('Gửi yêu cầu Undock để robot rời khỏi trạm sạc...')
        goal_msg = Undock.Goal()
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            node.get_logger().info('Undock goal được chấp nhận. Đang lùi khỏi trạm sạc...')
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, res_future, timeout_sec=25.0)
            node.get_logger().info('[THÀNH CÔNG] Robot đã Undock an toàn khỏi dock!')
        else:
            node.get_logger().info('Robot không ở trên trạm sạc hoặc goal undock đã được xử lý.')
    else:
        node.get_logger().warn('Không tìm thấy server /undock trong thời gian chờ.')

    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
