#!/usr/bin/env python3
"""
SLAM Controller for RB300

Subscribes to /rb300_webui/slam_command (std_msgs/String) and starts/stops
slam_toolbox async_slam_toolbox_node accordingly.
"""

import os
import subprocess
import signal
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class SlamController(Node):
    def __init__(self):
        super().__init__('slam_controller')

        self.declare_parameter('slam_params_file', '')
        self.declare_parameter('slam_executable', 'async_slam_toolbox_node')
        self.declare_parameter('slam_package', 'slam_toolbox')
        self.declare_parameter('command_topic', '/rb300_webui/slam_command')

        # Determine params file path
        params_override = self.get_parameter('slam_params_file').get_parameter_value().string_value
        if params_override:
            self.params_file = params_override
        else:
            self.params_file = get_package_share_directory('rb300_nav') + '/config/slam_toolbox.yaml'

        self.slam_executable = self.get_parameter('slam_executable').get_parameter_value().string_value
        self.slam_package = self.get_parameter('slam_package').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        self.get_logger().info(f'Parameters: {self.params_file}')
        self.get_logger().info(f'Subscribing to {command_topic}')

        self._process = None
        self._sub = self.create_subscription(String, command_topic, self._on_command, 10)

    def _on_command(self, msg: String):
        data = msg.data.strip().lower()
        self.get_logger().info(f'Received command: "{data}"')

        if data == 'start':
            self._start_slam()
        elif data == 'stop':
            self._stop_slam()
        else:
            self.get_logger().warn(f'Unknown command: "{data}". Use "start" or "stop".')

    def _start_slam(self):
        if self._process is not None and self._process.poll() is None:
            self.get_logger().info('SLAM already running. Ignoring start command.')
            return

        cmd = [
            'ros2', 'run', self.slam_package, self.slam_executable,
            '--ros-args', '--params-file', self.params_file
        ]
        self.get_logger().info(f'Starting SLAM: {" ".join(cmd)}')
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            self.get_logger().info(f'SLAM started (PID: {self._process.pid})')
        except Exception as e:
            self.get_logger().error(f'Failed to start SLAM: {e}')
            self._process = None

    def _stop_slam(self):
        if self._process is None:
            self.get_logger().info('SLAM not running. Ignoring stop command.')
            return

        pid = self._process.pid
        self.get_logger().info(f'Stopping SLAM (PID: {pid})...')
        try:
            # Kill the entire process group to ensure child processes are terminated
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            self.get_logger().warn(f'Process {pid} already exited.')
        except Exception as e:
            self.get_logger().error(f'Error stopping SLAM: {e}')
            try:
                self._process.terminate()
            except Exception:
                pass
        finally:
            self._process = None
            self.get_logger().info('SLAM stopped.')

    def destroy_node(self):
        self._stop_slam()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlamController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
