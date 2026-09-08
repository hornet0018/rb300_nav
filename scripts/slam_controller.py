#!/usr/bin/env python3
"""
SLAM Controller for RB300

Subscribes to /rb300_webui/slam_command (std_msgs/String) and starts/stops
the scan matcher (slam_toolbox or Cartographer) accordingly.

Also handles map management commands from the web UI via
/rb300_webui/map_command (std_msgs/String, JSON payload):
  {"action": "save",  "name": "room1"}     save pbstream (+ PGM from /map)
  {"action": "list"}                       publish saved maps
  {"action": "select", "name": "room1"}    start Cartographer localization
  {"action": "stop"}                       stop Cartographer localization

Results are published to /rb300_webui/map_status (std_msgs/String, JSON).
"""

import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nav_msgs.msg import OccupancyGrid
    HAS_NAV_MSGS = True
except Exception:  # pragma: no cover
    OccupancyGrid = None
    HAS_NAV_MSGS = False

try:
    from cartographer_ros_msgs.srv import WriteState
    HAS_WRITE_STATE = True
except Exception:  # pragma: no cover
    WriteState = None
    HAS_WRITE_STATE = False

_SAFE_NAME_RE = re.compile(r'[^\w\-]+')


def _sanitize_map_name(name):
    name = _SAFE_NAME_RE.sub('_', (name or '').strip())
    return name.strip('_')[:100]


def _quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class SlamController(Node):
    def __init__(self):
        super().__init__('slam_controller')

        self.declare_parameter('slam_params_file', '')
        self.declare_parameter('slam_executable', 'async_slam_toolbox_node')
        self.declare_parameter('slam_package', 'slam_toolbox')
        self.declare_parameter('slam_mode', 'slam_toolbox')  # slam_toolbox | cartographer
        self.declare_parameter('command_topic', '/rb300_webui/slam_command')
        self.declare_parameter('cartographer_launch_cmd',
                               ['ros2', 'launch', 'rb300_webui', 'cartographer_mapping_launch.py'])
        self.declare_parameter('map_command_topic', '/rb300_webui/map_command')
        self.declare_parameter('map_status_topic', '/rb300_webui/map_status')
        self.declare_parameter('map_dir', os.path.expanduser('~/.rb300/maps'))
        self.declare_parameter('write_state_service', '/write_state')
        self.declare_parameter('localization_launch_cmd',
                               ['ros2', 'launch', 'rb300_webui', 'localization_launch.py'])

        # Determine params file path
        params_override = self.get_parameter('slam_params_file').get_parameter_value().string_value
        if params_override:
            self.params_file = params_override
        else:
            try:
                from ament_index_python.packages import get_package_share_directory
                self.params_file = get_package_share_directory('rb300_nav') + '/config/slam_toolbox.yaml'
            except Exception:
                self.params_file = ''

        self.slam_executable = self.get_parameter('slam_executable').get_parameter_value().string_value
        self.slam_package = self.get_parameter('slam_package').get_parameter_value().string_value
        self.slam_mode = self.get_parameter('slam_mode').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        map_command_topic = self.get_parameter('map_command_topic').get_parameter_value().string_value
        map_status_topic = self.get_parameter('map_status_topic').get_parameter_value().string_value

        self.map_dir = os.path.abspath(self.get_parameter('map_dir').value)
        try:
            os.makedirs(self.map_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().warn(f'Cannot create map_dir {self.map_dir}: {e}')

        self.get_logger().info(f'Parameters: {self.params_file}')
        self.get_logger().info(f'Subscribing to {command_topic}')
        self.get_logger().info(f'SLAM mode: {self.slam_mode}, map_dir: {self.map_dir}')

        self._process = None
        self._sub = self.create_subscription(String, command_topic, self._on_command, 10)
        self._map_cmd_sub = self.create_subscription(String, map_command_topic, self._on_map_command, 10)
        self._map_status_pub = self.create_publisher(String, map_status_topic, 10)

        # Cartographer write_state client (guarded)
        self._write_state_client = None
        if HAS_WRITE_STATE:
            self._write_state_client = self.create_client(
                WriteState, self.get_parameter('write_state_service').value
            )

        # Latest /map for PGM generation
        self._latest_map = None
        if HAS_NAV_MSGS:
            self._map_sub = self.create_subscription(
                OccupancyGrid, '/map', self._on_map, 10
            )

        # Localization (Cartographer) process management
        self._localization_proc = None
        self._current_map_name = None
        self._proc_lock = threading.Lock()

    # ------------------------------------------------------------ commands

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

        if self.slam_mode == 'cartographer':
            cmd = list(self.get_parameter('cartographer_launch_cmd').value)
            self.get_logger().info(f'Starting Cartographer mapping: {" ".join(cmd)}')
        else:
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

    # ------------------------------------------------------- map commands

    def _on_map_command(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self._publish_status(ok=False, message=f'map_command の JSON が不正です: {e}')
            return

        action = data.get('action')
        self.get_logger().info(f'Map command: {data}')

        if action == 'save':
            self._save_map(data.get('name', ''))
        elif action == 'list':
            self._publish_status(ok=True, message='')
        elif action == 'select':
            self._select_map(data.get('name', ''))
        elif action == 'stop':
            self._stop_localization()
        else:
            self._publish_status(ok=False, message=f'不明な map_command です: {action}')

    def _save_map(self, name):
        name = _sanitize_map_name(name) or datetime.now().strftime('map_%Y%m%d_%H%M%S')

        def finish(messages, ok):
            # /map から PGM + YAML (表示用)
            if self._write_thumbnail(name):
                messages.append('PGM/YAML を生成しました')
            else:
                messages.append('/map 未受信のため PGM/YAML を生成できませんでした')
            self._publish_status(ok=ok, message=' / '.join(messages), name=name)

        def on_write_done(future, name):
            messages = []
            ok = False
            try:
                future.result()
                messages.append('pbstream を保存しました')
                ok = True
            except Exception as e:
                messages.append(f'pbstream 保存失敗: {e}')
            finish(messages, ok)

        def run():
            if self._write_state_client is None:
                finish(['cartographer_ros_msgs が無いため pbstream 保存をスキップ'], False)
                return
            if not self._wait_service(self._write_state_client):
                finish(['write_state サービスが見つかりません (cartographer_node は起動していますか?)'], False)
                return
            req = WriteState.Request()
            req.filename = os.path.join(self.map_dir, name + '.pbstream')
            if hasattr(req, 'unfinished_trajectories_allowed'):
                req.unfinished_trajectories_allowed = True
            future = self._write_state_client.call_async(req)
            future.add_done_callback(lambda f: on_write_done(f, name))

        threading.Thread(target=run, daemon=True).start()

    def _select_map(self, name):
        name = _sanitize_map_name(name)
        if not name:
            self._publish_status(ok=False, message='地図名が不正です')
            return
        pbstream_path = self._map_path(name, '.pbstream')
        if not os.path.isfile(pbstream_path):
            self._publish_status(ok=False, message='地図が見つかりません (pbstream が必要です)')
            return

        def run():
            self._stop_localization()
            time.sleep(1.0)
            cmd = list(self.get_parameter('localization_launch_cmd').value) + [
                f'load_state_filename:={pbstream_path}'
            ]
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except Exception as e:
                self.get_logger().error(f'Failed to start localization: {e}')
                self._publish_status(ok=False, message=f'ローカライゼーション起動失敗: {e}')
                return

            time.sleep(4.0)
            if proc.poll() is not None:
                self._publish_status(ok=False,
                                     message='ローカライゼーションが起動に失敗しました '
                                             '(cartographer_ros がインストールされているか確認してください)')
                return

            with self._proc_lock:
                self._localization_proc = proc
                self._current_map_name = name
            self.get_logger().info(f'Cartographer localization started with map: {name}')
            self._publish_status(ok=True, name=name,
                                 message=f'地図「{name}」でローカライゼーションを開始しました')

        threading.Thread(target=run, daemon=True).start()

    def _stop_localization(self):
        with self._proc_lock:
            proc = self._localization_proc
            self._localization_proc = None
            self._current_map_name = None
        if proc is None or proc.poll() is not None:
            self._publish_status(ok=True, message='ローカライゼーションは起動していません')
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            return
        try:
            proc.wait(timeout=8.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5.0)
            except Exception:
                pass
        self.get_logger().info('Localization stopped.')
        self._publish_status(ok=True, message='ローカライゼーションを停止しました')

    # ------------------------------------------------------------- maps

    def _map_path(self, name, ext):
        return os.path.join(self.map_dir, f'{name}{ext}')

    def _on_map(self, msg):
        self._latest_map = msg

    def _list_maps(self):
        maps = []
        try:
            entries = sorted(os.listdir(self.map_dir))
        except OSError:
            return maps
        for fn in entries:
            if not fn.endswith('.pbstream'):
                continue
            name = fn[:-len('.pbstream')]
            info = {}
            info_path = self._map_path(name, '.info.json')
            if os.path.isfile(info_path):
                try:
                    with open(info_path, encoding='utf-8') as f:
                        info = json.load(f)
                except Exception:
                    info = {}
            maps.append({
                'name': name,
                'has_pbstream': True,
                'has_pgm': os.path.isfile(self._map_path(name, '.pgm')),
                'saved_at': info.get('saved_at'),
                'resolution': info.get('resolution'),
                'width': info.get('width'),
                'height': info.get('height'),
            })
        return maps

    def _write_thumbnail(self, name):
        """最新の /map (OccupancyGrid) から map_server 互換の PGM + YAML を生成"""
        grid = self._latest_map
        if grid is None or not grid.data:
            return False
        try:
            w = int(grid.info.width)
            h = int(grid.info.height)
            if w <= 0 or h <= 0 or len(grid.data) < w * h:
                return False

            # PGM: free=254 / occupied=0 / unknown=205 (map_server 規約)
            buf = bytearray()
            for v in grid.data:
                if v < 0:
                    buf.append(205)
                elif v == 0:
                    buf.append(254)
                else:
                    buf.append(max(0, 254 - int(round(v * 2.54))))

            with open(self._map_path(name, '.pgm'), 'wb') as f:
                f.write(b'P5\n%d %d\n255\n' % (w, h))
                f.write(bytes(buf))

            res = float(grid.info.resolution)
            yaw = _quat_to_yaw(grid.info.origin.orientation)
            origin = [
                grid.info.origin.position.x - res / 2.0,
                grid.info.origin.position.y - res / 2.0,
                yaw,
            ]
            yaml_text = (
                f'image: {name}.pgm\n'
                f'resolution: {res:.6f}\n'
                f'origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.196\n'
            )
            with open(self._map_path(name, '.yaml'), 'w', encoding='utf-8') as f:
                f.write(yaml_text)

            info = {
                'name': name,
                'saved_at': datetime.now().isoformat(timespec='seconds'),
                'width': w,
                'height': h,
                'resolution': res,
                'origin': origin,
            }
            with open(self._map_path(name, '.info.json'), 'w', encoding='utf-8') as f:
                json.dump(info, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().warn(f'Thumbnail write failed: {e}')
            return False

    # ------------------------------------------------------------ status

    def _publish_status(self, ok, message='', name=None):
        try:
            payload = {
                'ok': ok,
                'message': message,
                'name': name,
                'maps': self._list_maps(),
                'localization': {
                    'running': self._localization_proc is not None,
                    'map_name': self._current_map_name,
                },
            }
            self._map_status_pub.publish(String(data=json.dumps(payload)))
        except Exception as e:
            self.get_logger().error(f'Failed to publish map_status: {e}')

    @staticmethod
    def _wait_service(client, timeout=10.0):
        if client.service_is_ready():
            return True
        try:
            return client.wait_for_service(timeout_sec=timeout)
        except Exception:
            return False

    def destroy_node(self):
        self._stop_slam()
        self._stop_localization()
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
