#!/usr/bin/env python3
"""Exercise the production ROS/USB bridge on a PTY, without robot hardware."""
import collections
import math
import os
import pathlib
import pty
import select
import struct
import subprocess
import tempfile
import time
import zlib
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from ros_interfaces.msg import Behavior, SentryInfoOffline
from std_msgs.msg import UInt32MultiArray
from tf2_ros import TransformBroadcaster


def crc8(data):
    crc = 255
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8c if crc & 1 else 0)
    return crc


def topic_key(name):
    return zlib.crc32(name.encode()) ^ 0xffffffff


def packet(name, payload, timestamp):
    header = b'\x5a' + len(payload).to_bytes(3, 'little')
    header += struct.pack('<I', topic_key(name)) + timestamp.to_bytes(6, 'little') + b'\x01'
    header += bytes([crc8(header)])
    frame = header + payload
    return frame + bytes([crc8(frame)])


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    executable = pathlib.Path(os.environ.get('VISION_BUILD_DIR', root / 'build-link')) / 'sentry_link_check'
    telemetry_only = os.environ.get('TELEMETRY_ONLY_TEST') == '1'
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    rclpy.init()
    node = rclpy.create_node('navigation_bridge_pty_test')
    velocity_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    behavior_pub = node.create_publisher(Behavior, '/sentry/behaivor_send', 10)
    broadcaster = TransformBroadcaster(node)
    received = collections.Counter()
    offline_values = []
    diagnostics_values = []
    offline_sub = node.create_subscription(SentryInfoOffline, '/sentry/offline_info', lambda msg: offline_values.append(msg), 10)
    diagnostics_sub = node.create_subscription(UInt32MultiArray, '/sentry/link_diagnostics', lambda msg: diagnostics_values.append(msg.data), 10)
    del offline_sub, diagnostics_sub
    with tempfile.TemporaryDirectory(prefix='sentry-bridge-test-') as temp:
        config = pathlib.Path(temp) / 'config.yaml'
        config.write_text(f'com_port: "{os.ttyname(slave)}"\nbaudrate: 921600\n')
        log = open(pathlib.Path(temp) / 'runtime.log', 'w+')
        process = subprocess.Popen([str(executable), str(config), '8'] + (['--telemetry-only'] if telemetry_only else []), cwd=root, stdout=log, stderr=subprocess.STDOUT)
        buffer = bytearray()
        records = []
        start = time.monotonic()
        try:
            while time.monotonic() - start < 7.95:
                elapsed = time.monotonic() - start
                stamp = int(elapsed * 1e6) + 1
                frames = [packet('ahrs_quaternion', struct.pack('<4f', 0, 0, 0, 1), stamp),
                          packet('nav_gimbal_feedback', struct.pack('<5fHBBB3x', .1, .2, .3, .4, 23, 100, 1, 0, 127), stamp),
                          packet('offline_info', struct.pack('<fB?fBf?', 12, 1, True, .5, 80, .25, True), stamp),
                          packet('nav_link_diagnostics', struct.pack('<4I', 10, 20, 0, 30), stamp)]
                # Deliberately corrupt one packet; it must not publish the bogus value.
                corrupt = bytearray(packet('offline_info', struct.pack('<fB?fBf?', 999, 1, True, .5, 80, .25, True), stamp))
                corrupt[-1] ^= 1
                stream = b''.join(frames) + corrupt
                os.write(master, stream[:7]); os.write(master, stream[7:])
                if elapsed < 4.5:
                    velocity = Twist(); velocity.linear.x = 1.25; velocity.linear.y = -.5; velocity.angular.z = .2
                    velocity_pub.publish(velocity)
                    behavior = Behavior(); behavior.desired_stance = 2; behavior.scan_yaw_min = -90.0; behavior.scan_yaw_max = 90.0
                    behavior_pub.publish(behavior)
                    # First phase has no TF: translation must be suppressed.
                    if elapsed > 2.5:
                        transform = TransformStamped(); transform.header.stamp = node.get_clock().now().to_msg()
                        transform.header.frame_id = 'camera_init'; transform.child_frame_id = 'base_link'; transform.transform.rotation.w = 1.0
                        broadcaster.sendTransform(transform)
                rclpy.spin_once(node, timeout_sec=.01)
                if select.select([master], [], [], 0)[0]:
                    buffer.extend(os.read(master, 65536))
                while len(buffer) >= 17:
                    if buffer[0] != 90 or crc8(buffer[:15]) != buffer[15]:
                        del buffer[0]; continue
                    size = int.from_bytes(buffer[1:4], 'little'); length = 17 + size
                    if size > 512:
                        del buffer[0]; continue
                    if len(buffer) < length: break
                    frame = bytes(buffer[:length]); del buffer[:length]
                    assert crc8(frame[:-1]) == frame[-1]
                    key = int.from_bytes(frame[4:8], 'little'); payload = frame[16:-1]
                    received[key] += 1
                    if key == topic_key('nav_data'):
                        records.append((elapsed, struct.unpack('<10f?f', payload)))
                    elif key == topic_key('behavior_data'):
                        assert len(payload) == 26
                        if 3 < elapsed < 4:
                            assert payload[1] == 2
                            assert math.isclose(struct.unpack_from('<f', payload, 3)[0], -math.pi / 2, abs_tol=1e-5)
                    elif key == topic_key('fire_notify'):
                        assert payload == b'\x00', 'Test must never send fire=true'
                time.sleep(.01)
                if process.poll() is not None:
                    raise AssertionError('Bridge exited early')
            process.wait(timeout=5)
            log.seek(0); output = log.read(); print(output)
            assert process.returncode == 0
            assert 'fresh_ahrs=1' in output and 'fresh_feedback=1' in output
            assert offline_values and all(math.isclose(v.yaw_camerainit_to_gimbal, 12) for v in offline_values)
            assert diagnostics_values and list(diagnostics_values[-1]) == [10, 20, 0, 30]
            if telemetry_only:
                assert all(key == topic_key('fire_notify') for key in received), 'Telemetry mode must suppress all motion and behavior packets'
                print('PASS: telemetry received; motion/behavior suppressed; only fire=false safety packets allowed')
            else:
                assert any(1.5 < t < 2.4 and abs(v[0]) < 1e-6 and abs(v[1]) < 1e-6 for t, v in records), 'Missing TF must suppress translation'
                assert any(3 < t < 4.3 and math.isclose(v[0], .5, abs_tol=.001) and math.isclose(v[1], 1.25, abs_tol=.001) for t, v in records), 'Body coordinate conversion failed'
                assert any(t > 5.5 and all(abs(x) < 1e-6 for x in v[:3]) for t, v in records), 'Stale commands must stop the chassis'
                assert received[topic_key('nav_data')] > 100 and received[topic_key('behavior_data')] > 100
                print('PASS: ROS to USB, USB to ROS, CRC rejection, fragmented RX, TF gate, units, stale stop, fire=false')
        finally:
            if process.poll() is None: process.terminate(); process.wait(timeout=5)
            log.close()
            os.close(master); os.close(slave); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
