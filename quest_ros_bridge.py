"""Publish Quest Touch poses and analog inputs to ROS 2 without robot control."""

import argparse
import select
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32

from pose_check import parse


FRAME_ID = "quest_world_openxr"


def analog_value(buttons: str, key: str):
    for field in buttons.split(","):
        parts = field.strip().split()
        if len(parts) == 2 and parts[0] == key:
            try:
                return float(parts[1])
            except ValueError:
                return None
    return None


class QuestBridge(Node):
    def __init__(self):
        super().__init__("quest_pose_bridge")
        self.pose_pubs = {
            side: self.create_publisher(PoseStamped, f"/quest/{side}/pose", 10)
            for side in ("left", "right")
        }
        self.analog_pubs = {
            (side, control): self.create_publisher(
                Float32, f"/quest/{side}/{control}", 10
            )
            for side in ("left", "right")
            for control in ("trigger", "grip")
        }
        self.frames = 0
        self.last_pose_time = None

    def publish_packet(self, packet):
        poses, buttons = packet
        stamp = self.get_clock().now().to_msg()
        for key, (position, orientation) in poses.items():
            side = "left" if key == "l" else "right"
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = FRAME_ID
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ) = orientation
            self.pose_pubs[side].publish(msg)
            for control, suffix in (("trigger", "Trig"), ("grip", "Grip")):
                value = analog_value(buttons, side + suffix)
                if value is not None:
                    self.analog_pubs[(side, control)].publish(Float32(data=value))
        if poses:
            self.frames += 1
            self.last_pose_time = time.monotonic()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=0, help="0 means run until Ctrl+C")
    args = parser.parse_args()
    rclpy.init()
    node = QuestBridge()
    process = subprocess.Popen(
        ["adb", "shell", "logcat", "-T", "1", "-s", "wE9ryARX:I", "*:S"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    node.get_logger().info(
        "Publishing /quest/{left,right}/pose, trigger, grip; frame=quest_world_openxr"
    )
    deadline = time.monotonic() + args.seconds if args.seconds else None
    last_report = time.monotonic()
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            if process.poll() is not None:
                raise RuntimeError("adb logcat exited unexpectedly")
            ready, _, _ = select.select([process.stdout], [], [], 0.1)
            if ready:
                packet = parse(process.stdout.readline())
                if packet is not None:
                    node.publish_packet(packet)
            now = time.monotonic()
            if now - last_report >= 5:
                age = None if node.last_pose_time is None else now - node.last_pose_time
                node.get_logger().info(
                    f"pose frames={node.frames}, last pose age={age} seconds"
                )
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
