"""ROS 2 rover mission controller for the Gazebo Harmonic demo.

Stages: scan from a distance -> reject uncertain prediction -> inspect closer ->
assign material class -> navigate to next object -> return to base.
"""

from __future__ import annotations

import json
import math
import random
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .core import CLASS_NAMES, PACKAGES, GaussianMaterialAI, plan


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def heading(quat):
    return math.atan2(2 * (quat.w * quat.z + quat.x * quat.y),
                      1 - 2 * (quat.y * quat.y + quat.z * quat.z))


class RescueController(Node):
    def __init__(self):
        super().__init__('rescue_ai_controller')
        self.command = self.create_publisher(Twist, '/model/rescue_rover/cmd_vel', 10)
        self.status = self.create_publisher(String, '/rescue_ai/status', 10)
        self.telemetry = self.create_publisher(String, '/rescue_ai/telemetry', 10)
        self.create_subscription(Odometry, '/model/rescue_rover/ground_truth', self.odom_cb, 10)
        self.create_timer(.067, self.tick)
        self.model = GaussianMaterialAI()
        self.rng = random.Random(38)
        self.pose = None
        self.last_odom_rx = None
        self.last_pose_log = 0.0
        self.last_drive_log = 0.0
        self.last_wait_log = time.monotonic()
        self.phase = 'TO_SCAN'
        self.goal = PACKAGES[0]['scan']
        self.waypoints = []
        self.index = 0
        self.scan_done_at = None
        self.correct = 0
        self.extra_inspections = 0
        self.inspections_current = 0
        self.last_status = None
        self.started = time.monotonic()
        self.records = {p['name']: {'classification': None, 'confidence': None,
                                    'observations': 0, 'decision': 'PENDING'} for p in PACKAGES}
        self.last_prediction = None
        self.last_confidence = None
        self.last_distance = None
        self.last_decision = 'AWAITING SCAN'
        self.events = []
        self.create_timer(0.25, self.publish_telemetry)
        self.get_logger().info('Waiting for Gazebo ground-truth pose...')

    def odom_cb(self, msg):
        # The OdometryPublisher reports the Gazebo model pose in world coordinates.
        # Do not rebase this pose as wheel-integrated DiffDrive odometry.
        p = msg.pose.pose.position
        angle = heading(msg.pose.pose.orientation)
        if not all(map(math.isfinite, (p.x, p.y, angle))):
            self.idle()
            self.announce('SAFETY STOP: invalid ground-truth pose')
            self.phase = 'STOPPED'
            return
        self.pose = (p.x, p.y, angle)
        self.last_odom_rx = time.monotonic()

    def announce(self, message):
        if message == self.last_status:
            return
        self.last_status = message
        self.events.append({'t': round(time.monotonic() - self.started, 1), 'message': message})
        self.events = self.events[-24:]
        self.get_logger().info(message)
        status_msg = String()
        status_msg.data = message
        self.status.publish(status_msg)

    def publish_telemetry(self):
        """Expose the real controller state to the browser dashboard via ROS 2."""
        current = PACKAGES[self.index]['name'] if self.index < len(PACKAGES) else None
        payload = {
            'elapsed_s': round(time.monotonic() - self.started, 1),
            'phase': self.phase,
            'current_box': current,
            'completed': sum(1 for item in self.records.values() if item['classification'] is not None),
            'correct': self.correct,
            'active_inspections': self.extra_inspections,
            'goal': [round(v, 3) for v in self.goal],
            'pose': {'x': round(self.pose[0], 3), 'y': round(self.pose[1], 3),
                     'yaw_deg': round(math.degrees(self.pose[2]), 1)} if self.pose else None,
            'last_prediction': self.last_prediction,
            'last_confidence': self.last_confidence,
            'last_distance': self.last_distance,
            'last_decision': self.last_decision,
            'boxes': self.records,
            'events': self.events,
            'last_status': self.last_status,
            'sensor_type': 'synthetic_multispectral',
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True, allow_nan=False)
        self.telemetry.publish(msg)

    def new_goal(self, point, phase):
        self.goal = point
        self.phase = phase
        try:
            self.waypoints = plan(self.pose[:2], point)
        except (ValueError, RuntimeError) as exc:
            self.idle()
            self.phase = 'STOPPED'
            self.announce(f'SAFETY STOP: planning failed: {exc}')
            return
        self.announce(f'{phase}: navigation to ({point[0]:.2f}, {point[1]:.2f})')
        self.get_logger().info(f'PATH PLAN: {len(self.waypoints)} waypoint(s) {self.waypoints}')

    def idle(self):
        self.command.publish(Twist())

    def on_arrival(self):
        self.idle()
        self.waypoints = []
        if self.phase in ('TO_SCAN', 'TO_INSPECT'):
            self.phase = 'SENSING'
            self.scan_done_at = time.monotonic() + .8
            self.announce('Sensor sampling...')
        elif self.phase == 'TO_HOME':
            self.phase = 'FINISHED'
            self.announce(f'MISSION COMPLETE: {self.correct}/3 correct, '
                          f'{self.extra_inspections} active inspections')

    def classify(self):
        obj = PACKAGES[self.index]
        d = math.dist(self.pose[:2], obj['pos'])
        sensed = self.model.sense(obj['class'], d, self.rng)
        prediction, confidence, _ = self.model.predict(sensed, d)
        name = obj['name']
        self.last_prediction = CLASS_NAMES[prediction]
        self.last_confidence = round(confidence, 4)
        self.last_distance = round(d, 3)
        self.records[name]['observations'] += 1
        self.announce(f'BOX {obj["name"]} AI: {CLASS_NAMES[prediction]}, '
                      f'confidence={confidence:.1%}, distance={d:.2f}m')
        if confidence < .77 and self.inspections_current < 2:
            self.extra_inspections += 1
            self.inspections_current += 1
            self.last_decision = 'RE-INSPECT'
            self.records[name]['decision'] = 'RE-INSPECT'
            x, y = obj['pos']
            # Stop at a collision-safe inspection standoff from the package.
            self.new_goal((x - .80, y - .08), 'TO_INSPECT')
            return
        self.last_decision = 'CLASSIFIED' if confidence >= .77 else 'CLASSIFIED (MAX INSPECTIONS)'
        self.records[name]['classification'] = CLASS_NAMES[prediction]
        self.records[name]['confidence'] = round(confidence, 4)
        self.records[name]['decision'] = self.last_decision
        if prediction == obj['class']:
            self.correct += 1
        self.announce(f'BOX {obj["name"]} classified as {CLASS_NAMES[prediction]} '
                      f'({confidence:.0%})')
        self.index += 1
        self.inspections_current = 0
        if self.index >= len(PACKAGES):
            self.new_goal((1.05, 1.08), 'TO_HOME')
        else:
            self.new_goal(PACKAGES[self.index]['scan'], 'TO_SCAN')

    def tick(self):
        if self.pose is None:
            now = time.monotonic()
            if now - self.last_wait_log >= 5.0:
                self.get_logger().warning('Waiting for /model/rescue_rover/ground_truth (Gazebo pose)...')
                self.last_wait_log = now
            return
        if self.phase in ('FINISHED', 'STOPPED'):
            self.idle()
            return
        if self.last_odom_rx is None or time.monotonic() - self.last_odom_rx > 1.5:
            self.idle()
            self.announce('SAFETY STOP: stale Gazebo pose')
            return
        x, y, _ = self.pose
        if not (0.24 <= x <= 11.76 and 0.24 <= y <= 6.76):
            self.idle()
            self.phase = 'STOPPED'
            self.announce(f'SAFETY STOP: outside field, pose=({x:.2f},{y:.2f})')
            return
        now = time.monotonic()
        if now - self.last_pose_log >= 3.0:
            self.get_logger().info(f'WORLD POSE: x={x:.2f}m y={y:.2f}m yaw={math.degrees(self.pose[2]):.1f}deg phase={self.phase}')
            self.last_pose_log = now
        if self.phase == 'SENSING':
            self.idle()
            if time.monotonic() >= self.scan_done_at:
                self.classify()
            return
        if not self.waypoints:
            try:
                self.waypoints = plan(self.pose[:2], self.goal)
            except (ValueError, RuntimeError) as exc:
                self.idle()
                self.phase = 'STOPPED'
                self.announce(f'SAFETY STOP: planning failed: {exc}')
                return
        x, y, theta = self.pose
        while self.waypoints and math.dist((x, y), self.waypoints[0]) < .08:
            self.waypoints.pop(0)
        if not self.waypoints:
            self.on_arrival()
            return
        target = self.waypoints[0]
        dist = math.dist((x, y), target)
        desired = math.atan2(target[1] - y, target[0] - x)
        error = wrap(desired - theta)
        velocity = Twist()
        velocity.angular.z = max(-1.0, min(1.0, 2.0 * error))
        velocity.linear.x = min(.40, dist * .75) * max(0, math.cos(error)) ** 2
        if now - self.last_drive_log >= 3.0:
            self.get_logger().info(
                f'DRIVE DEBUG: target=({target[0]:.2f},{target[1]:.2f}) '
                f'heading_error={math.degrees(error):.1f}deg '
                f'linear={velocity.linear.x:.2f} angular={velocity.angular.z:.2f}'
            )
            self.last_drive_log = now
        self.command.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = RescueController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.idle()
        node.destroy_node()
        rclpy.shutdown()
