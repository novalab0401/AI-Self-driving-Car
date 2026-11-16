#!/usr/bin/python3
"""Single-window Qt dashboard for the LIMO sim.

Combines traffic light color control, robot steering (keyboard teleop,
publishing geometry_msgs/Twist on /cmd_vel) and camera capture into one
window, launched alongside the simulator.

[GZ] Ported from limo_dashboard/dashboard_node.py of JACKSON-KIM-JAEHO/limo_ros2
(branch humble_low_Mesh, Apache-2.0).

  ros2 run limo_gz dashboard_node.py

Traffic light color is set by publishing std_msgs/String ("red" |
"yellow" | "green" | "off") on /traffic_light/color, read by the
traffic light plugin (src/traffic_light_plugin.cpp).

Same keys as teleop_twist_keyboard, but linear and angular are set
independently instead of as fixed combos - teleop_twist_keyboard's
own i/j/k/l zero out the other axis (i = pure forward, j = pure
turn-in-place), so holding forward and tapping a turn key doesn't
curve the path, only u/o/m/. (fixed diagonals) do. Setting each axis
on its own key lets "i" (hold forward) + "l" (nudge right) combine
into an actual curve.

[GZ] Both axes are hold-to-drive here: the robot moves only while i/, is held and
steers only while j/l is held. (In the original, i/, latched the speed until k.)

[GZ] j/l mean "front wheels left/right", like a steering wheel or a joystick, also
when reversing. /cmd_vel angular.z is a yaw rate, and for the same yaw rate the
wheels point the other way in reverse, so the sign of angular.z is flipped while
linear.x is negative (see _command).

To add a new panel later: write a `_build_..._group(self) -> QGroupBox`
method and add it to the QVBoxLayout in `_build_ui`.
"""

import os
import signal
import sys
import threading

import cv2
import numpy as np
# opencv-python bundles its own (older/mismatched) Qt plugins and points
# QT_QPA_PLATFORM_PLUGIN_PATH at them on import, which breaks PyQt5's xcb
# platform plugin lookup ("Could not load the Qt platform plugin 'xcb'").
# Drop it so PyQt5 falls back to the system Qt5 plugins it actually needs.
os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

from PyQt5.QtCore import Qt, QEvent, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFormLayout, QGroupBox, QPushButton, QLabel, QDoubleSpinBox, QSpinBox,
    QLineEdit,
)

LIGHT_COLORS = ['red', 'yellow', 'green']

PUBLISH_HZ = 20.0

LINEAR_KEYS = {Qt.Key_I: 1, Qt.Key_Comma: -1}
ANGULAR_KEYS = {Qt.Key_J: 1, Qt.Key_L: -1}
STOP_KEYS = {Qt.Key_K}
SPEED_STEP = 1.1  # multiply/divide by this, same as teleop_twist_keyboard's 10% (1/1.1 ~= 0.9)
SPEED_UP_KEYS = {Qt.Key_Q}
SPEED_DOWN_KEYS = {Qt.Key_Z}
LINEAR_UP_KEYS = {Qt.Key_W}
LINEAR_DOWN_KEYS = {Qt.Key_X}
ANGULAR_UP_KEYS = {Qt.Key_E}
ANGULAR_DOWN_KEYS = {Qt.Key_C}

# Bounds for the max-speed / max-turn spin boxes (also clamp the q/z/w/x/e/c
# keyboard step so the two controls can't drift out of sync with each other).
# [GZ] The Ackermann plugin in urdf/ackermann.xacro caps both linear and angular
# at its max_velocity (1.0 for now), so values above that have no extra effect.
SPEED_MIN, SPEED_MAX = 0.05, 3.0
TURN_MIN, TURN_MAX = 0.1, 5.0

# Manual capture hotkey. Deliberately NOT 'k' - that's already STOP above,
# and driving + capturing at the same time is exactly the use case, so the
# two must not collide.
CAPTURE_KEY = Qt.Key_P
CAPTURE_KEY_NAME = 'P'

# 워크스페이스 이름/위치를 가정하지 않는 기본값 (누구 컴퓨터에서 clone해도 동작).
# 다른 곳에 저장하고 싶으면 패널의 "Save dir" 칸에서 바로 바꾸면 된다.
DEFAULT_CAPTURE_DIR = os.path.expanduser('~/limo_capture')


class DashboardNode(Node):
    """Just the ROS I/O side (publishers). No Qt here."""

    def __init__(self):
        super().__init__('limo_dashboard')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.light_pub = self.create_publisher(String, '/traffic_light/color', 10)

        # rclpy.spin() runs on a background thread (see main()) while Qt
        # owns the main thread, so the latest frame is handed over through
        # a lock instead of touching Qt widgets from the ROS callback.
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self.image_sub = self.create_subscription(
            Image, '/rgb/image_raw', self._image_cb, 10)

    def _image_cb(self, msg):
        # [GZ] Converted by hand instead of with cv_bridge: cv_bridge's compiled part is
        # built for NumPy 1.x and fails to import where a pip-installed NumPy 2.x is in
        # use (common next to ultralytics). The bridge delivers rgb8 or bgr8.
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(
                f'unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        cv_image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        else:
            cv_image = cv_image.copy()
        with self._frame_lock:
            self._latest_frame = cv_image

    def get_latest_frame(self):
        """Most recent camera frame (BGR ndarray), or None if none received yet."""
        with self._frame_lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def set_light(self, color):
        self.light_pub.publish(String(data=color))


class DashboardWindow(QMainWindow):

    def __init__(self, ros_node: DashboardNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('LIMO Dashboard')

        self._teleop_active = False
        self._linear = 0.0
        self._angular = 0.0
        self._speed = 0.5   # m/s, teleop_twist_keyboard's default
        self._turn = 1.0    # rad/s, teleop_twist_keyboard's default
        # Which angular key (if any) is currently held down, so releasing
        # it can self-center the steering back to 0.
        self._active_angular_key = None
        # [GZ] Same for linear: releasing the held i/, key stops the robot.
        self._active_linear_key = None

        self._capture_hotkey_active = False
        self._capture_count = 0
        self._auto_capture_remaining = 0

        self._build_ui()
        self.setFocusPolicy(Qt.StrongFocus)

        # Keep publishing the current command at a fixed rate, not just
        # on key press, so the command stays fresh while a key is held.
        self._cmd_timer = QTimer(self)
        self._cmd_timer.timeout.connect(self._publish_cmd_vel)
        self._cmd_timer.start(int(1000 / PUBLISH_HZ))

        self._auto_capture_timer = QTimer(self)
        self._auto_capture_timer.timeout.connect(self._auto_capture_tick)

    def _build_ui(self):
        central = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self._build_traffic_light_group())
        layout.addWidget(self._build_steering_group())
        layout.addWidget(self._build_capture_group())
        # Add future panels here, e.g.:
        # layout.addWidget(self._build_mission_status_group())
        central.setLayout(layout)
        self.setCentralWidget(central)

    def _build_traffic_light_group(self) -> QGroupBox:
        group = QGroupBox('Traffic Light')
        row = QHBoxLayout()
        colors = {'red': '#d62728', 'yellow': '#e0b400', 'green': '#2ca02c'}
        for color, hexcode in colors.items():
            btn = QPushButton(color.capitalize())
            btn.setStyleSheet(
                f'background-color: {hexcode}; color: white; font-weight: bold; padding: 8px;'
            )
            btn.clicked.connect(lambda checked, c=color: self._set_traffic_light(c))
            row.addWidget(btn)
        group.setLayout(row)
        return group

    def _set_traffic_light(self, active_color):
        self.ros_node.set_light(active_color)

    def _build_steering_group(self) -> QGroupBox:
        group = QGroupBox('Robot Steering (keyboard teleop)')
        layout = QVBoxLayout()

        self.teleop_btn = QPushButton('Keyboard Teleop: OFF')
        self.teleop_btn.setCheckable(True)
        self._style_teleop_button()
        self.teleop_btn.clicked.connect(self._toggle_teleop)
        layout.addWidget(self.teleop_btn)

        layout.addWidget(QLabel(
            'hold i/,: forward/back   hold j/l: turn left/right\n'
            '(hold i, tap j/l to curve - releasing a key stops that motion)\n'
            'k: stop   or set the limits below (q/z/w/x/e/c keys do the same)'
        ))

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(SPEED_MIN, SPEED_MAX)
        self.speed_spin.setSingleStep(0.05)
        self.speed_spin.setDecimals(2)
        self.speed_spin.setSuffix(' m/s')
        self.speed_spin.setValue(self._speed)
        self.speed_spin.valueChanged.connect(self._set_speed)

        self.turn_spin = QDoubleSpinBox()
        self.turn_spin.setRange(TURN_MIN, TURN_MAX)
        self.turn_spin.setSingleStep(0.1)
        self.turn_spin.setDecimals(2)
        self.turn_spin.setSuffix(' rad/s')
        self.turn_spin.setValue(self._turn)
        self.turn_spin.valueChanged.connect(self._set_turn)

        limits_layout = QFormLayout()
        limits_layout.addRow('Max speed:', self.speed_spin)
        limits_layout.addRow('Max turn:', self.turn_spin)
        layout.addLayout(limits_layout)

        self.cmd_label = QLabel('linear 0.00 m/s  angular 0.00 rad/s  (wheels straight)')
        layout.addWidget(self.cmd_label)

        stop_btn = QPushButton('STOP')
        stop_btn.setStyleSheet('background-color: #d62728; color: white; font-weight: bold; padding: 10px;')
        stop_btn.clicked.connect(self._stop)
        layout.addWidget(stop_btn)

        group.setLayout(layout)
        return group

    def _build_capture_group(self) -> QGroupBox:
        group = QGroupBox('Camera Capture (/rgb/image_raw)')
        layout = QVBoxLayout()

        path_form = QFormLayout()
        self.capture_dir_edit = QLineEdit(DEFAULT_CAPTURE_DIR)
        path_form.addRow('Save dir:', self.capture_dir_edit)
        self.capture_prefix_edit = QLineEdit('limo')
        path_form.addRow('Prefix:', self.capture_prefix_edit)
        layout.addLayout(path_form)

        self.capture_hotkey_btn = QPushButton()
        self.capture_hotkey_btn.setCheckable(True)
        self._style_capture_hotkey_button()
        self.capture_hotkey_btn.clicked.connect(self._toggle_capture_hotkey)
        layout.addWidget(self.capture_hotkey_btn)

        manual_btn = QPushButton('Capture Now')
        manual_btn.setStyleSheet(
            'background-color: #1f77b4; color: white; font-weight: bold; padding: 8px;')
        manual_btn.clicked.connect(self._manual_capture)
        layout.addWidget(manual_btn)

        auto_form = QFormLayout()
        self.interval_spin = QDoubleSpinBox()
        self.interval_spin.setRange(0.2, 60.0)
        self.interval_spin.setSingleStep(0.5)
        self.interval_spin.setDecimals(1)
        self.interval_spin.setSuffix(' s')
        self.interval_spin.setValue(2.0)
        auto_form.addRow('Interval:', self.interval_spin)

        self.max_frames_spin = QSpinBox()
        self.max_frames_spin.setRange(1, 5000)
        self.max_frames_spin.setValue(50)
        auto_form.addRow('Max frames:', self.max_frames_spin)
        layout.addLayout(auto_form)

        self.auto_capture_btn = QPushButton()
        self.auto_capture_btn.setCheckable(True)
        self._style_auto_capture_button()
        self.auto_capture_btn.clicked.connect(self._toggle_auto_capture)
        layout.addWidget(self.auto_capture_btn)

        self.capture_status_label = QLabel('0 frames captured')
        layout.addWidget(self.capture_status_label)

        group.setLayout(layout)
        return group

    def _style_capture_hotkey_button(self):
        if self._capture_hotkey_active:
            self.capture_hotkey_btn.setText(f'Capture Hotkey ({CAPTURE_KEY_NAME}): ON')
            self.capture_hotkey_btn.setStyleSheet(
                'background-color: #2ca02c; color: white; font-weight: bold; padding: 8px;')
        else:
            self.capture_hotkey_btn.setText(f'Capture Hotkey ({CAPTURE_KEY_NAME}): OFF')
            self.capture_hotkey_btn.setStyleSheet(
                'background-color: #888888; color: white; font-weight: bold; padding: 8px;')

    def _toggle_capture_hotkey(self):
        self._capture_hotkey_active = not self._capture_hotkey_active
        self._style_capture_hotkey_button()
        if self._capture_hotkey_active:
            self.setFocus()  # so key presses reach keyPressEvent below

    def _style_auto_capture_button(self):
        if self.auto_capture_btn.isChecked():
            self.auto_capture_btn.setText('Stop Auto Capture')
            self.auto_capture_btn.setStyleSheet(
                'background-color: #d62728; color: white; font-weight: bold; padding: 8px;')
        else:
            self.auto_capture_btn.setText('Start Auto Capture')
            self.auto_capture_btn.setStyleSheet(
                'background-color: #1f77b4; color: white; font-weight: bold; padding: 8px;')

    def _toggle_auto_capture(self):
        if self.auto_capture_btn.isChecked():
            self._auto_capture_remaining = self.max_frames_spin.value()
            self._auto_capture_timer.start(int(self.interval_spin.value() * 1000))
            self.interval_spin.setEnabled(False)
            self.max_frames_spin.setEnabled(False)
        else:
            self._auto_capture_timer.stop()
            self.interval_spin.setEnabled(True)
            self.max_frames_spin.setEnabled(True)
        self._style_auto_capture_button()

    def _auto_capture_tick(self):
        self._save_current_frame(context='Auto capture')
        self._auto_capture_remaining -= 1
        if self._auto_capture_remaining <= 0:
            self.auto_capture_btn.setChecked(False)
            self._toggle_auto_capture()

    def _manual_capture(self):
        self._save_current_frame(context='Capture')

    def _save_current_frame(self, context):
        frame = self.ros_node.get_latest_frame()
        if frame is None:
            self.capture_status_label.setText(
                f'{context} failed: no frame from /rgb/image_raw yet '
                '(is the simulator/camera running?)')
            return

        out_dir = self.capture_dir_edit.text().strip() or DEFAULT_CAPTURE_DIR
        os.makedirs(out_dir, exist_ok=True)
        prefix = self.capture_prefix_edit.text().strip() or 'limo'
        self._capture_count += 1
        filename = f'{prefix}_{self._capture_count:05d}.png'
        cv2.imwrite(os.path.join(out_dir, filename), frame)
        self.capture_status_label.setText(
            f'{self._capture_count} frames captured (latest: {filename})')

    def _style_teleop_button(self):
        if self._teleop_active:
            self.teleop_btn.setText('Keyboard Teleop: ON')
            self.teleop_btn.setStyleSheet(
                'background-color: #2ca02c; color: white; font-weight: bold; padding: 10px;')
        else:
            self.teleop_btn.setText('Keyboard Teleop: OFF')
            self.teleop_btn.setStyleSheet(
                'background-color: #888888; color: white; font-weight: bold; padding: 10px;')

    def _toggle_teleop(self):
        self._teleop_active = not self._teleop_active
        self._style_teleop_button()
        if self._teleop_active:
            self.setFocus()  # so key presses reach keyPressEvent below
        else:
            # [GZ] one explicit stop here, because nothing is published while OFF
            self._stop()

    def keyPressEvent(self, event):
        # Capture hotkey is independent of the teleop toggle below - driving
        # and snapping photos at the same time is the whole point, and it's
        # a different key (P) from STOP (K) so they can't collide.
        if (self._capture_hotkey_active and not event.isAutoRepeat()
                and event.key() == CAPTURE_KEY):
            self._manual_capture()
            return

        if not self._teleop_active or event.isAutoRepeat():
            super().keyPressEvent(event)
            return
        key = event.key()
        if key in LINEAR_KEYS:
            self._linear = LINEAR_KEYS[key] * self._speed
            self._active_linear_key = key
        elif key in ANGULAR_KEYS:
            self._angular = ANGULAR_KEYS[key] * self._turn
            self._active_angular_key = key
        elif key in STOP_KEYS:
            self._linear = 0.0
            self._angular = 0.0
            self._active_linear_key = None
            self._active_angular_key = None
        elif key in SPEED_UP_KEYS:
            self._set_speed(self._speed * SPEED_STEP)
            self._set_turn(self._turn * SPEED_STEP)
        elif key in SPEED_DOWN_KEYS:
            self._set_speed(self._speed / SPEED_STEP)
            self._set_turn(self._turn / SPEED_STEP)
        elif key in LINEAR_UP_KEYS:
            self._set_speed(self._speed * SPEED_STEP)
        elif key in LINEAR_DOWN_KEYS:
            self._set_speed(self._speed / SPEED_STEP)
        elif key in ANGULAR_UP_KEYS:
            self._set_turn(self._turn * SPEED_STEP)
        elif key in ANGULAR_DOWN_KEYS:
            self._set_turn(self._turn / SPEED_STEP)
        else:
            super().keyPressEvent(event)
            return
        self._update_cmd_label()

    def _set_speed(self, value):
        """Set max linear speed - shared by the spin box and q/w/x keys."""
        self._speed = max(SPEED_MIN, min(SPEED_MAX, value))
        self.speed_spin.blockSignals(True)
        self.speed_spin.setValue(self._speed)
        self.speed_spin.blockSignals(False)

    def _set_turn(self, value):
        """Set max angular turn rate - shared by the spin box and e/c/z keys."""
        self._turn = max(TURN_MIN, min(TURN_MAX, value))
        self.turn_spin.blockSignals(True)
        self.turn_spin.setValue(self._turn)
        self.turn_spin.blockSignals(False)

    def keyReleaseEvent(self, event):
        # Steering self-centers: releasing the angular key that's
        # currently driving the turn snaps angular back to 0, same as a
        # real car's wheel returning to straight when you let go.
        # [GZ] Linear does the same: releasing the held i/, key sets speed to 0.
        if not self._teleop_active or event.isAutoRepeat():
            super().keyReleaseEvent(event)
            return
        key = event.key()
        if key in ANGULAR_KEYS and key == self._active_angular_key:
            self._angular = 0.0
            self._active_angular_key = None
            self._update_cmd_label()
        elif key in LINEAR_KEYS and key == self._active_linear_key:
            self._linear = 0.0
            self._active_linear_key = None
            self._update_cmd_label()
        else:
            super().keyReleaseEvent(event)

    def changeEvent(self, event):
        # [GZ] A key release is not delivered once another window becomes active,
        # so a held key would keep driving. Stop instead. (Clicking a button inside
        # this window does not count: the window stays active.)
        if (event.type() == QEvent.ActivationChange and not self.isActiveWindow()
                and self._teleop_active and (self._linear or self._angular)):
            self._stop()
        super().changeEvent(event)

    def _command(self):
        """(linear.x, angular.z) to publish. self._angular is the steering side (+ = wheels left)."""
        # [GZ] same wheel direction forward and backward: flip the yaw rate in reverse
        angular = -self._angular if (self._linear < 0.0 and self._angular) else self._angular
        return self._linear, angular

    def _update_cmd_label(self):
        linear, angular = self._command()
        steer = 'left' if self._angular > 0.0 else 'right' if self._angular < 0.0 else 'straight'
        self.cmd_label.setText(f'linear {linear:.2f} m/s  angular {angular:.2f} rad/s  (wheels {steer})')

    def _stop(self):
        self._linear = 0.0
        self._angular = 0.0
        self._active_linear_key = None
        self._active_angular_key = None
        self._update_cmd_label()
        self.ros_node.publish_cmd_vel(0.0, 0.0)

    def _publish_cmd_vel(self):
        # [GZ] The original published all the time. Here nothing is sent while the
        # teleop toggle is OFF, so this window does not overwrite /cmd_vel from
        # other nodes (ros2 topic pub, and later Nav2) with zeros.
        if not self._teleop_active:
            return
        self.ros_node.publish_cmd_vel(*self._command())

    def closeEvent(self, event):
        self._stop()
        super().closeEvent(event)


def _spin(ros_node):
    try:
        rclpy.spin(ros_node)
    except ExternalShutdownException:
        pass


def main(args=None):
    # [GZ] Qt owns the main loop, so Ctrl+C (also from `ros2 launch`) has to close
    # the Qt app. Without this the window stays open after the launch is stopped.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    ros_node = DashboardNode()

    spin_thread = threading.Thread(target=_spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = DashboardWindow(ros_node)
    window.resize(380, 620)
    window.show()
    # close() runs closeEvent, which sends the stop command first
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
