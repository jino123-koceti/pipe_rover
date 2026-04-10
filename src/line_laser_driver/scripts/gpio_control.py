#!/usr/bin/env python3
"""
Jetson GPIO control for line laser on/off
Supports single or dual laser configuration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import time

# Try to import Jetson GPIO, fall back to mock for development
try:
    import Jetson.GPIO as GPIO
    JETSON_AVAILABLE = True
except ImportError:
    JETSON_AVAILABLE = False
    print("Warning: Jetson.GPIO not available, running in simulation mode")


class LaserGPIOController(Node):
    """GPIO controller for line laser on/off control"""

    def __init__(self):
        super().__init__('laser_gpio_controller')

        # Declare parameters
        self.declare_parameter('laser1_gpio_pin', 18)
        self.declare_parameter('laser2_gpio_pin', 23)
        self.declare_parameter('dual_laser_mode', False)
        self.declare_parameter('default_state', False)
        self.declare_parameter('pulse_mode', False)
        self.declare_parameter('pulse_duration_ms', 10)
        self.declare_parameter('simulation_mode', not JETSON_AVAILABLE)

        # Get parameters
        self.laser1_pin = self.get_parameter('laser1_gpio_pin').value
        self.laser2_pin = self.get_parameter('laser2_gpio_pin').value
        self.dual_laser = self.get_parameter('dual_laser_mode').value
        self.default_state = self.get_parameter('default_state').value
        self.pulse_mode = self.get_parameter('pulse_mode').value
        self.pulse_duration = self.get_parameter('pulse_duration_ms').value / 1000.0
        self.simulation_mode = self.get_parameter('simulation_mode').value

        # Current states
        self.laser1_state = False
        self.laser2_state = False

        # Initialize GPIO
        self._init_gpio()

        # Publishers for laser state feedback
        self.laser1_state_pub = self.create_publisher(Bool, 'line_laser/laser1_state', 10)
        self.laser2_state_pub = self.create_publisher(Bool, 'line_laser/laser2_state', 10)

        # Subscribers for laser control
        self.laser1_cmd_sub = self.create_subscription(
            Bool, 'line_laser/laser1_cmd', self._laser1_cmd_callback, 10)
        self.laser2_cmd_sub = self.create_subscription(
            Bool, 'line_laser/laser2_cmd', self._laser2_cmd_callback, 10)

        # Services
        self.srv_laser1_set = self.create_service(
            SetBool, 'line_laser/set_laser1', self._set_laser1_callback)
        self.srv_laser2_set = self.create_service(
            SetBool, 'line_laser/set_laser2', self._set_laser2_callback)
        self.srv_all_on = self.create_service(
            Trigger, 'line_laser/all_on', self._all_on_callback)
        self.srv_all_off = self.create_service(
            Trigger, 'line_laser/all_off', self._all_off_callback)
        self.srv_pulse = self.create_service(
            Trigger, 'line_laser/pulse', self._pulse_callback)

        # Timer for state publishing
        self.timer = self.create_timer(0.1, self._publish_states)  # 10 Hz

        # Set default state
        self._set_laser1(self.default_state)
        if self.dual_laser:
            self._set_laser2(self.default_state)

        self.get_logger().info(f'Laser GPIO Controller initialized')
        self.get_logger().info(f'  Laser 1 GPIO: {self.laser1_pin}')
        if self.dual_laser:
            self.get_logger().info(f'  Laser 2 GPIO: {self.laser2_pin}')
        self.get_logger().info(f'  Simulation mode: {self.simulation_mode}')

    def _init_gpio(self):
        """Initialize GPIO pins"""
        if not self.simulation_mode and JETSON_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(self.laser1_pin, GPIO.OUT, initial=GPIO.LOW)
                if self.dual_laser:
                    GPIO.setup(self.laser2_pin, GPIO.OUT, initial=GPIO.LOW)
                self.get_logger().info('GPIO initialized successfully')
            except Exception as e:
                self.get_logger().error(f'GPIO initialization failed: {e}')
                self.simulation_mode = True
        else:
            self.get_logger().warn('Running in GPIO simulation mode')

    def _set_laser1(self, state: bool):
        """Set laser 1 state"""
        self.laser1_state = state
        if not self.simulation_mode and JETSON_AVAILABLE:
            try:
                GPIO.output(self.laser1_pin, GPIO.HIGH if state else GPIO.LOW)
            except Exception as e:
                self.get_logger().error(f'Failed to set laser 1: {e}')
        self.get_logger().debug(f'Laser 1 set to {"ON" if state else "OFF"}')

    def _set_laser2(self, state: bool):
        """Set laser 2 state"""
        if not self.dual_laser:
            return
        self.laser2_state = state
        if not self.simulation_mode and JETSON_AVAILABLE:
            try:
                GPIO.output(self.laser2_pin, GPIO.HIGH if state else GPIO.LOW)
            except Exception as e:
                self.get_logger().error(f'Failed to set laser 2: {e}')
        self.get_logger().debug(f'Laser 2 set to {"ON" if state else "OFF"}')

    def _laser1_cmd_callback(self, msg: Bool):
        """Callback for laser 1 command topic"""
        self._set_laser1(msg.data)

    def _laser2_cmd_callback(self, msg: Bool):
        """Callback for laser 2 command topic"""
        self._set_laser2(msg.data)

    def _set_laser1_callback(self, request, response):
        """Service callback for setting laser 1"""
        self._set_laser1(request.data)
        response.success = True
        response.message = f'Laser 1 {"ON" if request.data else "OFF"}'
        return response

    def _set_laser2_callback(self, request, response):
        """Service callback for setting laser 2"""
        if not self.dual_laser:
            response.success = False
            response.message = 'Dual laser mode not enabled'
            return response
        self._set_laser2(request.data)
        response.success = True
        response.message = f'Laser 2 {"ON" if request.data else "OFF"}'
        return response

    def _all_on_callback(self, request, response):
        """Turn all lasers on"""
        self._set_laser1(True)
        if self.dual_laser:
            self._set_laser2(True)
        response.success = True
        response.message = 'All lasers ON'
        return response

    def _all_off_callback(self, request, response):
        """Turn all lasers off"""
        self._set_laser1(False)
        if self.dual_laser:
            self._set_laser2(False)
        response.success = True
        response.message = 'All lasers OFF'
        return response

    def _pulse_callback(self, request, response):
        """Generate a single pulse on all lasers"""
        self._set_laser1(True)
        if self.dual_laser:
            self._set_laser2(True)
        time.sleep(self.pulse_duration)
        self._set_laser1(False)
        if self.dual_laser:
            self._set_laser2(False)
        response.success = True
        response.message = f'Pulse generated ({self.pulse_duration*1000:.1f}ms)'
        return response

    def _publish_states(self):
        """Publish current laser states"""
        msg1 = Bool()
        msg1.data = self.laser1_state
        self.laser1_state_pub.publish(msg1)

        if self.dual_laser:
            msg2 = Bool()
            msg2.data = self.laser2_state
            self.laser2_state_pub.publish(msg2)

    def destroy_node(self):
        """Cleanup GPIO on shutdown"""
        self._set_laser1(False)
        if self.dual_laser:
            self._set_laser2(False)
        if not self.simulation_mode and JETSON_AVAILABLE:
            try:
                GPIO.cleanup([self.laser1_pin])
                if self.dual_laser:
                    GPIO.cleanup([self.laser2_pin])
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaserGPIOController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
