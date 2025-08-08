import rclpy
from rclpy.node import Node
import random
from robotnik_msgs.msg import BatteryStatus

from prometheus_client import start_http_server, Gauge

class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')
        self.subscription = self.create_subscription(
            BatteryStatus,
            '/robot/battery_estimator/data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #------// Declare Gauges for Prometheus client

        self.voltage_gauge = Gauge("battery_voltage", "Voltage of the battery in volts")
        self.current_gauge = Gauge("battery_current", "Current drawn by the battery in amperes")
        self.level_gauge = Gauge("battery_level", "Battery level as a percentage")
        self.time_remaining_gauge = Gauge("battery_time_remaining", "Estimated time remaining on battery in minutes")
        self.time_charging_gauge = Gauge("battery_time_charging", "Time spent charging the battery in minutes")
        self.is_charging_gauge = Gauge("battery_status", "Charging status of the battery (1 for true, 0 for false)")

        start_http_server(8000)  # Expose metrics on port 8000

    def listener_callback(self, msg):
        self.get_logger().info(f"Received BatteryStatus: Voltage: {msg.voltage}, Current: {msg.current}, Level: {msg.level}, Is Charging: {msg.is_charging}")

        self.voltage_gauge.set(msg.voltage)
        self.current_gauge.set(msg.current)
        self.level_gauge.set(msg.level)
        self.time_remaining_gauge.set(msg.time_remaining)
        self.time_charging_gauge.set(msg.time_charging)
        self.is_charging_gauge.set(msg.is_charging)

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    battery_monitor.get_logger().info("Battery Monitor Node Started")
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()