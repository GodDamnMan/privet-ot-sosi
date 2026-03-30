#!/usr/bin/env python3
import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from smbus2 import SMBus


class MPU6050Node(Node):

    REG_PWR_MGMT_1 = 0x6B
    REG_ACCEL_START = 0x3B  
    ACCEL_LSB_PER_G = 16384.0
    
    GYRO_LSB_PER_DPS = 131.0
    G_STD = 9.80665

    def __init__(self) -> None:
        super().__init__("mpu6050_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x68)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("frame_id", "imu_link")

        self._bus_id = int(self.get_parameter("i2c_bus").value)
        self._addr = int(self.get_parameter("i2c_address").value)
        self._rate = float(self.get_parameter("publish_rate_hz").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        self._pub = self.create_publisher(Imu, "/imu/raw", 10)

        self._bus: Optional[SMBus] = None
        self._open_and_init_sensor()

        # throttle warning spam
        self._last_warn_ns = 0

        period = 1.0 / self._rate if self._rate > 0.0 else 0.01
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"MPU6050 IMU publisher started: bus={self._bus_id}, addr=0x{self._addr:02X}, "
            f"rate={self._rate} Hz, frame_id={self._frame_id}"
        )

    def _open_and_init_sensor(self) -> None:
        self._bus = SMBus(self._bus_id)
        # Wake once
        self._bus.write_byte_data(self._addr, self.REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

    def _recover_i2c(self, err: Exception) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > 1_000_000_000:
            self.get_logger().warn(f"I2C read failed: {err} (recovering...)")
            self._last_warn_ns = now_ns

        try:
            if self._bus is not None:
                self._bus.write_byte_data(self._addr, self.REG_PWR_MGMT_1, 0x00)
                return
        except Exception:
            pass

        try:
            if self._bus is not None:
                self._bus.close()
        except Exception:
            pass
        self._bus = None

        try:
            self._open_and_init_sensor()
        except Exception:
            self._bus = None

    @staticmethod
    def _int16_from(hi: int, lo: int) -> int:
        value = (hi << 8) | lo
        if value & 0x8000:
            value -= 0x10000
        return value

    def _read_block(self, start_reg: int, length: int) -> List[int]:
        assert self._bus is not None
        # SMBus does: write register pointer -> repeated start -> read N bytes
        return self._bus.read_i2c_block_data(self._addr, start_reg, length)

    def _tick(self) -> None:
        if self._bus is None:
            self._recover_i2c(RuntimeError("I2C bus not open"))
            return

        try:
            data = self._read_block(self.REG_ACCEL_START, 14)

            ax_raw = self._int16_from(data[0], data[1])
            ay_raw = self._int16_from(data[2], data[3])
            az_raw = self._int16_from(data[4], data[5])

            gx_raw = self._int16_from(data[8], data[9])
            gy_raw = self._int16_from(data[10], data[11])
            gz_raw = self._int16_from(data[12], data[13])

            # Convert accel to m/s^2 (default ±2g)
            ax = (ax_raw / self.ACCEL_LSB_PER_G) * self.G_STD
            ay = (ay_raw / self.ACCEL_LSB_PER_G) * self.G_STD
            az = (az_raw / self.ACCEL_LSB_PER_G) * self.G_STD

            # Convert gyro to rad/s (default ±250 deg/s)
            gx_dps = gx_raw / self.GYRO_LSB_PER_DPS
            gy_dps = gy_raw / self.GYRO_LSB_PER_DPS
            gz_dps = gz_raw / self.GYRO_LSB_PER_DPS

            gx = gx_dps * (math.pi / 180.0)
            gy = gy_dps * (math.pi / 180.0)
            gz = gz_dps * (math.pi / 180.0)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id

            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)

            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)

            # Orientation not provided by this RAW node
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0

            msg.angular_velocity_covariance = [
                1e-3, 0.0, 0.0,
                0.0, 1e-3, 0.0,
                0.0, 0.0, 1e-3
            ]
            msg.linear_acceleration_covariance = [
                1e-2, 0.0, 0.0,
                0.0, 1e-2, 0.0,
                0.0, 0.0, 1e-2
            ]

            self._pub.publish(msg)

        except OSError as e:
            self._recover_i2c(e)

    def destroy_node(self) -> bool:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
