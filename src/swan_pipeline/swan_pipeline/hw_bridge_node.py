#!/usr/bin/env python3
"""
[하드웨어 브리지] /cmd_vel → ESP32 시리얼 패킷.  시뮬↔실물의 유일한 경계선.

이 노드 위쪽(인지·판단·Jerk 제어)은 시뮬/실물 100% 동일 코드.
아래쪽만 바뀜:  시뮬=gz diff_drive  /  실물=ESP32 → 모터드라이버(또는 조이스틱 전압)

사용:
  # 실물 (ESP32 USB 연결)
  ros2 run swan_pipeline hw_bridge_node --ros-args -p port:=/dev/ttyUSB0
  # 부품 없이 검증 (드라이런 — 패킷을 로그로만 출력)
  ros2 run swan_pipeline hw_bridge_node --ros-args -p dry_run:=true

프로토콜 (호스트 → ESP32, 8바이트):
  [0xAA][seq][v_hi][v_lo][w_hi][w_lo][flags][crc8]
    v: int16 mm/s , w: int16 mrad/s , flags bit0=자율모드, crc8=앞 7바이트 XOR
"""
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

HEADER = 0xAA


def crc8_xor(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class HwBridge(Node):
    def __init__(self):
        super().__init__('hw_bridge_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 20.0)      # ESP32 워치독(150ms)보다 빨라야 함
        self.declare_parameter('dry_run', False)     # True면 시리얼 없이 로그만
        self.declare_parameter('v_limit', 1.67)      # 안전 상한 [m/s]
        self.declare_parameter('w_limit', 1.5)       # 안전 상한 [rad/s]
        self.declare_parameter('autonomous', True)   # flags bit0 (릴레이 ON 요청)

        g = lambda n: self.get_parameter(n).value
        self.dry = g('dry_run')
        self.v_lim, self.w_lim = g('v_limit'), g('w_limit')
        self.auto = g('autonomous')
        rate = g('rate_hz')

        self.ser = None
        if not self.dry:
            try:
                import serial
                self.ser = serial.Serial(g('port'), g('baud'), timeout=0.01)
                self.get_logger().info(f"시리얼 연결: {g('port')} @ {g('baud')}")
            except Exception as e:
                self.get_logger().error(f"시리얼 실패 → 드라이런 전환: {e}")
                self.dry = True

        self.v = self.w = 0.0
        self.seq = 0
        self.sent = 0
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.state_pub = self.create_publisher(String, '/swan/hw_state', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"hw_bridge 시작: {'드라이런(로그만)' if self.dry else '실물'} , {rate}Hz")

    def on_cmd(self, m: Twist):
        # 브리지에서도 한 번 더 클램프 — 상위가 이상해도 하드웨어엔 안전값만
        self.v = clamp(m.linear.x, -self.v_lim, self.v_lim)
        self.w = clamp(m.angular.z, -self.w_lim, self.w_lim)

    def build_packet(self):
        v_mm = int(clamp(self.v * 1000.0, -32767, 32767))    # m/s → mm/s
        w_mr = int(clamp(self.w * 1000.0, -32767, 32767))    # rad/s → mrad/s
        flags = 0x01 if self.auto else 0x00
        body = struct.pack('<BBhhB', HEADER, self.seq & 0xFF, v_mm, w_mr, flags)
        return body + bytes([crc8_xor(body)])

    def tick(self):
        pkt = self.build_packet()
        self.seq += 1
        if self.ser:
            try:
                self.ser.write(pkt)
                self.read_telemetry()
            except Exception as e:
                self.get_logger().error(f"시리얼 쓰기 실패: {e}")
        else:
            self.sent += 1
            if self.sent % 20 == 0:      # 1초에 한 번만 로그
                self.get_logger().info(
                    f"[드라이런] v={self.v:+.2f} m/s  w={self.w:+.2f} rad/s  → {pkt.hex(' ')}")
        s = String()
        s.data = f"{'dry' if self.dry else 'serial'} v={self.v:.2f} w={self.w:.2f}"
        self.state_pub.publish(s)

    def read_telemetry(self):
        """ESP32 → 호스트: [0xBB][state][v_hi][v_lo][err][crc8]"""
        try:
            data = self.ser.read(6)
            if len(data) == 6 and data[0] == 0xBB and crc8_xor(data[:5]) == data[5]:
                state, err = data[1], data[4]
                names = {0: '수동', 1: '자율', 2: '워치독정지', 3: 'E-stop'}
                if err:
                    self.get_logger().warn(f"ESP32 오류코드 0x{err:02x} (상태:{names.get(state,'?')})")
        except Exception:
            pass

    def destroy_node(self):
        # 종료 시 반드시 정지 명령
        if self.ser:
            try:
                self.v = self.w = 0.0
                self.ser.write(self.build_packet())
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = HwBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
