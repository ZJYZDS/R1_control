"""Serial receiver for R1 KFS IDs — MapSecReceive.

Frame format: [HEADER(0x5A) | ID1(1B) | ID2(1B) | TAIL(0xA5)]
0x00 = sentinel for "no ID", supplemented by GraphMapperNode.
"""

import rospy
import serial
import threading
import time
import yaml
import os
from std_msgs.msg import Int32MultiArray

from .config import Config as R1Config


def _load_config(yaml_path=None):
    if yaml_path is None:
        yaml_path = os.path.join(os.path.dirname(__file__), "..", "..",
                                 "config", "map_sec_receive_config.yaml")
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


_cfg = _load_config()


class Config:
    SERIAL_PORT = _cfg['serial']['port']
    BAUD_RATE = _cfg['serial']['baud_rate']
    TIMEOUT = _cfg['serial']['timeout']
    HEADER = _cfg['frame']['header']
    TAIL = _cfg['frame']['tail']
    FRAME_LEN = _cfg['frame']['frame_len']
    RECONNECT_BASE_DELAY = _cfg['reconnect']['base_delay']
    RECONNECT_MAX_DELAY = _cfg['reconnect']['max_delay']
    BUFFER_MAX = _cfg['buffer_max']
    RECEIVE_HZ = _cfg['receive_hz']


class MapSectionReceiver:
    def __init__(self):
        rospy.init_node("map_section_receiver")

        self.serial_port = None
        self.serial_fault = False
        self.receive_buffer = bytearray()
        self.lock = threading.Lock()
        self._last_ids = None

        self.id_to_coord = R1Config.ID_TO_COORD

        self.pub_token_id = rospy.Publisher(
            "/R1_grap_pos", Int32MultiArray, queue_size=10, latch=True)

        self._serial_init()

        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

        if self.serial_fault:
            threading.Thread(target=self._reconnect, daemon=True).start()

        rospy.on_shutdown(self._cleanup)
        rospy.loginfo(f"MapSecReceive ready on {Config.SERIAL_PORT} @ {Config.BAUD_RATE}")
        rospy.loginfo("  ID→coord mapping: %s",
                      {k: f"({v[0]:.2f},{v[1]:.2f})" for k, v in self.id_to_coord.items()})

    # ── Serial management ──

    def _serial_init(self):
        try:
            self.serial_port = serial.Serial(
                Config.SERIAL_PORT, Config.BAUD_RATE, timeout=Config.TIMEOUT)
            self.serial_fault = False
            rospy.loginfo(f"Serial {Config.SERIAL_PORT} opened")
            return True
        except serial.SerialException as e:
            rospy.logerr(f"Serial open failed: {e}")
            self.serial_fault = True
            return False

    def _reconnect(self):
        delay = Config.RECONNECT_BASE_DELAY
        while not rospy.is_shutdown():
            if self.serial_port and self.serial_port.is_open:
                self.serial_fault = False
                return True
            rospy.logwarn(f"Reconnecting {Config.SERIAL_PORT}... ({delay:.1f}s)")
            time.sleep(delay)
            if self._serial_init():
                rospy.loginfo("Reconnected")
                return True
            delay = min(delay * 2, Config.RECONNECT_MAX_DELAY)
        return False

    def _cleanup(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("Serial closed")

    # ── Receive loop ──

    def _receive_loop(self):
        HEADER = Config.HEADER
        TAIL = Config.TAIL
        FRAME_LEN = Config.FRAME_LEN
        PERIOD = 1.0 / Config.RECEIVE_HZ

        while not rospy.is_shutdown():
            if self.serial_fault or not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.5)
                continue

            try:
                waiting = self.serial_port.in_waiting
                if waiting > 0:
                    raw = self.serial_port.read(waiting)
                    self.receive_buffer.extend(raw)

                buf = self.receive_buffer
                while len(buf) >= FRAME_LEN:
                    head_pos = buf.find(bytes([HEADER]))
                    if head_pos < 0:
                        buf.clear()
                        break
                    if head_pos > 0:
                        rospy.logdebug(f"Discarding {head_pos} junk bytes")
                        del buf[:head_pos]
                    if len(buf) < FRAME_LEN:
                        break
                    if buf[FRAME_LEN - 1] == TAIL:
                        self._parse_frame(buf[:FRAME_LEN])
                        del buf[:FRAME_LEN]
                    else:
                        rospy.logdebug(f"Tail mismatch, dropping 0x{buf[0]:02x}")
                        del buf[0]

                if len(buf) > Config.BUFFER_MAX:
                    rospy.logwarn(f"Buffer overflow ({len(buf)} bytes), clearing")
                    buf.clear()

            except serial.SerialException as e:
                rospy.logwarn(f"Serial read error: {e}")
                self.serial_fault = True

            time.sleep(PERIOD)

    # ── Frame parsing ──

    def _parse_frame(self, frame):
        """Parse: [HEADER(1) | ID1(1) | ID2(1) | TAIL(1)]

        0x00 = sentinel (no ID), supplemented later by GraphMapperNode.
        """
        id1 = frame[1]
        id2 = frame[2]

        current_ids = (id1, id2)
        if current_ids == self._last_ids:
            rospy.logdebug(f"Duplicate IDs [{id1}, {id2}], ignored")
            return
        self._last_ids = current_ids

        # Validate non-zero IDs
        for idx, rid in enumerate([id1, id2], start=1):
            if rid != 0 and rid not in self.id_to_coord:
                rospy.logerr(f"Unknown ID{idx}={rid}, available: {list(self.id_to_coord.keys())}")
                return

        # Log coordinate info
        parts = []
        for rid in [id1, id2]:
            if rid == 0:
                parts.append(f"ID0 (sentinel)")
            else:
                c = self.id_to_coord[rid]
                parts.append(f"ID{rid} → ({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})")
        rospy.loginfo(f"Frame received: IDs=[{id1}, {id2}]")
        for p in parts:
            rospy.loginfo(f"  {p}")

        n_confirm = R1Config.N_CONFIRM
        msg = Int32MultiArray(data=[id1, id2])
        for i in range(n_confirm):
            self.pub_token_id.publish(msg)
            if i < n_confirm - 1:
                time.sleep(0.05)
        rospy.loginfo(f"Published /R1_grap_pos: [{id1}, {id2}] x{n_confirm}")


if __name__ == "__main__":
    try:
        MapSectionReceiver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
