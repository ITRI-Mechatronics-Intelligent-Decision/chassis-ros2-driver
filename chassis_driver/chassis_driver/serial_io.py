"""
Handle RS485 serial communication with the VCU.

This includes packet encoding, decoding, and background I/O.
"""

import struct
import threading
import time

import serial

START_BYTE = 0x0A
END_BYTE = 0xFF

TX_LENGTH = 11
RX_LENGTH = 21

# '>' 大端序
# B  起始位 (uint8)
# h  馬達1 目標轉速 (int16)
# h  馬達2 目標轉速 (int16)
# B  Alarm 清除 (uint8)
# 4s 保留 (4 bytes)
# B  結束位 (uint8)
TX_STRUCT = struct.Struct(">BhhB4sB")

# '>' 大端序
# B  起始位 (uint8)
# h  馬達1 實際轉速 (int16)   H 馬達1 霍爾值 (uint16)   B 馬達1 Alarm (uint8)
# h  馬達2 實際轉速 (int16)   H 馬達2 霍爾值 (uint16)   B 馬達2 Alarm (uint8)
# H  電池電壓原始值 (uint16)  h 電池電流原始值 (int16)
# B  SoC (uint8)             B SoH (uint8)
# 2s 保留 (2 bytes)
# B  通訊狀態計數 (uint8)
# B  結束位 (uint8)
RX_STRUCT = struct.Struct(">BhHBhHBHhBB2sBB")


class ChassisSerial:

    def __init__(self, port: str, baudrate: int, timeout: float, send_interval: float):
        self._send_interval = send_interval

        self._conn = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

        self._lock = threading.Lock()
        self._target_left_rpm = 0
        self._target_right_rpm = 0
        self._clear_alarm_flag = False
        self._latest_state = None

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()
        self._conn.close()

    def set_target_rpm(self, left_rpm: int, right_rpm: int, clear_alarm: bool = False):
        with self._lock:
            self._target_left_rpm = left_rpm
            self._target_right_rpm = right_rpm
            self._clear_alarm_flag = clear_alarm

    def get_latest_state(self) -> dict | None:
        """Return the most recent decoded RX state or None if no valid packet has been received."""
        with self._lock:
            return self._latest_state

    def _run(self):
        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            with self._lock:
                left_rpm = self._target_left_rpm
                right_rpm = self._target_right_rpm
                clear_alarm = self._clear_alarm_flag
                self._clear_alarm_flag = False  # 單次觸發，送出後歸零

            tx_packet = self._encode_tx(left_rpm, right_rpm, clear_alarm)
            self._conn.write(tx_packet)

            rx_bytes = self._conn.read(RX_LENGTH)
            state = self._decode_rx(rx_bytes)
            if state is not None:
                with self._lock:
                    self._latest_state = state

            elapsed = time.monotonic() - loop_start
            sleep_time = self._send_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    @staticmethod
    def _encode_tx(left_rpm: int, right_rpm: int, clear_alarm: bool) -> bytes:
        return TX_STRUCT.pack(
            START_BYTE,
            left_rpm,
            right_rpm,
            0x01 if clear_alarm else 0x00,
            b"\x00\x00\x00\x00",
            END_BYTE,
        )

    @staticmethod
    def _decode_rx(raw: bytes) -> dict | None:
        if len(raw) != RX_LENGTH:
            return None

        unpacked = RX_STRUCT.unpack(raw)
        (start, m1_rpm, m1_hall, m1_alarm,
         m2_rpm, m2_hall, m2_alarm,
         voltage_raw, current_raw, soc, soh,
         _reserved, seq_count, end) = unpacked

        if start != START_BYTE or end != END_BYTE:
            return None

        return {
            "left_rpm": m1_rpm,
            "left_hall": m1_hall,
            "left_alarm": bool(m1_alarm),
            "right_rpm": m2_rpm,
            "right_hall": m2_hall,
            "right_alarm": bool(m2_alarm),
            "battery_voltage": voltage_raw * 0.001,
            "battery_current": current_raw * 0.001,
            "battery_soc": soc,
            "battery_soh": soh,
            "seq_count": seq_count,
        }
