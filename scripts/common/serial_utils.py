"""Serial communication utilities shared across modules.

Frame building, padding, sending — used by both a_star (R2 commands, R1 IDs)
and r1_graph (TGT handshake, waypoint execution).
"""

import struct
import serial


def load_serial_config(config_path="/home/zjy/R1_control/config/a_star.yaml",
                       section="serial"):
    """Load serial config section from a YAML file."""
    import yaml
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f) or {}
    return cfg.get(section, {})


def pad_buffer(buf, fixed_len=25):
    """Pad bytearray to fixed_len, filling with last byte."""
    buf = bytearray(buf)
    if len(buf) >= fixed_len:
        return buf[:fixed_len]
    pad_byte = buf[-1] if buf else 0
    buf.extend(bytearray([pad_byte] * (fixed_len - len(buf))))
    return buf


def frame_buffer(buf, header=0x66, tail=0x99):
    """Wrap buffer with header and tail bytes."""
    return bytearray([header]) + bytearray(buf) + bytearray([tail])


def print_binary(buf):
    """Print hex and decimal dump of a bytearray."""
    print(f"Length: {len(buf)} bytes")
    print(f"Hex: {' '.join(f'{b:02X}' for b in buf)}")
    print(f"Dec: {list(buf)}")


def send_via_serial(buf, port, baudrate=115200, timeout=1.0, verbose=True):
    """Send bytearray over serial port.  Logs and returns on failure."""
    ser = None
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        ser.write(bytes(buf))
        if verbose:
            print(f"[Serial] {port} ← {len(buf)} bytes: "
                  f"{' '.join(f'{b:02X}' for b in buf)}")
    except (serial.SerialException, OSError) as e:
        print(f"[Serial] {port} unavailable: {e}")
        print(f"[Serial] data: {' '.join(f'{b:02X}' for b in buf)}")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
