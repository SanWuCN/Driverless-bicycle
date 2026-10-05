#!/usr/bin/env python3
"""Send a short UART7 probe and verify that diagnostic firmware echoes it."""

import argparse
import os
import select
import termios
import time


def configure_115200(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default="/dev/cu.usbserial-1140")
    args = parser.parse_args()

    probe = b"BIKE_UART7_TEST\r\n"
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_115200(fd)
        os.write(fd, probe)
        received = bytearray()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(received) < len(probe):
            readable, _, _ = select.select([fd], [], [], 0.2)
            if readable:
                received.extend(os.read(fd, 4096))
    finally:
        os.close(fd)

    if bytes(received) == probe:
        print(f"PASS: {args.port} echoed {len(probe)} bytes at 115200 8N1")
        return 0

    print(f"FAIL: expected {probe!r}, received {bytes(received)!r}")
    print("Flash the UART7 diagnostic build and verify TX/RX are crossed plus GND is common.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
