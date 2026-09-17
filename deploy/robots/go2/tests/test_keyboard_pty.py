#!/usr/bin/env python3
"""Exercise --keyboard-check through a real foreground pseudo-terminal."""
import os
import pty
import select
import signal
import sys
import termios
import time


def read_until(fd, needle, timeout=3.0):
    data = b""
    deadline = time.monotonic() + timeout
    while needle not in data and time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.1)
        if ready:
            try:
                data += os.read(fd, 4096)
            except OSError:
                break
    if needle not in data:
        raise AssertionError(f"missing {needle!r} in {data!r}")
    return data


def main(stop_signal=signal.SIGINT):
    pid, master = pty.fork()
    if pid == 0:
        os.execv(sys.argv[1], [sys.argv[1], "--keyboard-check"])

    initial_flags = termios.tcgetattr(master)[3]
    output = read_until(master, b"NO DDS / NO MOTOR OUTPUT")
    os.write(master, b"\x1b[3~")  # Delete must not become the bound digit '3'.
    time.sleep(0.05)
    os.write(master, b"i")
    output += read_until(master, b"speed=+0.25 turn=+0.00")
    time.sleep(0.15)  # idle raw reads must not clear sticky axes
    os.write(master, b"q")
    output += read_until(master, b"target_heading=+10.0 deg (base +x=0, left=+)")
    time.sleep(0.15)
    os.write(master, b"q")
    output += read_until(master, b"target_heading=+20.0 deg")
    os.write(master, b"e")
    output += read_until(master, b"target_heading=+10.0 deg")
    os.write(master, b"w")
    output += read_until(master, b"[key] w target_heading=+0.0 deg")
    os.write(master, b"k")
    output += read_until(master, b"[key] k speed=+0.00")
    os.kill(pid, stop_signal)
    _, status = os.waitpid(pid, 0)
    final_flags = termios.tcgetattr(master)[3]
    os.close(master)
    assert os.waitstatus_to_exitcode(status) == 0, output.decode(errors="replace")
    assert b"[key] 3" not in output, output.decode(errors="replace")
    mask = termios.ICANON | termios.ECHO
    assert initial_flags & mask == final_flags & mask, (initial_flags, final_flags)


if __name__ == "__main__":
    for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        main(stop_signal)
