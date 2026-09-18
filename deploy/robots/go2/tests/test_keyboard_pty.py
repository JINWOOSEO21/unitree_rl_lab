#!/usr/bin/env python3
"""Exercise --keyboard-check through a real foreground pseudo-terminal."""
import os
import pty
import select
import signal
import shutil
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
        signal.signal(signal.SIGPIPE, signal.SIG_DFL)
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


def broken_logging_pipe(stop_signal):
    """A lost tee reader must not kill the controller before controlled shutdown.

    Use the real binary's no-DDS mode; existing shutdown tests cover the descent.
    Python ignores SIGPIPE, so explicitly restore the shell's default in the child.
    """
    read_fd, write_fd = os.pipe()
    pid, master = pty.fork()
    if pid == 0:
        os.close(read_fd)
        os.dup2(write_fd, 1)
        os.dup2(write_fd, 2)
        os.close(write_fd)
        signal.signal(signal.SIGPIPE, signal.SIG_DFL)
        os.execv(shutil.which("stdbuf"),
                 ["stdbuf", "-oL", sys.argv[1], "--keyboard-check"])

    os.close(write_fd)
    initial_flags = termios.tcgetattr(master)[3]
    reaped = False
    try:
        read_until(read_fd, b"NO DDS / NO MOTOR OUTPUT")
        os.write(master, b"i")
        read_until(read_fd, b"speed=+0.25 turn=+0.00")
        # tee has exited: both stdout and stderr now have no pipe reader.
        os.close(read_fd)
        read_fd = None
        os.write(master, b"h")
        time.sleep(0.15)
        done, status = os.waitpid(pid, os.WNOHANG)
        reaped = bool(done)
        assert not done, f"closed logging pipe killed controller: {os.waitstatus_to_exitcode(status)}"
        if stop_signal == signal.SIGINT:
            os.write(master, b"\x03")  # Real terminal Ctrl+C.
        else:
            os.kill(pid, stop_signal)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            done, status = os.waitpid(pid, os.WNOHANG)
            if done:
                reaped = True
                break
            time.sleep(0.01)
        assert reaped, "termination signal did not reach controller"
        assert os.waitstatus_to_exitcode(status) == 0, status
        mask = termios.ICANON | termios.ECHO
        assert initial_flags & mask == termios.tcgetattr(master)[3] & mask
    finally:
        if not reaped:
            os.kill(pid, signal.SIGKILL)
            os.waitpid(pid, 0)
        if read_fd is not None:
            os.close(read_fd)
        os.close(master)


if __name__ == "__main__":
    for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        main(stop_signal)
        broken_logging_pipe(stop_signal)
