#!/usr/bin/env python3
"""ESP32-P4 native-USB flash without BOOT/RESET, with EXPLICIT ports.

1. open the firmware CDC port (--app-port) at 1200 baud, then close it
2. wait for the ROM loader port (--rom-port) to appear
3. run `idf.py -p <rom-port> flash`
4. wait for the application CDC port (--app-port) to come back

No VID/PID is assumed: both ports are given by the user (they can change
between plug-ins; check Device Manager). Nothing is flashed on a timeout.
"""
import argparse, os, subprocess, sys, time
import serial
from serial.tools import list_ports


def present():
    return {p.device.upper() for p in list_ports.comports()}


def wait_port(name, timeout, what):
    end = time.time() + timeout
    while time.time() < end:
        if name.upper() in present():
            time.sleep(0.5)  # let enumeration settle
            return
        time.sleep(0.25)
    sys.exit(f"Timeout ({timeout:g}s) waiting for {what} {name}. Present ports: {sorted(present())}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--app-port", required=True, help="current firmware CDC port, e.g. COM37")
    ap.add_argument("--rom-port", required=True, help="ROM loader port expected after the reset, e.g. COM36")
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--project-dir", default=".")
    ap.add_argument("--no-flash", action="store_true", help="only trigger the ROM loader")
    a = ap.parse_args()

    if a.app_port.upper() == a.rom_port.upper():
        sys.exit("--app-port and --rom-port must differ")
    ports = present()
    if a.rom_port.upper() in ports and a.app_port.upper() not in ports:
        print(f"ROM loader port {a.rom_port} already present, {a.app_port} absent: skipping 1200 bps")
    else:
        if a.app_port.upper() not in ports:
            sys.exit(f"{a.app_port} not present. Present ports: {sorted(ports)}")

        print(f"1200 bps on {a.app_port}")
        with serial.Serial(a.app_port, 1200) as s:
            time.sleep(0.1)
        time.sleep(0.3)

        wait_port(a.rom_port, a.timeout, "ROM loader port")
        print(f"ROM loader port {a.rom_port} present")
    if a.no_flash:
        return

    idf_py = os.path.join(os.environ.get("IDF_PATH", ""), "tools", "idf.py")
    if not os.path.isfile(idf_py):
        sys.exit("IDF_PATH not set: run from an ESP-IDF environment")
    cmd = [sys.executable, idf_py, "-C", a.project_dir, "-p", a.rom_port, "flash"]
    print("Running:", " ".join(cmd))
    r = subprocess.run(cmd)
    if r.returncode:
        sys.exit(r.returncode)

    print("Waiting for application CDC port...")
    wait_port(a.app_port, a.timeout * 2, "application CDC port")
    print(f"Firmware back on {a.app_port}")


if __name__ == "__main__":
    main()
