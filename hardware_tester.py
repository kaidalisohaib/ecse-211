#!/usr/bin/env python3
"""
Interactive Hardware Diagnostic & Test Utility for ECSE 211 (BrickPi3)
Allows quick interactive testing of:
  - Motors (Ports A, B, C, D — single or dual, DPS speed, Power %, relative degrees, float & encoder streaming)
  - EV3 Color Sensors (Ports 1-4 — single or dual, Reflected Red, RGB Components, Color ID, Ambient)
  - EV3 Ultrasonic Sensors (Ports 1-4 — cm, inches, listen mode)
  - EV3 Gyro Sensors (Ports 1-4 — angle & rotational speed)
  - Touch Sensors (Ports 1-4 — pressed/released state)
"""

import os
import sys
import time

# Ensure 'utils.brick' can be imported whether run from project root or inside a lab folder
_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (_HERE, os.path.join(_HERE, "lab3"), os.path.join(_HERE, "lab2"), os.path.join(_HERE, "lab1")):
    if os.path.isdir(os.path.join(_candidate, "utils")) and _candidate not in sys.path:
        sys.path.insert(0, _candidate)
        break

from utils.brick import (
    Motor,
    EV3ColorSensor,
    EV3UltrasonicSensor,
    EV3GyroSensor,
    TouchSensor,
    Sensor,
    reset_brick,
)

COLOR_ID_NAMES = {
    0: "Unknown",
    1: "Black",
    2: "Blue",
    3: "Green",
    4: "Yellow",
    5: "Red",
    6: "White",
    7: "Orange",
}


def wait_sensor_ready(sensor: Sensor, port_num: int, timeout: float = 6.0) -> bool:
    """
    Waits for a sensor to report VALID_DATA with a timeout so an unplugged
    port does not freeze the program indefinitely.
    """
    print(f"Initializing {type(sensor).__name__} on Port S{port_num}...", end=" ", flush=True)
    start = time.time()
    last_status = "UNKNOWN"
    while time.time() - start < timeout:
        try:
            last_status = sensor.get_status()
            if last_status == Sensor.Status.VALID_DATA:
                print("READY!")
                return True
        except Exception as e:
            last_status = f"ERR ({e})"
        time.sleep(0.05)

    print(f"\n[WARNING] Port S{port_num} timed out after {timeout:.1f}s (Last status: {last_status}).")
    print("          Check that the cable is firmly clicked into the right port and battery is ON.")
    return False


def prompt_sensor_ports(allow_multiple: bool = False) -> list[int]:
    """Prompts the user for 1 (or multiple) sensor port numbers in 1..4."""
    while True:
        prompt_str = (
            "Enter sensor port(s) [1-4, or e.g. '2,4' for two sensors] (or 'b' to go back): "
            if allow_multiple
            else "Enter sensor port [1-4] (or 'b' to go back): "
        )
        raw = input(prompt_str).strip().lower()
        if raw in ("b", "back", "q"):
            return []

        cleaned = raw.replace(",", " ").split()
        if not allow_multiple and len(cleaned) > 1:
            print("Please enter a single sensor port (1, 2, 3, or 4).")
            continue

        ports = []
        valid = True
        for token in cleaned:
            if token in ("1", "2", "3", "4"):
                p = int(token)
                if p not in ports:
                    ports.append(p)
            else:
                valid = False
                break

        if valid and ports:
            return ports
        print("Invalid input. Valid sensor ports are 1, 2, 3, 4.")


def prompt_motor_ports() -> list[str]:
    """Prompts the user for 1 or more motor ports in A, B, C, D."""
    while True:
        raw = input("Enter motor port(s) [A, B, C, D, or e.g. 'B,C' for both] (or 'b' to go back): ").strip().upper()
        if raw in ("B_BACK", "BACK", "Q"):
            return []
        if raw.lower() == "b":
            # 'B' is a valid motor port! Confirm if they meant Motor B or back:
            return ["B"]

        # Allow formats like "B,C", "B C", or "BC"
        tokens = raw.replace(",", " ").split()
        if len(tokens) == 1 and len(tokens[0]) > 1 and all(ch in "ABCD" for ch in tokens[0]):
            tokens = list(tokens[0])

        ports = []
        valid = True
        for t in tokens:
            if t in ("A", "B", "C", "D"):
                if t not in ports:
                    ports.append(t)
            else:
                valid = False
                break

        if valid and ports:
            return ports
        print("Invalid motor port. Enter A, B, C, D (or e.g. 'B,C' or 'BACK').")


# ==================== 1. MOTOR TESTER ====================
def test_motors():
    print("\n--- MOTOR TESTER ---")
    print("Tip: Type 'BACK' to return to main menu, or enter port(s) like 'B', 'C', or 'B,C'.")
    ports = prompt_motor_ports()
    if not ports:
        return

    motors = {p: Motor(p) for p in ports}
    port_label = ", ".join(f"Motor {p}" for p in ports)
    print(f"\nConfigured {port_label}.")

    try:
        while True:
            # Read current status snapshot
            status_parts = []
            for p, m in motors.items():
                enc = m.get_encoder()
                spd = m.get_speed()
                pwr = m.get_power()
                status_parts.append(f"Port {p}: [Enc={enc} deg | Spd={spd} dps | Pwr={pwr}%]")

            print("\n" + "  ||  ".join(status_parts))
            print("Commands:")
            print("  <number>       : Set speed in DPS (e.g. '150', '-150', '0')"
                  + (" or two numbers e.g. '150 -150'" if len(ports) == 2 else ""))
            print("  p <power>      : Set power % from -100 to 100 (e.g. 'p 50', 'p -60')")
            print("  deg <angle>    : Rotate relative degrees (e.g. 'deg 360', 'deg -90')")
            print("  f              : Float motor(s) and live-stream encoders (spin by hand)")
            print("  m              : Live-monitor encoder & speed while running")
            print("  z              : Zero / reset encoder(s) to 0")
            print("  s              : Stop motor(s) immediately")
            print("  b              : Stop motor(s) and go back to main menu")

            cmd = input("Motor cmd > ").strip().lower()
            if not cmd:
                continue

            if cmd in ("b", "back", "q", "quit"):
                break

            elif cmd in ("s", "stop", "0"):
                for m in motors.values():
                    m.set_power(0)
                print("-> Stopped motor(s).")

            elif cmd in ("z", "zero", "reset"):
                for m in motors.values():
                    m.reset_encoder()
                print("-> Reset encoder(s) to 0.")

            elif cmd in ("f", "float"):
                for m in motors.values():
                    m.float_motor()
                print("-> Motor(s) floating. Spin wheels by hand! Press Ctrl+C to stop streaming.")
                try:
                    while True:
                        line = " | ".join(
                            f"Port {p}: Enc={m.get_encoder():6.1f} deg" if m.get_encoder() is not None else f"Port {p}: None"
                            for p, m in motors.items()
                        )
                        print(f"\r{line}    ", end="", flush=True)
                        time.sleep(0.05)
                except KeyboardInterrupt:
                    print("\nStopped encoder stream.")

            elif cmd in ("m", "monitor"):
                print("-> Live monitoring motor(s). Press Ctrl+C to return to motor prompt.")
                try:
                    while True:
                        line = " | ".join(
                            f"Port {p}: Enc={m.get_encoder()} deg, Spd={m.get_speed()} dps, Pwr={m.get_power()}%"
                            for p, m in motors.items()
                        )
                        print(f"\r{line}      ", end="", flush=True)
                        time.sleep(0.05)
                except KeyboardInterrupt:
                    print("\nStopped live monitor.")

            elif cmd.startswith("p "):
                parts = cmd.split()[1:]
                try:
                    vals = [float(x) for x in parts]
                    if len(vals) == 1:
                        for m in motors.values():
                            m.set_power(vals[0])
                        print(f"-> Set power to {vals[0]:.0f}% on {port_label}.")
                    elif len(vals) == len(ports):
                        for p, val in zip(ports, vals):
                            motors[p].set_power(val)
                        print("-> Set individual powers: " + ", ".join(f"{p}={v:.0f}%" for p, v in zip(ports, vals)))
                    else:
                        print(f"Provide 1 value or {len(ports)} values.")
                except ValueError:
                    print("Invalid power value. Example: 'p 50' or 'p -50'.")

            elif cmd.startswith("deg "):
                parts = cmd.split()[1:]
                try:
                    deg_val = float(parts[0])
                    for m in motors.values():
                        m.set_limits(power=60, dps=200)
                        m.set_position_relative(int(deg_val))
                    print(f"-> Rotating {deg_val:+.0f} degrees relative...")
                except (ValueError, IndexError):
                    print("Invalid degree value. Example: 'deg 360' or 'deg -90'.")

            else:
                # Parse direct DPS number(s), e.g. "150" or "-150" or "150 -150"
                parts = cmd.replace(",", " ").split()
                try:
                    vals = [float(x) for x in parts]
                    if len(vals) == 1:
                        for m in motors.values():
                            m.set_dps(vals[0])
                        print(f"-> Set speed to {vals[0]:.0f} DPS on {port_label}.")
                    elif len(vals) == len(ports):
                        for p, val in zip(ports, vals):
                            motors[p].set_dps(val)
                        print("-> Set individual speeds: " + ", ".join(f"{p}={v:.0f} DPS" for p, v in zip(ports, vals)))
                    else:
                        print(f"Unrecognized command. Enter 1 speed or {len(ports)} speeds.")
                except ValueError:
                    print("Unrecognized command. Enter a number for DPS (e.g. 150, -150) or 's' to stop.")
    finally:
        for m in motors.values():
            m.set_power(0)
        print("Motors stopped.")


# ==================== 2. COLOR SENSOR TESTER ====================
def test_color_sensor():
    print("\n--- EV3 COLOR SENSOR TESTER ---")
    ports = prompt_sensor_ports(allow_multiple=True)
    if not ports:
        return

    while True:
        print("\nSelect Color Sensor Mode:")
        print("  1 / red     : Reflected Red Light (0-100, used for black line detection)")
        print("  2 / rgb     : RGB Components [R, G, B]")
        print("  3 / id      : Color ID & Name (Black, White, Red, Blue, Green, Yellow...)")
        print("  4 / ambient : Ambient Light Intensity (0-100)")
        print("  b           : Back to main menu")

        mode_in = input("Mode > ").strip().lower()
        if mode_in in ("b", "back", "q"):
            break

        mode_map = {
            "1": "red", "red": "red",
            "2": "component", "rgb": "component", "component": "component",
            "3": "id", "id": "id",
            "4": "ambient", "ambient": "ambient",
        }
        if mode_in not in mode_map:
            print("Invalid mode selection.")
            continue

        chosen_mode = mode_map[mode_in]
        threshold = 30.0
        if chosen_mode == "red":
            th_in = input("Enter black line threshold [press Enter for default 30]: ").strip()
            if th_in:
                try:
                    threshold = float(th_in)
                except ValueError:
                    pass

        sensors = {}
        for p in ports:
            s = EV3ColorSensor(p, mode=chosen_mode)
            wait_sensor_ready(s, p)
            sensors[p] = s

        print(f"\nStreaming Color Sensor(s) in '{chosen_mode.upper()}' mode. Press Ctrl+C to stop stream...\n")
        try:
            while True:
                out_chunks = []
                for p, s in sensors.items():
                    if chosen_mode == "red":
                        val = s.get_red()
                        tag = "BLACK LINE" if (val is not None and val < threshold) else "WHITE/TILE"
                        val_str = f"{val:5.1f}" if isinstance(val, (int, float)) else str(val)
                        out_chunks.append(f"S{p}: Red={val_str} [{tag}]")
                    elif chosen_mode == "component":
                        rgb = s.get_rgb()
                        out_chunks.append(f"S{p}: RGB={rgb}")
                    elif chosen_mode == "id":
                        code = s.get_value()
                        name = COLOR_ID_NAMES.get(code, f"Code({code})")
                        out_chunks.append(f"S{p}: {name} (id={code})")
                    elif chosen_mode == "ambient":
                        amb = s.get_ambient()
                        out_chunks.append(f"S{p}: Ambient={amb}")

                print("  |  ".join(out_chunks))
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped sensor stream.")


# ==================== 3. ULTRASONIC SENSOR TESTER ====================
def test_ultrasonic_sensor():
    print("\n--- EV3 ULTRASONIC SENSOR TESTER ---")
    ports = prompt_sensor_ports(allow_multiple=False)
    if not ports:
        return
    port = ports[0]

    while True:
        print("\nSelect Ultrasonic Sensor Mode:")
        print("  1 / cm     : Distance in Centimeters (cm)")
        print("  2 / in     : Distance in Inches (in)")
        print("  3 / listen : Listen for other ultrasonic sensors (0/1)")
        print("  b          : Back to main menu")

        mode_in = input("Mode > ").strip().lower()
        if mode_in in ("b", "back", "q"):
            break

        mode_map = {
            "1": "cm", "cm": "cm",
            "2": "in", "in": "in",
            "3": "listen", "listen": "listen",
        }
        if mode_in not in mode_map:
            print("Invalid mode.")
            continue

        chosen_mode = mode_map[mode_in]
        us = EV3UltrasonicSensor(port, mode=chosen_mode)
        wait_sensor_ready(us, port)

        print(f"\nStreaming Ultrasonic Sensor (Port S{port}, mode={chosen_mode}). Press Ctrl+C to stop...\n")
        try:
            while True:
                if chosen_mode == "cm":
                    val = us.get_cm()
                    if isinstance(val, (int, float)):
                        bars = int(min(max(val, 0), 100) // 2.5)
                        bar_str = "#" * bars
                        print(f"S{port} Distance: {val:6.1f} cm  |{bar_str}")
                    else:
                        print(f"S{port} Distance: {val}")
                elif chosen_mode == "in":
                    val = us.get_inches()
                    print(f"S{port} Distance: {val} in")
                elif chosen_mode == "listen":
                    detected = us.detects_other_us_sensor()
                    print(f"S{port} Other US Detected: {detected}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped ultrasonic stream.")


# ==================== 4. GYRO SENSOR TESTER ====================
def test_gyro_sensor():
    print("\n--- EV3 GYRO SENSOR TESTER ---")
    ports = prompt_sensor_ports(allow_multiple=False)
    if not ports:
        return
    port = ports[0]

    gyro = EV3GyroSensor(port, mode="both")
    wait_sensor_ready(gyro, port)

    print(f"\nStreaming Gyro Sensor (Port S{port}). Press Ctrl+C to stop...\n")
    try:
        while True:
            val = gyro.get_both_measure()
            if isinstance(val, list) and len(val) >= 2:
                print(f"S{port} Gyro -> Angle: {val[0]:6.1f} deg  |  Rate: {val[1]:6.1f} deg/s")
            else:
                print(f"S{port} Gyro -> {val}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped gyro stream.")


# ==================== 5. TOUCH SENSOR TESTER ====================
def test_touch_sensor():
    print("\n--- TOUCH SENSOR TESTER ---")
    ports = prompt_sensor_ports(allow_multiple=True)
    if not ports:
        return

    sensors = {}
    for p in ports:
        t = TouchSensor(p)
        wait_sensor_ready(t, p)
        sensors[p] = t

    print("\nStreaming Touch Sensor(s). Press Ctrl+C to stop...\n")
    try:
        while True:
            chunks = []
            for p, t in sensors.items():
                pressed = t.is_pressed()
                state_str = "PRESSED  [1]" if pressed else "RELEASED [0]"
                chunks.append(f"S{p}: {state_str}")
            print("  |  ".join(chunks))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped touch sensor stream.")


# ==================== MAIN MENU ====================
def main():
    print("==========================================================")
    print("       ECSE 211 INTERACTIVE HARDWARE TEST UTILITY         ")
    print("==========================================================")
    try:
        while True:
            print("\nSelect device to test:")
            print("  1 / m : Motor(s)             (Ports A, B, C, D)")
            print("  2 / c : EV3 Color Sensor(s)  (Ports 1, 2, 3, 4)")
            print("  3 / u : EV3 Ultrasonic       (Ports 1, 2, 3, 4)")
            print("  4 / g : EV3 Gyro Sensor      (Ports 1, 2, 3, 4)")
            print("  5 / t : Touch Sensor(s)      (Ports 1, 2, 3, 4)")
            print("  r     : Reset BrickPi hardware")
            print("  q     : Quit & Reset Brick")

            choice = input("\nSelect option > ").strip().lower()
            if choice in ("1", "m", "motor", "motors"):
                test_motors()
            elif choice in ("2", "c", "color"):
                test_color_sensor()
            elif choice in ("3", "u", "us", "ultrasonic"):
                test_ultrasonic_sensor()
            elif choice in ("4", "g", "gyro"):
                test_gyro_sensor()
            elif choice in ("5", "t", "touch"):
                test_touch_sensor()
            elif choice in ("r", "reset"):
                reset_brick()
                print("-> BrickPi hardware reset complete.")
            elif choice in ("q", "quit", "exit"):
                break
            else:
                print("Invalid option. Choose 1-5, 'r', or 'q'.")
    except (KeyboardInterrupt, SystemExit):
        print("\nExiting hardware tester...")
    finally:
        reset_brick()
        print("All motors stopped and BrickPi reset.")


if __name__ == "__main__":
    main()
