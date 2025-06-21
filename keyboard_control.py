import sys
import termios
import time
import tty

from trossen import Trossen, DXL_ID, DXL_CONSTANTS

DEVICE_NAME = '/dev/tty.usbserial-FT6Z5MSQ'
BAUD_RATE = 1000000

# Adjustable increment size for each key press
INCREMENT_SIZE = 50

# Map keys to joint indices and direction (+1 or -1)
KEY_BINDINGS = {
    'q': [(DXL_ID.BASE_ROTATE, +1)],
    'w': [(DXL_ID.BASE_ROTATE, -1)],
    'a': [(DXL_ID.BASE_PIVOT_1, +1), (DXL_ID.BASE_PIVOT_2, +1)],
    's': [(DXL_ID.BASE_PIVOT_1, -1), (DXL_ID.BASE_PIVOT_2, -1)],
    'e': [(DXL_ID.ELBOW_PIVOT_1, +1), (DXL_ID.ELBOW_PIVOT_2, +1)],
    'r': [(DXL_ID.ELBOW_PIVOT_1, -1), (DXL_ID.ELBOW_PIVOT_2, -1)],
    'y': [(DXL_ID.ELBOW_ROTATE, +1)],
    'u': [(DXL_ID.ELBOW_ROTATE, -1)],
    'h': [(DXL_ID.WRIST_PIVOT, +1)],
    'j': [(DXL_ID.WRIST_PIVOT, -1)],
    'i': [(DXL_ID.WRIST_ROTATE, +1)],
    'o': [(DXL_ID.WRIST_ROTATE, -1)],
    'k': [(DXL_ID.END_EFFECTOR, +1)],
    'l': [(DXL_ID.END_EFFECTOR, -1)],
}

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def main():
    trossen = Trossen(port=DEVICE_NAME, baudrate=BAUD_RATE)

    joint_names = {member.value: member.name for member in DXL_ID}

    print("Control arm joints with keys:")
    for key, bindings in KEY_BINDINGS.items():
        for joint_enum, direction in bindings:
            action = "increase" if direction > 0 else "decrease"
            print(f"  {key}: {action} {joint_enum.name}")

    print("Press 'x' to exit.")

    # Initialize current positions by reading actual robot joint positions
    positions, success = trossen.positions()
    if not success:
        print("Error: Failed to read current joint positions from hardware.")
        exit(1)
    else:
        current_positions = {idx: pos for idx, pos in enumerate(positions)}

    try:
        while True:
            key = get_key()
            if key == 'x':
                print("Exiting...")
                break

            if key in KEY_BINDINGS:
                for joint_enum, direction in KEY_BINDINGS[key]:
                    joint_id = joint_enum.value
                    try:
                        idx = trossen.all_ids.index(joint_id)
                    except ValueError:
                        print(f"Joint ID {joint_id} not found in trossen.all_ids.")
                        continue

                    increment = direction * INCREMENT_SIZE
                    current = current_positions[idx]
                    min_pos = trossen.min_position_limits[idx]
                    max_pos = trossen.max_position_limits[idx]
                    new_pos = clamp(current + increment, min_pos, max_pos)

                    if new_pos != current:
                        print(f"Setting {joint_enum.name} to {new_pos}")
                        trossen.set_joint_position(joint_id, new_pos)
                        current_positions[idx] = new_pos
                    else:
                        print(f"{joint_enum.name} position at limit ({new_pos})")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        trossen.close()

if __name__ == "__main__":
    main()