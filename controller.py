# controller.py

from kinematics import forward_kinematics, inverse_kinematics

STEP = 10  # pixels per move

def get_direction_input():
    valid_inputs = {'w', 'a', 's', 'd'}
    while True:
        key = input("Enter direction (W/A/S/D to move, Q to quit): ").strip().lower()
        if key in valid_inputs or key == 'q':
            return key
        print("Invalid input. Use W/A/S/D or Q.")

def main():
    print("=== 3-Link Arm Controller ===")
    angles = input("Enter initial angles for joints (comma-separated degrees, e.g., 30, 45, 60): ")
    angles = list(map(float, angles.split(',')))

    state = forward_kinematics(angles)
    end_effector = state['end_effector']
    print("\nInitial configuration:")
    print(f"Joint 1 position: {state['joint1']}")
    print(f"Joint 2 position: {state['joint2']}")
    print(f"End Effector position: {end_effector}")

    current_angles = angles
    current_position = end_effector

    while True:
        key = get_direction_input()
        if key == 'q':
            print("Exiting controller.")
            break

        dx, dy = 0, 0
        if key == 'w':
            dy = STEP
        elif key == 's':
            dy = -STEP
        elif key == 'a':
            dx = -STEP
        elif key == 'd':
            dx = STEP

        new_target = [current_position[0] + dx, current_position[1] + dy]
        result = inverse_kinematics(new_target, old_angles=current_angles, old_target=current_position)
        new_angles = result["angles"]

        new_state = forward_kinematics(new_angles)
        new_end_effector = new_state["end_effector"]

        print(f"\nCommand: {key.upper()}")
        print(f"New angles: {new_angles}")
        print(f"New end effector position: {new_end_effector}")

        current_angles = new_angles
        current_position = new_end_effector

if __name__ == '__main__':
    main()
