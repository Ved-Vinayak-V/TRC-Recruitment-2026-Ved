"""Task 1a - keyboard controlled robot.

Robot geometry (set in worlds/task1a.wbt):
    body   0.30 x 0.15 x 0.05 m   (l x b x h)
    wheels 0.03 m thick, 0.05 m radius, four of them, skid steer

Controls
    W / Up     forward
    S / Down   backward
    A / Left   turn left
    D / Right  turn right
    SPACE      brake (stop immediately)

W+A, W+D, S+A ... can be held together to drive along a curve.
Click once inside the 3D view so it has keyboard focus, otherwise
Webots sends the key presses to the editor instead of the robot.
"""

from controller import Robot, Keyboard

# ----------------------------------------------------------------------------
# tuning
# ----------------------------------------------------------------------------
WHEEL_RADIUS = 0.05          # m, must match the Cylinder radius in the world
CRUISE = 6.0                 # rad/s  -> 6.0 * 0.05 = 0.30 m/s top speed
TURN = 4.0                   # rad/s  added/subtracted per side when steering
ACCEL = 0.4                  # rad/s added per control step (soft start)

LEFT_WHEELS = ("wheel_fl", "wheel_rl")
RIGHT_WHEELS = ("wheel_fr", "wheel_rr")


def clamp(value, low, high):
    return max(low, min(high, value))


def ramp(current, target, step):
    """Move `current` towards `target` by at most `step`."""
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return target


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # --- motors -------------------------------------------------------------
    left_motors = []
    right_motors = []
    for name in LEFT_WHEELS:
        left_motors.append(robot.getDevice(name))
    for name in RIGHT_WHEELS:
        right_motors.append(robot.getDevice(name))

    for motor in left_motors + right_motors:
        # infinite target position == velocity control mode
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    # --- keyboard -----------------------------------------------------------
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    print("Task 1a ready.  Click in the 3D view, then:")
    print("   W forward | S backward | A left | D right | SPACE brake")

    left_speed = 0.0
    right_speed = 0.0
    last_label = None

    while robot.step(timestep) != -1:
        # Webots hands back one key per call, so drain the buffer to support
        # several keys being held down at the same time (e.g. W + D).
        keys = []
        key = keyboard.getKey()
        while key != -1:
            keys.append(key & 0xFFFF)   # mask off SHIFT/CTRL/ALT modifier bits
            key = keyboard.getKey()

        forward = ord("W") in keys or Keyboard.UP in keys
        backward = ord("S") in keys or Keyboard.DOWN in keys
        left = ord("A") in keys or Keyboard.LEFT in keys
        right = ord("D") in keys or Keyboard.RIGHT in keys
        brake = ord(" ") in keys

        if brake:
            # hard stop: skip the ramp entirely
            left_speed = 0.0
            right_speed = 0.0
            label = "BRAKE"
        else:
            target_left = 0.0
            target_right = 0.0

            if forward:
                target_left += CRUISE
                target_right += CRUISE
            if backward:
                target_left -= CRUISE
                target_right -= CRUISE
            if left:
                target_left -= TURN
                target_right += TURN
            if right:
                target_left += TURN
                target_right -= TURN

            target_left = clamp(target_left, -CRUISE, CRUISE)
            target_right = clamp(target_right, -CRUISE, CRUISE)

            left_speed = ramp(left_speed, target_left, ACCEL)
            right_speed = ramp(right_speed, target_right, ACCEL)

            parts = []
            if forward and not backward:
                parts.append("FORWARD")
            elif backward and not forward:
                parts.append("BACKWARD")
            if left and not right:
                parts.append("LEFT")
            elif right and not left:
                parts.append("RIGHT")
            label = "+".join(parts) if parts else "IDLE"

        for motor in left_motors:
            motor.setVelocity(left_speed)
        for motor in right_motors:
            motor.setVelocity(right_speed)

        if label != last_label:
            linear = (left_speed + right_speed) / 2.0 * WHEEL_RADIUS
            print("{:<14} L={:+.2f} R={:+.2f} rad/s  ({:+.2f} m/s)".format(
                label, left_speed, right_speed, linear))
            last_label = label


if __name__ == "__main__":
    main()
