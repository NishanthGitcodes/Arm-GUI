import math

L = 100  # Length of each arm
MAX_REACH = 3 * L
ANGLE_TOLERANCE = 0.5  # degrees

def to_radians(deg):
    return deg * math.pi / 180

def to_degrees(rad):
    return rad * 180 / math.pi

def approx_equal(a, b, tol=ANGLE_TOLERANCE):
    return abs(a - b) <= tol

def forward_kinematics(angles):
    """Compute positions of joints and end effector based on joint angles."""
    theta1, theta2, theta3 = map(to_radians, angles)

    x1 = L * math.cos(theta1)
    y1 = L * math.sin(theta1)

    x2 = x1 + L * math.cos(theta1 + theta2)
    y2 = y1 + L * math.sin(theta1 + theta2)

    x3 = x2 + L * math.cos(theta1 + theta2 + theta3)
    y3 = y2 + L * math.sin(theta1 + theta2 + theta3)

    return {
        "joint1": [x1, y1],
        "joint2": [x2, y2],
        "end_effector": [x3, y3],
        "angles": angles
    }

def inverse_kinematics(target, old_angles=None, old_target=None):
    """Compute angles for joints to move end effector to a given target position."""
    x, y = target
    if old_angles is None:
        old_angles = [0, 0, 0]
    if old_target is None:
        old_target = [x, y]

    dist = math.sqrt(x**2 + y**2)
    if dist > MAX_REACH:
        x = x * MAX_REACH / dist
        y = y * MAX_REACH / dist

    # Approximation: treat third link as aligned in direction
    xw = x - L * (x / dist) if dist != 0 else x
    yw = y - L * (y / dist) if dist != 0 else y

    r = math.sqrt(xw**2 + yw**2)
    cos_angle2 = (r**2 - L**2 - L**2) / (2 * L * L)
    cos_angle2 = max(min(cos_angle2, 1), -1)
    angle2 = math.acos(cos_angle2)

    k1 = L + L * math.cos(angle2)
    k2 = L * math.sin(angle2)
    angle1 = math.atan2(yw, xw) - math.atan2(k2, k1)

    angle3 = math.atan2(y - yw, x - xw) - angle1 - angle2

    theta1 = to_degrees(angle1)
    theta2 = to_degrees(angle2)
    theta3 = to_degrees(angle3)

    # Handle special edge case
    if approx_equal(theta2, 0) and approx_equal(theta3, 0):
        old_t1, old_t2, old_t3 = old_angles

        quadrant = None
        if x > 0 and y < 0: quadrant = 4
        elif x > 0 and y > 0: quadrant = 1
        elif x < 0 and y > 0: quadrant = 2
        elif x < 0 and y < 0: quadrant = 3

        old_x, old_y = old_target
        dx = x - old_x
        dy = y - old_y

        forbidden = False
        if quadrant == 4 and dx > 0 and dy < 0:
            forbidden = True
        elif quadrant == 1 and dx > 0 and dy > 0:
            forbidden = True
        elif quadrant == 2 and dx < 0 and dy > 0:
            forbidden = True
        elif quadrant == 3 and dx < 0 and dy < 0:
            forbidden = True

        if forbidden:
            return {
                "angles": [round(old_t1, 2), round(old_t2, 2), round(old_t3, 2)]
            }

    return {
        "angles": [round(theta1, 2), round(theta2, 2), round(theta3, 2)]
    }
