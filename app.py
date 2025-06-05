from flask import Flask, request, jsonify, render_template
import math

app = Flask(__name__)

L = 100  # Length of each arm
MAX_REACH = 3 * L
ANGLE_TOLERANCE = 0.5  # degrees

def to_radians(deg):
    return deg * math.pi / 180

def to_degrees(rad):
    return rad * 180 / math.pi

def approx_equal(a, b, tol=ANGLE_TOLERANCE):
    return abs(a - b) <= tol

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/forward', methods=['POST'])
def forward():
    data = request.json
    theta1, theta2, theta3 = map(to_radians, data['angles'])

    x1 = L * math.cos(theta1)
    y1 = L * math.sin(theta1)

    x2 = x1 + L * math.cos(theta1 + theta2)
    y2 = y1 + L * math.sin(theta1 + theta2)

    x3 = x2 + L * math.cos(theta1 + theta2 + theta3)
    y3 = y2 + L * math.sin(theta1 + theta2 + theta3)

    return jsonify({
        "joint1": [x1, y1],
        "joint2": [x2, y2],
        "end_effector": [x3, y3],
        "angles": data['angles']
    })

@app.route('/inverse', methods=['POST'])
def inverse():
    data = request.json
    x, y = data['target']
    old_angles = data.get('old_angles', [0, 0, 0])  # For validation

    dist = math.sqrt(x**2 + y**2)
    if dist > MAX_REACH:
        # Clamp target inside max reach circle
        x = x * MAX_REACH / dist
        y = y * MAX_REACH / dist

    # Inverse kinematics for 3-link planar arm with theta3=0 approx
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

    # Check special case when angles approx (A1, 0, 0)
    # We'll check if theta2 and theta3 near zero, and theta1 can be any angle A1
    if approx_equal(theta2, 0) and approx_equal(theta3, 0):
        old_t1, old_t2, old_t3 = old_angles
        # We assume old_t1 == theta1 or close enough for this test to make sense

        # Determine quadrant of current EE position
        quadrant = None
        if x > 0 and y < 0: quadrant = 4
        elif x > 0 and y > 0: quadrant = 1
        elif x < 0 and y > 0: quadrant = 2
        elif x < 0 and y < 0: quadrant = 3

        # Movement attempt will be sent from front-end, so backend cannot see direction easily,
        # but can check if the new x,y changed in forbidden direction from previous x,y
        old_x, old_y = data.get('old_target', [x, y])

        dx = x - old_x
        dy = y - old_y

        # Forbidden moves per quadrant
        forbidden = False
        if quadrant == 4 and dx > 0 and dy < 0:  # case 1
            forbidden = True
        elif quadrant == 1 and dx > 0 and dy > 0:  # case 2
            forbidden = True
        elif quadrant == 2 and dx < 0 and dy > 0:  # case 3
            forbidden = True
        elif quadrant == 3 and dx < 0 and dy < 0:  # case 4
            forbidden = True

        if forbidden:
            # Reject move: return old angles unchanged
            return jsonify({
                "angles": [round(old_t1, 2), round(old_t2, 2), round(old_t3, 2)]
            })

    return jsonify({
        "angles": [round(theta1, 2), round(theta2, 2), round(theta3, 2)]
    })

if __name__ == '__main__':
    app.run(debug=True)
