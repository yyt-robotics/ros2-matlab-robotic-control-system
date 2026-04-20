import threading
import time
import json
import os

from flask import Flask, request, render_template_string
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_interfaces.action import MoveToPose

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>6-DOF Robot Task Execution GUI</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 40px;
            max-width: 900px;
        }
        .row {
            margin-bottom: 14px;
        }
        label {
            display: inline-block;
            width: 80px;
            font-weight: bold;
        }
        input[type=range] {
            width: 320px;
            vertical-align: middle;
        }
        .val {
            display: inline-block;
            width: 90px;
            text-align: right;
            margin-left: 10px;
            color: #333;
        }
        button {
            padding: 8px 16px;
            margin-top: 10px;
            margin-right: 10px;
        }
        .log {
            margin-top: 20px;
            padding: 12px;
            border: 1px solid #ccc;
            background: #f7f7f7;
            white-space: pre-wrap;
            min-height: 220px;
        }
        .status {
            margin-top: 16px;
            padding: 12px;
            border-radius: 6px;
            font-weight: bold;
        }
        .status.ok {
            background: #e8f5e9;
            color: #1b5e20;
            border: 1px solid #a5d6a7;
        }
        .status.fail {
            background: #ffebee;
            color: #b71c1c;
            border: 1px solid #ef9a9a;
        }
        .status.info {
            background: #e3f2fd;
            color: #0d47a1;
            border: 1px solid #90caf9;
        }
    </style>
    <script>
        function bindSlider(name) {
            const slider = document.getElementById(name);
            const output = document.getElementById(name + "_val");
            output.textContent = slider.value;
            slider.oninput = function() {
                output.textContent = this.value;
            }
        }

        window.onload = function() {
            ["x","y","z","roll","pitch","yaw"].forEach(bindSlider);
        }
    </script>
</head>
<body>
    <h2>6-DOF Robot Task Execution GUI</h2>

    <form method="post" action="/send_goal">
        <div class="row">
            <label>x</label>
            <input type="range" id="x" name="x" min="-0.409" max="0.409" step="0.001" value="{{ x }}">
            <span class="val" id="x_val">{{ x }}</span>
        </div>

        <div class="row">
            <label>y</label>
            <input type="range" id="y" name="y" min="-0.429" max="0.429" step="0.001" value="{{ y }}">
            <span class="val" id="y_val">{{ y }}</span>
        </div>

        <div class="row">
            <label>z</label>
            <input type="range" id="z" name="z" min="-0.297" max="0.297" step="0.001" value="{{ z }}">
            <span class="val" id="z_val">{{ z }}</span>
        </div>

        <div class="row">
            <label>roll</label>
            <input type="range" id="roll" name="roll" min="-3.14" max="3.14" step="0.01" value="{{ roll }}">
            <span class="val" id="roll_val">{{ roll }}</span>
        </div>

        <div class="row">
            <label>pitch</label>
            <input type="range" id="pitch" name="pitch" min="-3.14" max="3.14" step="0.01" value="{{ pitch }}">
            <span class="val" id="pitch_val">{{ pitch }}</span>
        </div>

        <div class="row">
            <label>yaw</label>
            <input type="range" id="yaw" name="yaw" min="-3.14" max="3.14" step="0.01" value="{{ yaw }}">
            <span class="val" id="yaw_val">{{ yaw }}</span>
        </div>

        <div class="row">
            <label>Waypoint Name</label>
            <input type="text" id="wp_name" name="wp_name" placeholder="Enter name (optional)">
        </div>

        <div class="row">
            <label>motion</label>
            <select name="motion">
                <option value="R">R</option>
                <option value="L">L</option>
            </select>
        </div>

        <button type="submit">Send Goal</button>
        <button type="submit" formaction="/save_point">Save Point</button>
        <button type="submit" formaction="/run_waypoints">Run Sequence</button>
        <button type="submit" formaction="/clear_waypoints">Clear Points</button>
        <button type="submit" formaction="/save_program">Save Program</button>
        <button type="submit" formaction="/load_program">Load Program</button>
    </form>

    <form method="post" action="/cancel_goal">
        <button type="submit">Cancel Goal</button>
    </form>

    {% if status_text %}
    <div class="status {{ status_class }}">{{ status_text }}</div>
    {% endif %}

    <h3>Waypoints</h3>
    <div>
    {% for wp in waypoints %}
        <div style="border:1px solid #ccc; padding:8px; margin-bottom:6px; border-radius:6px; background:#fafafa;">
            <strong>[{{ wp.name }}]</strong> motion={{ wp.motion }}<br>
            x={{ wp.x }}, y={{ wp.y }}, z={{ wp.z }}<br>
            roll={{ wp.roll }}, pitch={{ wp.pitch }}, yaw={{ wp.yaw }}<br>

            <form method="post" action="/delete_point" style="display:inline;">
                <input type="hidden" name="idx" value="{{ loop.index0 }}">
                <button type="submit">Delete</button>
            </form>
            <form method="post" action="/move_up" style="display:inline; margin-left:4px;">
                <input type="hidden" name="idx" value="{{ loop.index0 }}">
                <button type="submit">Up</button>
            </form>
            <form method="post" action="/move_down" style="display:inline; margin-left:4px;">
                <input type="hidden" name="idx" value="{{ loop.index0 }}">
                <button type="submit">Down</button>
            </form>
        </div>
    {% endfor %}
    </div>

    <div class="log">{{ log_text }}</div>
</body>
</html>
"""

class WebActionClient(Node):
    def __init__(self):
        super().__init__('pose_web_gui_client')

        self._action_client = ActionClient(
            self,
            MoveToPose,
            'execute_task'
        )

        self.waypoints = []

        self.program_dir = os.path.expanduser('~/ros2_project_data/programs')
        os.makedirs(self.program_dir, exist_ok=True)
        self.program_file = os.path.join(self.program_dir, 'waypoints_program.json')

        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

        self.logs = []
        self._lock = threading.Lock()

        self.last_status_text = 'Web GUI started.'
        self.last_status_class = 'info'

    def save_waypoint(self, x, y, z, roll, pitch, yaw, motion="R",name=None):

        if not name:
            name = f"P{len(self.waypoints)}"

        wp = {
            'type': 'pose',
            'motion': motion, 
            'name': name,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }
        self.waypoints.append(wp)
        self.append_log(f"Saved waypoint '{name}': {wp}")
        self.set_status(f"Waypoint saved. Total: {len(self.waypoints)}", 'info')

    def clear_waypoints(self):
        self.waypoints = []
        self.append_log("Waypoints cleared.")
        self.set_status("Waypoints cleared.", 'info')

    def run_waypoints(self):
        if not self.waypoints:
            self.append_log('No waypoints to run.')
            self.set_status('No waypoints.', 'fail')
            return

        goal_msg = MoveToPose.Goal()

        goal_msg.waypoints_json = json.dumps(self.waypoints)

        self.append_log(f"Sending {len(self.waypoints)} waypoints...")
        self.set_status('Running waypoint sequence...', 'info')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def append_log(self, msg):
        with self._lock:
            timestamp = time.strftime('%H:%M:%S')
            self.logs.append(f'[{timestamp}] {msg}')
            self.logs = self.logs[-100:]

    def get_logs(self):
        with self._lock:
            return '\n'.join(self.logs)

    def set_status(self, text, css_class='info'):
        with self._lock:
            self.last_status_text = text
            self.last_status_class = css_class

    def get_status(self):
        with self._lock:
            return self.last_status_text, self.last_status_class

    def send_goal(self, x, y, z, roll, pitch, yaw):
        goal_msg = MoveToPose.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        goal_msg.roll = roll
        goal_msg.pitch = pitch
        goal_msg.yaw = yaw

        self.append_log(
            f'Sending goal: x={x:.3f}, y={y:.3f}, z={z:.3f}, '
            f'roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}'
        )
        self.set_status('Sending goal...', 'info')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.append_log('Action server not available.')
            self.set_status('Action server not available.', 'fail')
            return

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.append_log('Goal rejected')
            self.set_status('Goal rejected.', 'fail')
            return

        self.append_log('Goal accepted')
        self.set_status('Goal accepted. Waiting for execution result...', 'info')
        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.append_log(
            f'Feedback: step {feedback.current_step}/{feedback.total_steps}, '
            f'current_joints={list(feedback.current_joints)}'
        )

    def get_result_callback(self, future):
        result_wrap = future.result()
        result = result_wrap.result
        status = result_wrap.status

        self.append_log(
            f'Result: success={result.success}, '
            f'final_joints={list(result.final_joints)}, '
            f'message="{result.message}"'
        )
        self.append_log(f'Final status: {status}')

        if result.success:
            self.set_status(f'Success: {result.message}', 'ok')
        else:
            self.set_status(f'Failed: {result.message}', 'fail')

    def cancel_goal(self):
        if self._goal_handle is None:
            self.append_log('No active goal to cancel.')
            self.set_status('No active goal to cancel.', 'fail')
            return

        self.append_log('Canceling goal...')
        self.set_status('Canceling goal...', 'info')
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            self.append_log(f'Cancel response: {cancel_response}')
            self.set_status('Cancel request sent.', 'info')
        except Exception as e:
            self.append_log(f'Cancel failed: {e}')
            self.set_status(f'Cancel failed: {e}', 'fail')

    def delete_waypoint(self, idx):
        if 0 <= idx < len(self.waypoints):
            removed = self.waypoints.pop(idx)
            self.append_log(f"Deleted waypoint {idx}: {removed}")
            self.set_status(f"Waypoint {idx} deleted.", 'info')
        else:
            self.append_log(f"Invalid delete index: {idx}")
            self.set_status("Invalid index.", 'fail')

    def move_waypoint_up(self, idx):
        if 0 < idx < len(self.waypoints):
            self.waypoints[idx - 1], self.waypoints[idx] = self.waypoints[idx], self.waypoints[idx - 1]
            self.append_log(f"Moved waypoint {idx} up.")
            self.set_status(f"Waypoint {idx} moved up.", 'info')
        else:
            self.append_log(f"Invalid move up index: {idx}")
            self.set_status("Cannot move up.", 'fail')


    def move_waypoint_down(self, idx):
        if 0 <= idx < len(self.waypoints) - 1:
            self.waypoints[idx + 1], self.waypoints[idx] = self.waypoints[idx], self.waypoints[idx + 1]
            self.append_log(f"Moved waypoint {idx} down.")
            self.set_status(f"Waypoint {idx} moved down.", 'info')
        else:
            self.append_log(f"Invalid move down index: {idx}")
            self.set_status("Cannot move down.", 'fail')
    
    def save_program_to_file(self):
        try:
            with open(self.program_file, 'w', encoding='utf-8') as f:
                json.dump(self.waypoints, f, indent=2)

            self.append_log(f"Program saved to: {self.program_file}")
            self.set_status("Program saved successfully.", 'info')
        except Exception as e:
            self.append_log(f"Save program failed: {e}")
            self.set_status(f"Save failed: {e}", 'fail')

    def load_program_from_file(self):
        try:
            if not os.path.exists(self.program_file):
                self.append_log(f"Program file not found: {self.program_file}")
                self.set_status("Program file not found.", 'fail')
                return

            with open(self.program_file, 'r', encoding='utf-8') as f:
                self.waypoints = json.load(f)

            self.append_log(f"Program loaded from: {self.program_file}")
            self.set_status(f"Program loaded. Total waypoints: {len(self.waypoints)}", 'info')
        except Exception as e:
            self.append_log(f"Load program failed: {e}")
            self.set_status(f"Load failed: {e}", 'fail')


rclpy.init()
ros_node = WebActionClient()


def ros_spin():
    rclpy.spin(ros_node)


ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

app = Flask(__name__)


def render_page(x='0.10', y='0.10', z='0.10', roll='0.0', pitch='0.0', yaw='0.0'):
    status_text, status_class = ros_node.get_status()
    return render_template_string(
        HTML_PAGE,
        log_text=ros_node.get_logs(),
        status_text=status_text,
        status_class=status_class,
        x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw,
        waypoints=ros_node.waypoints
    )


@app.route('/', methods=['GET'])
def index():
    return render_page()


@app.route('/send_goal', methods=['POST'])
def send_goal():
    x_raw = request.form.get('x', '0.4')
    y_raw = request.form.get('y', '0.2')
    z_raw = request.form.get('z', '0.5')
    roll_raw = request.form.get('roll', '0.0')
    pitch_raw = request.form.get('pitch', '0.0')
    yaw_raw = request.form.get('yaw', '0.0')

    try:
        x = float(x_raw)
        y = float(y_raw)
        z = float(z_raw)
        roll = float(roll_raw)
        pitch = float(pitch_raw)
        yaw = float(yaw_raw)

        ros_node.send_goal(x, y, z, roll, pitch, yaw)

    except Exception as e:
        ros_node.append_log(f'Input error: {e}')
        ros_node.set_status(f'Input error: {e}', 'fail')

    return render_page(x_raw, y_raw, z_raw, roll_raw, pitch_raw, yaw_raw)


@app.route('/cancel_goal', methods=['POST'])
def cancel_goal():
    ros_node.cancel_goal()
    return render_page()


def main():
    ros_node.append_log('Web GUI started.')
    ros_node.set_status('Web GUI started.', 'info')
    app.run(host='0.0.0.0', port=8080, debug=False)

@app.route('/save_point', methods=['POST'])
def save_point():
    try:
        x = float(request.form.get('x', 0))
        y = float(request.form.get('y', 0))
        z = float(request.form.get('z', 0))
        roll = float(request.form.get('roll', 0))
        pitch = float(request.form.get('pitch', 0))
        yaw = float(request.form.get('yaw', 0))

        name = request.form.get('name', "").strip()

        motion = request.form.get('motion', 'R').strip().upper()

        ros_node.save_waypoint(x, y, z, roll, pitch, yaw, motion, name)

    except Exception as e:
        ros_node.append_log(f'Input error: {e}')
        ros_node.set_status(f'Input error: {e}', 'fail')

    return render_page(x, y, z, roll, pitch, yaw)

@app.route('/run_waypoints', methods=['POST'])
def run_waypoints():
    ros_node.run_waypoints()
    return render_page()

@app.route('/clear_waypoints', methods=['POST'])
def clear_waypoints():
    ros_node.clear_waypoints()
    return render_page()

@app.route('/delete_point', methods=['POST'])
def delete_point():
    idx = int(request.form.get('idx', -1))
    ros_node.delete_waypoint(idx)
    return render_page()

@app.route('/move_up', methods=['POST'])
def move_up():
    idx = int(request.form.get('idx', -1))
    if 0 < idx < len(ros_node.waypoints):
        ros_node.waypoints[idx-1], ros_node.waypoints[idx] = ros_node.waypoints[idx], ros_node.waypoints[idx-1]
        ros_node.append_log(f"Moved waypoint {idx} up")
        ros_node.set_status(f"Waypoint {idx} moved up", 'info')
    return render_page()

@app.route('/move_down', methods=['POST'])
def move_down():
    idx = int(request.form.get('idx', -1))
    if 0 <= idx < len(ros_node.waypoints)-1:
        ros_node.waypoints[idx+1], ros_node.waypoints[idx] = ros_node.waypoints[idx], ros_node.waypoints[idx+1]
        ros_node.append_log(f"Moved waypoint {idx} down")
        ros_node.set_status(f"Waypoint {idx} moved down", 'info')
    return render_page()

@app.route('/save_program', methods=['POST'])
def save_program():
    ros_node.save_program_to_file()
    return render_page()

@app.route('/load_program', methods=['POST'])
def load_program():
    ros_node.load_program_from_file()
    return render_page()


if __name__ == '__main__':
    main()
