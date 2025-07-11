from flask import Flask, request
import subprocess
import os
import signal
import atexit
import signal

app = Flask(__name__)

# 프로세스 저장
rosbridge_proc = None
lidarslam_proc = None
lidar_proc = None
frmt_proc = None
serial_proc = None

@app.route('/launch_rosbridge', methods=['POST'])
def launch_rosbridge():
    global rosbridge_proc
    if rosbridge_proc is not None and rosbridge_proc.poll() is None:
        return 'rosbridge already running', 400
    try:
        rosbridge_proc = subprocess.Popen(
            ['bash', '-c', 'source ~/ros2_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        return 'rosbridge launched', 200
    except Exception as e:
        return str(e), 500

@app.route('/stop_rosbridge', methods=['POST'])
def stop_rosbridge():
    global rosbridge_proc
    if rosbridge_proc is not None and rosbridge_proc.poll() is None:
        try:
            os.killpg(os.getpgid(rosbridge_proc.pid), signal.SIGINT)
            rosbridge_proc = None
            return 'rosbridge stopped', 200
        except Exception as e:
            return f'Failed to stop rosbridge: {e}', 500
    return 'rosbridge not running', 400

@app.route('/launch_lidar', methods=['POST'])
def launch_lidar():
    global lidar_proc
    if lidar_proc is not None and lidar_proc.poll() is None:
        return 'LiDAR already running', 400
    try:
        lidar_proc = subprocess.Popen(
            ['bash', '-c', 'source ~/ros2_ws/install/setup.bash && ros2 launch ouster_ros driver.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        return 'LiDAR launched', 200
    except Exception as e:
        return str(e), 500

@app.route('/stop_lidar', methods=['POST'])
def stop_lidar():
    global lidar_proc
    if lidar_proc is not None and lidar_proc.poll() is None:
        try:
            os.killpg(os.getpgid(lidar_proc.pid), signal.SIGINT)
            lidar_proc = None
            return 'LiDAR stopped', 200
        except Exception as e:
            return f'Failed to stop LiDAR: {e}', 500
    return 'LiDAR not running', 400

@app.route('/launch_lidarslam', methods=['POST'])
def launch_lidarslam():
    global lidarslam_proc
    if lidarslam_proc is not None and lidarslam_proc.poll() is None:
        return 'lidarslam already running', 400
    try:
        lidarslam_proc = subprocess.Popen(
            ['bash', '-c', 'source ~/ros2_ws/install/setup.bash && ros2 launch lidarslam lidarslam.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        return 'lidarslam launched', 200
    except Exception as e:
        return str(e), 500

@app.route('/stop_lidarslam', methods=['POST'])
def stop_lidarslam():
    global lidarslam_proc
    if lidarslam_proc is not None and lidarslam_proc.poll() is None:
        try:
            os.killpg(os.getpgid(lidarslam_proc.pid), signal.SIGINT)
            lidarslam_proc = None
            return 'lidarslam stopped', 200
        except Exception as e:
            return f'Failed to stop lidarslam: {e}', 500
    return 'lidarslam not running', 400


@app.route('/launch_frmt', methods=['POST'])
def launch_frmt():
    global frmt_proc
    if frmt_proc is not None and frmt_proc.poll() is None:
        return 'FRMT already running', 400
    try:
        frmt_proc = subprocess.Popen(
            ['bash', '-c', 'source ~/study_ws/install/setup.bash && ros2 launch navigation odom_navigation.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        return 'FRMT Started', 200
    except Exception as e:
        return str(e), 500

@app.route('/stop_frmt', methods=['POST'])
def stop_frmt():
    global frmt_proc
    if frmt_proc is not None and frmt_proc.poll() is None:
        try:
            os.killpg(os.getpgid(frmt_proc.pid), signal.SIGINT)
            frmt_proc = None
            return 'FRMT stopped', 200
        except Exception as e:
            return f'Failed to stop FRMT: {e}', 500
    return 'FRMT not running', 400

@app.route('/send_data', methods=['POST'])
def send_data():
    global serial_proc
    if serial_proc is not None and serial_proc.poll() is None:
        return 'Serial 데이터가 이미 전송되고 있음', 400
    try:
        serial_proc = subprocess.Popen(
            ['bash', '-c', 'sudo chmod 777 /dev/tty* && source ~/study_ws/install/setup.bash && ros2 run ros2_serial_transporter navigation_serial_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        return 'Serial Start', 200
    except Exception as e:
        return str(e), 500

@app.route('/stop_data', methods=['POST'])
def stop_data():
    global serial_proc
    if serial_proc is not None and serial_proc.poll() is None:
        try:
            os.killpg(os.getpgid(serial_proc.pid), signal.SIGINT)
            serial_proc = None
            return 'Sending Serial stopped', 200
        except Exception as e:
            return f'Failed to stop Serial: {e}', 500
    return 'Serial not running', 400


def cleanup_processes():
    print("[INFO] Cleaning up ROS nodes...")
    for proc, name in [(rosbridge_proc, "rosbridge"),
                       (lidarslam_proc, "lidarslam"),
                       (lidar_proc, "lidar"),
                       (frmt_proc, "frmt"),
                       ]:
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                print(f"[INFO] {name} stopped.")
            except Exception as e:
                print(f"[WARN] Failed to stop {name}: {e}")

def handle_sigterm(sig, frame):
    cleanup_processes()
    exit(0)

    
# 시그널 핸들링 먼저 등록
signal.signal(signal.SIGINT, handle_sigterm)
signal.signal(signal.SIGTERM, handle_sigterm)

# 종료 시 atexit 백업 등록
atexit.register(cleanup_processes)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)