#!/usr/bin/env python3
import rclpy
import DR_init
import threading
import time
import subprocess
import os
import signal
import firebase_admin
from firebase_admin import credentials, db
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import RobotState
from dsr_msgs2.srv import MoveStop, SetRobotControl
from DR_common2 import posj, posx 

# 상수 직접 선언 (라이브러리 import 에러 방지)
DR_AXIS_Z = 2
DR_BASE = 0

# 초기에는 None으로 설정 (main에서 실제 import)
DR_LIB = None 

# ==========================================================
# 1. 환경 설정
# ==========================================================
FIREBASE_CERT_PATH = "/home/wook/cobot1_ws/src/m0609_monitor/config/serviceAccountKey.json"
DATABASE_URL = "https://rokey-baristar-robot-default-rtdb.asia-southeast1.firebasedatabase.app"

ORDER_PATH = 'barista_control/order_command'
STATUS_PATH = 'barista_status/current_state'
SPEED_PATH  = 'barista_control/setting/speed_ratio'

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

global pending_order
pending_order = None 

global bringup_process
bringup_process = None

global CURRENT_SPEED_RATIO
CURRENT_SPEED_RATIO = 1.0 

global STOP_EVENT
STOP_EVENT = False

# ==========================================================
# 2. [안전 장치] 로봇 동작 래퍼 함수
# ==========================================================
def check_stop_signal():
    global STOP_EVENT
    if STOP_EVENT:
        print(" 🛑 [System] STOP Flag Detected! Aborting Sequence...")
        raise Exception("EMERGENCY_STOP_TRIGGERED")

# DR_LIB가 main에서 import 된 후 사용되므로 global로 접근
def movej(*args, **kwargs):
    check_stop_signal()
    if DR_LIB: DR_LIB.movej(*args, **kwargs)
    check_stop_signal()

def movel(*args, **kwargs):
    check_stop_signal()
    if DR_LIB: DR_LIB.movel(*args, **kwargs)
    check_stop_signal()

def mwait():
    check_stop_signal()
    if DR_LIB: DR_LIB.mwait()
    check_stop_signal()

def wait(time_sec):
    check_stop_signal()
    if DR_LIB: DR_LIB.wait(time_sec)
    check_stop_signal()

def move_spiral(*args, **kwargs):
    check_stop_signal()
    if DR_LIB: DR_LIB.move_spiral(*args, **kwargs)
    check_stop_signal()

def initialize_robot_tool(tool_name, tcp_name, vel=100, acc=100):
    check_stop_signal()
    if DR_LIB:
        DR_LIB.set_tool(tool_name)
        DR_LIB.set_tcp(tcp_name)
    time.sleep(0.5)

def gripper_open():
    check_stop_signal()
    if DR_LIB:
        DR_LIB.set_digital_output(1, 0)
        DR_LIB.set_digital_output(2, 1)
        DR_LIB.wait(1.5)

def gripper_closed():
    check_stop_signal()
    if DR_LIB:
        DR_LIB.set_digital_output(1, 1)
        DR_LIB.set_digital_output(2, 0)
        DR_LIB.wait(1.5)

# ==========================================================
# 3. 로봇 동작 시퀀스
# ==========================================================

# (1) Pick Filter
def sequence_pick_filter(vel=100, acc=100):
    print("=== [Sequence] PICK FILTER START ===")
    
    initialize_robot_tool("Tool Weight", "GripperDA", vel, acc)
    gripper_open()

    initial_pose = posj([6.845, 3.351, 90.331, -0.175, 84.003, -11.663])
    filter_pose_pre = posj([12.557, 38.631, 132.647, 14.701, -79.879, 89.113])
    filter_pose = posj([11.157, 40.599, 124.127, 13.852, -73.648, 87.869])
    filter_pose_up = posj([5.738, 4.463, 110.467, 17.646, -24.381, 76.336])
    filter_pose_up_post = posj([-25.228, 12.677, 88.086, 25.676, -7.735, 65.953])
    filter_pose_post = posj([-51.328, 59.663, 52.511, -84.261, 133.62, 22.864])
    filter_pose_post_post = posj([-56.895, 43.272, 66.323, -88.407, 129.105, 19.202])
    filter_pose_drop = posj([-58.595, 48.597, 70.56, -82.317, 126.982, 30.648])
    filter_pose_push_pre_pre = posj([-51.883, 40.956, 47.111, -89.938, 130.794, 0.036])
    filter_pose_push_pre = posj([-98.908, 34.083, 18.596, 1.387, 127.95, 14.356])
    filter_pose_push = posj([-99.594, 22.166, 63.827, 2.978, 95.295, 9.695])
    filter_last_first = posx([-62.688, -504.252, 590.592, 165.085, -178.44, 146.402])
    filter_last_second = posx([96.574, -448.882, 618.168, 96.111, -179.722, 95.885])

    movej(initial_pose, vel=vel, acc=acc); mwait()
    movej(filter_pose_pre, vel=vel, acc=acc); mwait()
    movej(filter_pose, vel=vel, acc=acc); mwait()
    gripper_closed()

    movej(filter_pose_up, vel=vel, acc=acc); mwait()
    movej(filter_pose_up_post, vel=vel, acc=acc); mwait()
    movej(filter_pose_post, vel=vel, acc=acc); mwait()
    movej(filter_pose_post_post, vel=vel, acc=acc); mwait()
    movej(filter_pose_drop, vel=vel, acc=acc); mwait()
    
    gripper_open()

    movej(filter_pose_push_pre_pre, vel=vel, acc=acc); mwait()
    movej(filter_pose_push_pre, vel=vel, acc=acc); mwait()
    gripper_closed()
    movej(filter_pose_push, vel=vel, acc=acc); mwait()
    movel(filter_last_first, vel=vel, acc=acc); mwait()
    movel(filter_last_second, vel=vel, acc=acc); mwait()
    
    print("=== PICK FILTER COMPLETE ===")

# (2) Scoop
def sequence_scoop_joint(vel=50, acc=50):
    print("=== [Sequence] SCOOP START ===")
    
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", vel, acc)
    gripper_open()

    home = posj([0,0,90,0,90,0])
    scoop_approach  = posj([-57.05, 6.049, 108.19, -1.602, 62.046, 39.618])
    scoop_up        = posj([-6.989, 2.199, 89.892, 0.354, 86.657, -17.35])
    move_cup        = posx([204.3, -274.282, 681.772, 102.646, -175.475, -162.022])
    cup_up_first    = posx([-29.698, -287.555, 683.108, 101.592, -175.962, -162.972])
    cup_up_second   = posx([-34.487, -436.03, 566.438, 100.638, -174.732, -164.163])
    cup_down_first  = posx([-241.693, -488.041, 465.501, 179.75, -126.105, -102.649])
    cup_down_second = posx([-289.375, -471.722, 442.287, 178.345, -124.309, -98.711])
    cup_down_third  = posx([-262.545, -440.454, 183.676, 169.023, -57.383, -76.898])
    cup_up_return_first = posx([93.83, -332.521, 636.258, 78.345, -173.67, 163.539])
    cup_up_return_second    = posx([218.969, -362.568, 326.118, 160.762, 178.238, -109.933])

    movej(home, vel=vel, acc=acc); mwait()
    movej(scoop_up, vel=vel, acc=acc); mwait()

    movej(scoop_approach, vel=vel, acc=acc); mwait()
    gripper_closed()

    movel(move_cup, vel=vel, acc=acc); mwait()
    movel(cup_up_first, vel=vel, acc=acc); mwait()
    movel(cup_up_second, vel=vel, acc=acc); mwait()

    movel(cup_down_first, vel=vel, acc=acc); mwait()
    movel(cup_down_second, vel=vel, acc=acc); mwait()
    movel(cup_down_third, vel=vel, acc=acc); mwait()
    movel(cup_down_first, vel=vel, acc=acc); mwait()

    movel(cup_up_return_first, vel=vel, acc=acc); mwait()
    movel(cup_up_return_second, vel=vel, acc=acc); mwait()

    gripper_open()
    print("=== SCOOP COMPLETE ===")

# (3) Drip + Corn (HOT)
def sequence_drip_corn_HOT():
    print("=== [Sequence] DRIP + CORN (HOT) START ===")
    
    initialize_robot_tool("Tool Weight", "GripperDA", 100, 100)
    gripper_open()

    initial_pose = [-12.547, 3.93, 89.795, 0.636, 83.999, -31.147]
    initial_pose_post = [-35.91, 66.241, 31.802, 82.834, 117.793, -11.516]
    grip_pose_pre = [-36.048, 79.262, 31.271, 76.77, 116.035, -25.368]
    grip_pose = [-32.98, 79.395, 32.182, 76.772, 116.572, -27.1]
    grip_pose_post = [-60.78, 12.446, 104.298, 106.678, 61.859, -35.895]
    drip_pose_pre = [-139.826, 23.075, 92.657, 66.953, 131.54, -38.487]
    drip_pose_1 = [-139.826, 23.074, 92.657, 66.955, 131.539, -11.237]
    drip_pose_1_2 = [-140.198, 23.896, 91.039, 68.214, 132.575, -9.092]
    drip_pose_2 = [-139.826, 23.074, 92.657, 66.955, 131.539, -5.207]
    drip_pose_2_2 = [-140.198, 23.896, 91.039, 68.214, 132.575, -5.207]
    drip_pose_3 = [-138.03, 23.541, 88.005, 71.531, 131.463, 11.786]
    drip_pose_3_2 = [-138.231, 23.401, 86.95, 73.194, 132.565, 18.227]
    drip_all = [-135.714, 24.96, 81.541, 76.933, 130.696, 34.994]
    
    drip_home_first = posx([622.525, -344.134, 214.137, 83.547, 134.566, 92.034])
    drip_home_second = posx([535.126, -296.654, 287.331, 85.341, 92.104, 87.909])

    movej(initial_pose, vel=100, acc=100)
    movej(initial_pose_post, vel=100, acc=100)
    movej(grip_pose_pre, vel=100, acc=100)
    movej(grip_pose, vel=100, acc=100)
    gripper_closed()
    
    movej(grip_pose_post, vel=100, acc=100)
    movej(drip_pose_pre, vel=100, acc=100)

    print("first drip....")
    movej(drip_pose_1_2, vel=100, acc=100)
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_pose_pre, vel=100, acc=100)
    wait(5.0)

    movej(drip_pose_2_2, vel=100, acc=100)
    print("second drip....")
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_pose_pre, vel=100, acc=100)
    wait(5.0)

    movej(drip_pose_3_2, vel=100, acc=100)
    print("third drip....")
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_all, vel=100, acc=100)
    movej(drip_pose_pre, vel=100, acc=100)
    movej(grip_pose_post, vel=100, acc=100)
    movej(grip_pose, vel=100, acc=100)
    wait(1.0)
    gripper_open()

    movel(drip_home_first, vel=200, acc=200)
    movel(drip_home_second, vel=200, acc=200)

    movej(grip_pose_pre, vel=50, acc=50) 
    movej(initial_pose, vel=30, acc=30)
    print("drip done.")

    # 2. CORN
    print("=== CORN SEQUENCE (HOT) ===")
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", 100, 100)
    
    home = posj([0,0,90,0,90,0])
    corn_approach               = posx([249.466, -291.977, 415.322, 119.193, -176.125, -140.495])
    corn_move                   = posx([-252.153, -485.207, 397.362, 112.232, -176.032, -147.181])
    corn_move_gripclose_first   = posx([-350.443, -512.775, 360.696, 2.827, 112.498, 88.398])
    corn_move_gripclose_second  = posx([-339.012, -522.417, 279.587, 2.182, 104.321, 91.579])
    corn_move_gripclose_third   = posx([-256.695, -527.831, 211.872, 2.231, 96.789, 88.671])
    corn_move_third             = posx([148.621, -459.69, 246.718, 0.352, 95.692, 85.634])
    corn_move_fourth            = posx([199.966, -433.102, 158.873, 173.327, -134.387, -96.073])
    corn_move_fifth             = posx([259.322, -444.932, 121.129, 175.591, -155.109, -90.373])
    corn_move_sixth             = posx([362.302, -476.99, 344.538, 33.765, -178.573, 89.451])

    gripper_open()
    movej(home, vel=100, acc=100); mwait()
    
    movel(corn_approach, vel=100, acc=100); mwait()
    movel(corn_move, vel=100, acc=100); mwait()
    movel(corn_move_gripclose_first, vel=100, acc=100); mwait()
    movel(corn_move_gripclose_second, vel=100, acc=100); mwait()
    movel(corn_move_gripclose_third, vel=100, acc=100); mwait()

    gripper_closed()

    movel(corn_move_third, vel=100, acc=100); mwait()
    movel(corn_move_fourth, vel=100, acc=100); mwait()
    movel(corn_move_fifth, vel=100, acc=100); mwait()

    gripper_open()

    movel(corn_move_sixth, vel=100, acc=100); mwait()
    movej(home, vel=100, acc=100); mwait()
    
    print("=== DRIP + CORN (HOT) COMPLETE ===")

# (4) Drip + Corn + Ice (ICE 전용)
def sequence_drip_corn_ICE():
    print("=== [Sequence] DRIP + CORN + ICE (ICE) START ===")
    
    sequence_drip_corn_HOT()

    # 2. ICE 동작 추가 (속도 100 적용)
    print("=== ICE SEQUENCE START ===")
    
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", 100, 100)

    home = posj([0,0,90,0,90,0])
    scoop_approach  = posx([614.923, 207.339, 257.257, 160.743, -173.832, -112.913])
    scoop_back      = posj([22.675, 38.767, 57.022, -2.826, 87.728, 120.318])
    scoop_up        = posx([364.095, 180.291, 543.406, 10.158, 176.533, 102.411])
    ice_move_first  = posx([220.604, -344.674, 531.829, 3.904, 176.811, 96.375])
    ice_move_second = posx([-206.799, -497.996, 497.829, 4.45, 176.906, 97.276])
    ice_move_third  = posx([-170.265, -510.787, 457.536, 20.017, 176.391, 112.722])
    ice_down_first  = posx([51.115, -536.661, 406.13, 172.813, 134.785, -94.021])
    cup_up_return_first     = posx([272.998, -73.852, 441.73, 167.837, -176.627, -104.467])
    cup_up_return_second    = posx([455.164, 188.835, 364.774, 162.799, -176.761, -109.421])

    gripper_open()
    movej(home, vel=100, acc=100); mwait()

    print('scoop in (ICE)')
    movel(scoop_approach, vel=100, acc=100); mwait()

    gripper_closed()

    movej(scoop_back, vel=100, acc=100); mwait()
    movel(scoop_up, vel=100, acc=100); mwait()

    movel(ice_move_first, vel=100, acc=100); mwait()
    movel(ice_move_second, vel=100, acc=100); mwait()
    movel(ice_move_third, vel=100, acc=100); mwait()

    movel(ice_down_first, vel=100, acc=100); mwait()

    movel(ice_move_third, vel=100, acc=100); mwait()
    movel(cup_up_return_first, vel=100, acc=100); mwait()
    movel(cup_up_return_second, vel=100, acc=100); mwait()

    movej(scoop_back, vel=100, acc=100); mwait()
    movel(scoop_approach, vel=100, acc=100); mwait()
    gripper_open()
    
    print("=== ICE SEQUENCE COMPLETE ===")
    movej(home, vel=100, acc=100); mwait()


# ==========================================================
# 4. 작업 통합 실행 함수
# ==========================================================
def process_hot_task():
    print(">> Starting HOT Beverage Process")
    sequence_pick_filter()
    sequence_scoop_joint()
    sequence_drip_corn_HOT()
    print(">> HOT Process DONE")

def process_ice_task():
    print(">> Starting ICE Beverage Process")
    sequence_pick_filter()
    sequence_scoop_joint()
    sequence_drip_corn_ICE()
    print(">> ICE Process DONE")


# ==========================================================
# 5. 복구 및 제어 리스너
# ==========================================================
def recover_robot_sequence():
    print(" !!! [System] Recovery Routine Started...")
    from DSR_ROBOT2 import movej, wait
    global STOP_EVENT
    
    STOP_EVENT = False
    
    # 1. 하드웨어 리셋 (Red LED -> Servo ON)
    print("   -> 1. Attempting to SET SERVO ON (Red LED)...")
    try:
        servo_client = DR_init.__dsr__node.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')
        if servo_client.service_is_ready():
            req_safe_off = SetRobotControl.Request()
            req_safe_off.robot_control = 3
            servo_client.call_async(req_safe_off)
            time.sleep(1.0)

            req_safe_stop = SetRobotControl.Request()
            req_safe_stop.robot_control = 2
            servo_client.call_async(req_safe_stop)
            time.sleep(1.0)

            req_servo = SetRobotControl.Request()
            req_servo.robot_control = 1 
            servo_client.call_async(req_servo)
            print("      >> Servo ON Signal Sent.")
            time.sleep(1.5)
    except Exception as e:
        print(f"   [Warning] Servo Reset Failed: {e}")

    # 2. 소프트웨어 리셋 (Yellow LED / Motion Stop)
    print("   -> 2. Resetting Motion Context...")
    try:
        stop_client = DR_init.__dsr__node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
        if stop_client.service_is_ready():
            req_stop = MoveStop.Request()
            req_stop.stop_mode = 2 
            stop_client.call_async(req_stop)
            print("      >> Motion Reset Signal Sent.")
            time.sleep(0.5)
    except Exception as e:
        print(f"   [Warning] Motion Reset Failed: {e}")

    # 3. 안전 복귀
    print("   -> 3. Initializing Tool & Moving Home...")
    try:
        # 복구 함수 내부에서는 라이브러리를 직접 사용 (STOP_EVENT 체크 없이)
        if DR_LIB:
            DR_LIB.set_digital_output(1, 0)
            DR_LIB.set_digital_output(2, 1) # Gripper Open
            time.sleep(0.5)
            DR_LIB.set_tool("Tool Weight")
            DR_LIB.set_tcp("GripperDA")
            
            home_pose = [0, 0, 90, 0, 90, 0]
            DR_LIB.movej(home_pose, vel=10, acc=10)
        print(" >>> [Recovery] Done. System is ready!")
    except Exception as e:
        print(f"   [Error] Recover Move Failed: {e}")

def manage_robot_connection(command):
    global bringup_process
    if command == "CONNECT":
        if bringup_process is None:
            print(" >>> [System] Starting Robot Bringup...")
            cmd = ["ros2", "launch", "dsr_bringup2", "dsr_bringup2_rviz.launch.py", 
                   "mode:=real", "host:=192.168.1.100", "port:=12345", "model:=m0609"]
            bringup_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            print(" >>> [System] Bringup Started! (Wait 10 sec...)")
        else:
            print(" >>> [System] Robot is already connected.")
    elif command == "DISCONNECT":
        if bringup_process is not None:
            print(" >>> [System] Stopping Robot Bringup...")
            os.killpg(os.getpgid(bringup_process.pid), signal.SIGTERM)
            bringup_process = None
            print(" >>> [System] Robot Disconnected.")
        else:
            print(" >>> [System] Robot is already offline.")

def handle_speed_change(event):
    global CURRENT_SPEED_RATIO
    try:
        val = event.data
        if val is not None:
            CURRENT_SPEED_RATIO = float(val)
            speed_percent = int(CURRENT_SPEED_RATIO * 100)
            print(f" >>> [System] Real-time Speed Override: {speed_percent}%")
            if DR_LIB:
                DR_LIB.change_operation_speed(speed_percent)
    except Exception as e: 
        print(f"Speed Update Error: {e}")

def handle_order_change(event):
    global pending_order
    global STOP_EVENT
    
    order_data = event.data
    if order_data and isinstance(order_data, dict):
        command = order_data.get('command')
        
        if command == "START":
            STOP_EVENT = False 
            pending_order = order_data
            print(f"[Firebase] Order: {pending_order.get('beverage')} ({pending_order.get('temp')})")
            
        elif command == "STOP":
            print(" !!! [EMERGENCY] STOP COMMAND RECEIVED !!!")
            STOP_EVENT = True 
            try:
                stop_client = DR_init.__dsr__node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
                req = MoveStop.Request()
                req.stop_mode = 2 
                stop_client.call_async(req)
                print("   -> Stop Signal Sent & Flag Set.")
                db.reference(STATUS_PATH).set("EMERGENCY_STOP")
                pending_order = None 
            except Exception as e:
                print(f"Failed to call Stop Service: {e}")
            
        elif command == "RECOVER":
            print(" [Firebase] Recovery Command Received.")
            STOP_EVENT = False 
            pending_order = order_data
            
        elif command == "CONNECT":
            manage_robot_connection("CONNECT")
        elif command == "DISCONNECT":
            manage_robot_connection("DISCONNECT")

def update_firebase_loop():
    while True:
        try:
            status_data = {
                "last_update_timestamp": time.time(),
                "status": "Running"
            }
            db.reference('robot_status/dsr01').update(status_data)
        except: pass
        time.sleep(0.5)

# ==========================================================
# 6. 메인 실행부
# ==========================================================
def main(args=None):
    global pending_order
    global DR_LIB 

    rclpy.init(args=args)

    control_node = rclpy.create_node("dsr_control_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = control_node

    import DSR_ROBOT2 as DR_LIB_LOCAL
    DR_LIB = DR_LIB_LOCAL

    try:
        cred = credentials.Certificate(FIREBASE_CERT_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        print("[System] Firebase Connected.")
    except Exception as e:
        print(f"Firebase Init Error: {e}")
        return

    # 리스너 설정
    db.reference(ORDER_PATH).delete()
    db.reference(ORDER_PATH).listen(handle_order_change)
    db.reference(SPEED_PATH).listen(handle_speed_change) 
    db.reference(STATUS_PATH).set("IDLE")
    
    if db.reference(SPEED_PATH).get() is None:
        db.reference(SPEED_PATH).set(1.0)

    t_status = threading.Thread(target=update_firebase_loop)
    t_status.daemon = True
    t_status.start()

    print("==============================================")
    print("   Barista System (Final Complete Ver)        ")
    print("==============================================")
    
    time.sleep(2.0)

    try:
        DR_LIB.set_tool("Tool Weight")
        DR_LIB.set_tcp("GripperDA")
        
        print(" >>> [Main] Waiting for orders...")
        while rclpy.ok():
            if pending_order is not None:
                current_order = pending_order
                pending_order = None 
                
                cmd_type = current_order.get('command')
                temp_type = current_order.get('temp')
                
                if cmd_type == "START":
                    try:
                        db.reference(STATUS_PATH).set("MAKING")
                        if temp_type == "Ice":
                            process_ice_task()
                        else:
                            process_hot_task()
                        
                        db.reference(STATUS_PATH).set("IDLE")
                        db.reference(ORDER_PATH).delete()
                    except Exception as e:
                        print(f"Task Aborted: {e}")
                        db.reference(STATUS_PATH).set("ERROR")

                elif cmd_type == "RECOVER":
                    try:
                        db.reference(STATUS_PATH).set("RECOVERING")
                        recover_robot_sequence() 
                        db.reference(STATUS_PATH).set("IDLE")
                        db.reference(ORDER_PATH).delete()
                    except Exception as e:
                         print(f"Recovery Error: {e}")

            rclpy.spin_once(control_node, timeout_sec=0.01)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Closing...")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()