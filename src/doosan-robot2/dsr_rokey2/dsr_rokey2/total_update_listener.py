import rclpy
import DR_init
import threading
import time
import firebase_admin
from firebase_admin import credentials, db
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import RobotState

# [수정 1] 서비스 타입이 'MoveStop'임이 확인되었습니다!
from dsr_msgs2.srv import MoveStop 

# ==========================================================
# 1. 환경 설정
# ==========================================================
FIREBASE_CERT_PATH = "/home/wook/cobot1_ws/src/m0609_monitor/config/serviceAccountKey.json"
DATABASE_URL = "https://rokey-baristar-robot-default-rtdb.asia-southeast1.firebasedatabase.app"

ORDER_PATH = 'barista_control/order_command'
STATUS_PATH = 'barista_status/current_state'

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

global pending_order
pending_order = None 

# 모니터링을 위한 전역 변수
global current_status_data
current_status_data = {
    "joint_states_deg": [0]*6,
    "tcp_position": [0]*6,
    "robot_state": 0,
    "robot_mode": 0
}

# ==========================================================
# 2. 로봇 제어 함수
# ==========================================================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, wait
    time.sleep(1.0)
    print(" >>> [Init] Setting Tool...")
    set_tool(ROBOT_TOOL)
    time.sleep(0.5)
    print(" >>> [Init] Setting TCP...")
    set_tcp(ROBOT_TCP)
    time.sleep(0.5)
    print(" >>> [Init] Robot Initialized!")

def recover_robot_sequence():
    print(" !!! [System] Recovery Routine Started...")
    from DSR_ROBOT2 import movej, wait
    
    # 1. 그리퍼 열기
    gripper_control('open') 
    time.sleep(0.5)

    # 2. 설정 초기화 및 홈 이동
    initialize_robot()
    
    home_pose = [0, 0, 90, 0, 90, 0]
    print(" >>> [Recovery] Moving to Home Position (Slowly)...")
    movej(home_pose, vel=10, acc=10)
    
    print(" >>> [Recovery] Done. System is ready.")

def gripper_control(mode):
    from DSR_ROBOT2 import set_digital_output, wait
    PIN_CLOSE = 1
    PIN_OPEN = 2
    ON = 1; OFF = 0

    if mode == 'open':
        set_digital_output(index=PIN_CLOSE, val=OFF)
        set_digital_output(index=PIN_OPEN, val=ON)
    elif mode == 'close':
        set_digital_output(index=PIN_CLOSE, val=ON)
        set_digital_output(index=PIN_OPEN, val=OFF)
    wait(2.0)

def total_update():
    print(' >>> [Task] Starting Drip Process...')
    from DSR_ROBOT2 import movej, wait

    # 좌표 데이터
    drip_pose_start = [-0.001, -0.001, 90.033, 0.0, 90.0, 90.001]
    approach_pose_1 = [18.414, 3.883, 119.401, 6.07, 14.268, 90.137]
    approach_pose_2 = [35.197, 38.058, 121.363, 40.467, -64.321, 77.222]
    approach_pose_3 = [33.599, 39.295, 117.553, 40.532, -60.771, 77.222]

    gripper_control('open')

    movej(drip_pose_start, vel=30, acc=30)
    wait(1.0)
    movej(approach_pose_1, vel=30, acc=30)
    wait(1.0)
    movej(approach_pose_2, vel=30, acc=30)
    wait(1.0)
    movej(approach_pose_3, vel=30, acc=30)
    wait(1.0)

    gripper_control('close')
    wait(1.0)
    
    movej(approach_pose_2, vel=30, acc=30) 
    wait(1.0)

# ==========================================================
# 3. 통신 및 리스너
# ==========================================================
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

def handle_order_change(event):
    global pending_order
    
    order_data = event.data
    if order_data and isinstance(order_data, dict):
        command = order_data.get('command')
        
        # 1. 작업 시작
        if command == "START":
            pending_order = order_data
            print(f"[Firebase] Order Received: {pending_order.get('beverage')}")
            
        # 2. 비상 정지 (STOP)
        elif command == "STOP":
            print(" !!! [EMERGENCY] STOP COMMAND RECEIVED !!!")
            
            try:
                # [수정 2] MoveStop으로 클라이언트 생성
                # [수정 3] 주소도 '/motion/move_stop'이 표준입니다.
                stop_client = DR_init.__dsr__node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
                
                # Request 생성
                req = MoveStop.Request()
                req.stop_mode = 1 # 1: Quick Stop
                
                # 비동기 호출
                stop_client.call_async(req)
                print("   -> Stop Signal Sent via ROS2 Service (MoveStop).")

                db.reference(STATUS_PATH).set("EMERGENCY_STOP")
                pending_order = None 
                
            except Exception as e:
                print(f"Failed to call Stop Service: {e}")
            
        # 3. 복구 (RECOVER)
        elif command == "RECOVER":
            print(" [Firebase] Recovery Command Received.")
            pending_order = order_data

# ==========================================================
# 4. 메인 실행부
# ==========================================================
def main(args=None):
    global pending_order
    rclpy.init(args=args)

    control_node = rclpy.create_node("dsr_control_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = control_node

    try:
        cred = credentials.Certificate(FIREBASE_CERT_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        print("[System] Firebase Connected.")
    except Exception as e:
        print(f"Firebase Init Error: {e}")
        return

    db.reference(ORDER_PATH).delete()
    db.reference(ORDER_PATH).listen(handle_order_change)
    db.reference(STATUS_PATH).set("IDLE")

    t_status = threading.Thread(target=update_firebase_loop)
    t_status.daemon = True
    t_status.start()

    print("==============================================")
    print("   Barista System (Collision-Free Mode)       ")
    print("==============================================")
    
    time.sleep(2.0)

    try:
        initialize_robot()
        
        print(" >>> [Main] Waiting for orders...")
        while rclpy.ok():
            if pending_order is not None:
                current_order = pending_order
                pending_order = None 
                
                cmd_type = current_order.get('command')
                
                if cmd_type == "START":
                    print(f" >>> [Main] Making: {current_order.get('beverage')}")
                    try:
                        db.reference(STATUS_PATH).set("MAKING")
                        total_update() 
                        db.reference(STATUS_PATH).set("IDLE")
                        db.reference(ORDER_PATH).set(None)
                    except Exception as e:
                        print(f"Task Error: {e}")
                        db.reference(STATUS_PATH).set("ERROR")

                elif cmd_type == "RECOVER":
                    try:
                        db.reference(STATUS_PATH).set("RECOVERING")
                        recover_robot_sequence() 
                        db.reference(STATUS_PATH).set("IDLE")
                        db.reference(ORDER_PATH).set(None)
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