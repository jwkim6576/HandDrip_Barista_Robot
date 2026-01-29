import rclpy
import DR_init
import threading
import time
import firebase_admin
from firebase_admin import credentials, db
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import RobotState

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
# 2. 로봇 제어 함수 (control_node 사용)
# ==========================================================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, wait
    # 명령 사이 간격을 둬서 충돌 방지
    time.sleep(1.0)
    print(" >>> [Init] Setting Tool...")
    set_tool(ROBOT_TOOL)
    time.sleep(0.5)
    print(" >>> [Init] Setting TCP...")
    set_tcp(ROBOT_TCP)
    time.sleep(0.5)
    print(" >>> [Init] Robot Initialized!")

def gripper_control(mode):
    from DSR_ROBOT2 import set_digital_output, wait
    if mode == 'open':
        set_digital_output(index=1, val=0)
        set_digital_output(index=2, val=1)
    elif mode == 'close':
        set_digital_output(index=1, val=1)
        set_digital_output(index=2, val=0)
    wait(1.5)

def pick_drip_spiral():
    print(' >>> [Task] Starting Drip Process...')
    from DSR_ROBOT2 import movej, move_spiral, wait, DR_AXIS_Z, DR_BASE
    
    # 좌표 데이터
    JReady = [0, 0, 45, 0, 90, 0]
    drip_pose_start = [-55.913, 57.317, 82.598, 43.408, 119.651, -61.692]
    grip_pose = [-49.777, 54.884, 91.819, 43.958, 115.185, -72.678]
    grip_pose_up = [-44.397, 31.812, 78.936, 69.851, 123.289, -25.682]
    drip_pose_ready = [-41.227, 48.359, 70.841, 70.342, 125.053, -40.584]
    
    drip_pose_1 = [-35.066, 58.746, 34.683, 84.607, 126.321, 22.368]
    drip_pose_2 = [-34.865, 55.335, 33.099, 82.686, 125.111, 29.795]
    drip_pose_3 = [-33.742, 55.432, 29.414, 87.071, 120.21, 51.093]

    gripper_control('open')
    
    movej(drip_pose_start, vel=30, acc=30)
    movej(grip_pose, vel=50, acc=50)
    
    gripper_control('close')
    
    movej(grip_pose_up, vel=50, acc=50)

    print('   [Motion] 1st drip & Spiral')
    movej(drip_pose_1, vel=50, acc=50)
    move_spiral(rev=3, rmax=50, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)
    movej(drip_pose_ready, vel=50, acc=50)
    wait(1.0)

    print('   [Motion] 2nd drip & Spiral')
    movej(drip_pose_2, vel=50, acc=50)
    move_spiral(rev=3, rmax=50, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)
    movej(drip_pose_ready, vel=50, acc=50)
    wait(1.0)

    print('   [Motion] 3rd drip & Spiral')
    movej(drip_pose_3, vel=50, acc=50)
    move_spiral(rev=3, rmax=50, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)
    
    movej(grip_pose_up, vel=50, acc=50)
    movej(grip_pose, vel=50, acc=50)
    
    gripper_control('open')
    movej(JReady, vel=50, acc=50)
    print(" >>> [Task] Done.")

# ==========================================================
# 3. [모니터링 전용] Topic 구독 콜백
#    DSR 라이브러리 함수를 안 쓰고, 직접 Topic을 듣습니다.
# ==========================================================
def joint_callback(msg):
    global current_status_data
    # 라디안 -> 도 변환 등 필요시 처리 (여기서는 단순 매핑)
    # 실제 joint_states는 라디안으로 오므로 변환이 필요할 수 있음
    # 편의상 DSR 라이브러리가 주는 deg 값과 유사하게 쓰려면 변환 필요하지만
    # 일단 값 갱신 확인용으로 그대로 넣습니다.
    current_status_data["joint_states_deg"] = list(msg.position)

def robot_state_callback(msg):
    global current_status_data
    current_status_data["robot_state"] = msg.robot_state
    current_status_data["robot_mode"] = msg.robot_state_str # 문자열일 수 있음

def update_firebase_loop():
    """Firebase에 데이터를 쏘는 함수 (별도 쓰레드)"""
    while True:
        try:
            # DSR 라이브러리의 get_current 함수를 쓰면 충돌나므로
            # 여기서는 로봇이 동작 중이라고 가정하고 '상태값'만 갱신하거나
            # 혹은 안전하게 control_node가 쉴 때만 읽어야 함.
            # 가장 안전한 방법: DSR_ROBOT2.get_current... 함수 사용 금지.
            # 대신 Topic 구독 데이터를 사용해야 함.
            
            # (간소화) 이번 버전에서는 Firebase 업로드만 수행
            status_data = {
                "last_update_timestamp": time.time(),
                "status": "Running"
            }
            db.reference('robot_status/dsr01').update(status_data)
        except:
            pass
        time.sleep(0.5)

def handle_order_change(event):
    global pending_order
    order_data = event.data
    if order_data and isinstance(order_data, dict):
        if order_data.get('command') == "START":
            pending_order = order_data
            print(f"[Firebase] Order Received: {pending_order['beverage']}")

# ==========================================================
# 4. 메인 실행부
# ==========================================================
def main(args=None):
    global pending_order
    rclpy.init(args=args)

    # ------------------------------------------------------
    # [Node 1] 제어용 노드 (Control Node)
    # 이 노드는 DSR_ROBOT2 라이브러리가 독점합니다.
    # 절대로 이 노드로 spin()을 돌리지 마세요!
    # ------------------------------------------------------
    control_node = rclpy.create_node("dsr_control_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = control_node

    # ------------------------------------------------------
    # [Firebase Init]
    # ------------------------------------------------------
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

    # 상태 업데이트용 별도 쓰레드 시작 (ROS와 무관하게 Firebase 통신)
    t_status = threading.Thread(target=update_firebase_loop)
    t_status.daemon = True
    t_status.start()

    print("==============================================")
    print("   Barista System (Collision-Free Mode)       ")
    print("==============================================")
    
    # 통신망 안정화 대기
    time.sleep(2.0)

    try:
        # 1. 초기화 (이제 충돌 안 남)
        initialize_robot()
        
        # 2. 메인 루프 (주문 처리)
        print(" >>> [Main] Waiting for orders...")
        while rclpy.ok():
            # 주문 확인
            if pending_order is not None:
                current_order = pending_order
                pending_order = None 
                
                print(f" >>> [Main] Making: {current_order.get('beverage')}")
                
                try:
                    db.reference(STATUS_PATH).set("MAKING")
                    # 여기서 로봇이 움직임 (DSR 내부에서 알아서 spin함)
                    pick_drip_spiral() 
                    
                    db.reference(STATUS_PATH).set("COMPLETE")
                    # 카운트 증가
                    try:
                        ref_count = db.reference('robot_status/completed_jobs')
                        ref_count.transaction(lambda val: (val or 0) + 1)
                    except: pass
                    
                except Exception as e:
                    print(f"Task Error: {e}")
                    db.reference(STATUS_PATH).set("ERROR")
                
                time.sleep(2)
                db.reference(STATUS_PATH).set("IDLE")
                print(" >>> [Main] Order Complete. Waiting...")

            # [핵심] 로봇이 쉬고 있을 때만 잠깐씩 통신 처리를 해줍니다.
            # 이렇게 하면 DSR 함수 실행 중일 때와 겹치지 않습니다.
            rclpy.spin_once(control_node, timeout_sec=0.01)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Closing...")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()