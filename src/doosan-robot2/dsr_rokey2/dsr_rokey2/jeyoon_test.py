
import rclpy
import DR_init
from std_msgs.msg import Bool, Int8
from rclpy.qos import QoSProfile, DurabilityPolicy
from google.cloud import firestore

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"

VELOCITY = 60
ACC = 60
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

robot_state = True           # True=정상, False=대기
process_step_external = None # 외부에서 restart_process로 요청한 스텝

process_name = "move_cell"

db = firestore.Client.from_service_account_json(
    "/home/jihoon/test/firebase_key.json"
)
process_progress = db.collection("process_progress").document("process_progress")

def log_process_step(step):
    process_progress.document().set({
        "robot_id": ROBOT_ID,
        "process_name": process_name,
        "process_step": step,
        "timestamp": firestore.SERVER_TIMESTAMP
    })


# ====================================================
# Robot Initialization
# ====================================================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)
    print("Initializing robot...")
    print(f"TCP: {ROBOT_TCP}, TOOL: {ROBOT_TOOL}")
    print("#" * 40)

# ====================================================
# 그리퍼 함수
# ====================================================

def release():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(3, OFF)
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait(0.5)

def grip():
    from DSR_ROBOT2 import set_digital_output, wait
    release()
    set_digital_output(3, OFF)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(0.5)

# ====================================================
# STEP 기반 작업 수행
# ====================================================
def perform_task(node):
    global process_step_external, robot_state
    print("Performing task with STEP state-machine...")

    from DSR_ROBOT2 import (
        posx, posj, movej, movel, 
        DR_MV_MOD_REL
    )

    # ---------------- 위치 정의 ----------------
    pose_home = [0, 0, 90, 0, 90, 0]
    pos_list = [
        posj([-29.78, -13.396, 123.192, 0.179, 70.357, -29.757]),
        posj([-35.17, -10.748, 121.213, 0.145, 69.7, -35.114]),
        posj([-40.116, -8.08, 119.07, 0.109, 69.185, -40.041]),
        posj([-44.445, -5.192, 116.58, 0.073, 68.79, -44.352]),
        posj([-24.704, -5.992, 117.319, 0.142, 68.853, -24.631]),
        posj([-29.878, -4.039, 115.584, 0.11, 68.655, -29.791]),
        posj([-34.433, -1.847, 113.522, 0.076, 68.526, -34.331]),
        posj([-38.55, 0.577, 111.125, 0.042, 68.498, -38.434])
    ]
    pos_stack = posj([-17.755, 8.664, 97.157, -0.084, 74.173, -107.651])
    move_down = posx([0, 0, -30, 0, 0, 0])
    move_up = posx([0, 0, 65, 0, 0, 0])

    process_step = 0

    while True:
        rclpy.spin_once(node, timeout_sec=0.01)

        # 외부 restart 요청 반영
        if process_step_external is not None:
            print(f":arrows_counterclockwise: 외부 요청으로 process_step 변경 → {process_step_external}")
            process_step = process_step_external
            process_step_external = None

        if not robot_state:
            continue  # 대기 상태

        # ---------------- STEP 0: 홈 이동 + 그리퍼 오픈 ----------------
        if process_step == 0:
            log_process_step(process_step)
            movej(pose_home, vel=VELOCITY, acc=ACC, radius=20)
            release()
            process_step = 1
            continue

        # ---------------- STEP 1~8: 셀 픽업 & 스태킹 ----------------
        if 1 <= process_step <= 8:
            log_process_step(process_step)
            print(f"STEP {process_step}: 셀 {process_step} 픽업 및 스태킹")
            cell_idx = process_step - 1
            pos_target = pos_list[cell_idx]

            movej(pos_target, vel=VELOCITY, acc=ACC, radius=20)
            movel(move_down, vel=80, acc=80, mod=DR_MV_MOD_REL)
            grip()
            movel(move_up, vel=80, acc=80, mod=DR_MV_MOD_REL)

            movej(pos_stack, vel=VELOCITY, acc=ACC)

            # 스태킹 위치 이동
            offset_x = 19*(cell_idx%4)
            offset_y = -68*(cell_idx//4)
            movel(posx([offset_x, offset_y, 0, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
            movel(posx([0, 0, -60, 0, 0, 0]), vel=80, acc=ACC, mod=DR_MV_MOD_REL)
            release()

            # 4번째/8번째 셀 뒤로 밀기
            if cell_idx == 3 or cell_idx == 7:
                movel(posx([25, 0, 0, 0, 0, 0]), vel=80, acc=ACC, mod=DR_MV_MOD_REL)
                grip()
                movel(posx([-12, 0, 0, 0, 0, 0]), vel=30, acc=ACC, mod=DR_MV_MOD_REL)
                movel(posx([0, 0, 70, 0, 0, 0]), vel=80, acc=ACC, mod=DR_MV_MOD_REL)
                release()
            process_step += 1
            continue

        # ---------------- STEP 9: 완료 ----------------
        if process_step == 9:
            print("모든 셀 스태킹 완료!")
            break

# ====================================================
# MAIN
# ====================================================
def main(args=None):
    global robot_state, process_step_external

    rclpy.init(args=args)
    node = rclpy.create_node("move_cell", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    start_flag = False

    def start_callback(msg):
        nonlocal start_flag
        if msg.data:
            print(":large_green_circle: start 신호 수신 완료 → carry_case 시작 가능")
            start_flag = True

    def alarm_callback(msg):
        global robot_state
        if msg.data:
            robot_state = False
            print(f"state_alert 수신: {msg.data} → robot_state={robot_state}")

    def restart_callback(msg):
        global process_step_external, robot_state
        nonlocal start_flag
        if start_flag:
            process_step_external = int(msg.data)
        robot_state = True
        print(f"restart_process 수신: {msg.data}")

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(Bool, "/carry_case_done", start_callback, qos)
    node.create_subscription(Bool, "/state_alert", alarm_callback, qos)
    node.create_subscription(Int8, "/restart_process", restart_callback, qos)

    done_pub = node.create_publisher(Bool, "/move_cell_done", qos)

    try:
        initialize_robot()

        # while rclpy.ok() and not start_flag:
        #     rclpy.spin_once(node, timeout_sec=0.1)

        perform_task(node)

        msg = Bool()
        msg.data = True
        done_pub.publish(msg)
        print(":large_green_circle: carry_case 완료 신호 publish 완료")

    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

