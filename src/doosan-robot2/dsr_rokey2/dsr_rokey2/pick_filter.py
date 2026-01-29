import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 100
ACC = 100

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def gripper_open():
    print('gripper open...')
    from DSR_ROBOT2 import set_digital_output, wait
    # 핀 번호 설정 (본인 환경에 맞게 확인 필요)
    # 닫기 신호: 1번 ON / 2번 OFF
    # 열기 신호: 1번 OFF / 2번 ON
    PIN_CLOSE = 1
    PIN_OPEN = 2
    ON = 1
    OFF = 0
    set_digital_output(index=PIN_CLOSE, val=OFF)
    set_digital_output(index=PIN_OPEN, val=ON)
    # 물리적으로 열릴 시간 대기 (2초)
    wait(2.0)

def gripper_closed():
    print('gripper close...')
    from DSR_ROBOT2 import set_digital_output, wait
    # 핀 번호 설정 (본인 환경에 맞게 확인 필요)
    # 닫기 신호: 1번 ON / 2번 OFF
    # 열기 신호: 1번 OFF / 2번 ON
    PIN_CLOSE = 1
    PIN_OPEN = 2
    ON = 1
    OFF = 0
    set_digital_output(index=PIN_CLOSE, val=ON)
    set_digital_output(index=PIN_OPEN, val=OFF)
    # 물리적으로 열릴 시간 대기 (2초)
    wait(2.0)



def pick_filter():
    print('picking up filter...')
    from DSR_ROBOT2 import posx, movej, movel, move_spiral, wait, DR_AXIS_Z, DR_TOOL, DR_BASE
    # move_spiral(rev=3, rmax=30, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)

    initial_pose = [6.845, 3.351, 90.331, -0.175, 84.003, -11.663]
    filter_pose_pre = [12.557, 38.631, 132.647, 14.701, -79.879, 89.113]
    filter_pose = [11.157, 40.599, 124.127, 13.852, -73.648, 87.869]
    filter_pose_up = [5.738, 4.463, 110.467, 17.646, -24.381, 76.336]
    filter_pose_up_post = [-25.228, 12.677, 88.086, 25.676, -7.735, 65.953]
    filter_pose_post = [-51.328, 59.663, 52.511, -84.261, 133.62, 22.864]
    filter_pose_post_post = [-56.895, 43.272, 66.323, -88.407, 129.105, 19.202]
    filter_pose_drop = [-58.595, 48.597, 70.56, -82.317, 126.982, 30.648]
    filter_pose_push_pre_pre = [-51.883, 40.956, 47.111, -89.938, 130.794, 0.036]
    filter_pose_push_pre = [-98.908, 34.083, 18.596, 1.387, 127.95, 14.356]
    filter_pose_push = [-99.594, 22.166, 63.827, 2.978, 95.295, 9.695]

    movej(initial_pose, vel = 30, acc = 30)
    wait(1.0)
    movej(filter_pose_pre, vel = 30, acc = 30)
    movej(filter_pose, vel = 30, acc = 30)
    gripper_closed()
    movej(filter_pose_up , vel = 30, acc = 30)
    movej(filter_pose_up_post , vel = 30, acc = 30)
    movej(filter_pose_post , vel = 30, acc = 30)
    movej(filter_pose_post_post , vel = 30, acc = 30)
    movej(filter_pose_drop , vel = 30, acc = 30)
    gripper_open()
    movej(filter_pose_push_pre_pre, vel = 30, acc = 30)
    movej(filter_pose_push_pre, vel = 30, acc = 30)
    gripper_closed()
    movej(filter_pose_push, vel = 30, acc = 30)
    movej(filter_pose_push_pre, vel = 30, acc = 30)
    wait(1.0)
    movej(initial_pose, vel = 30, acc = 30)
    print("task done.")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("pick_filter", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node
    gripper_open()

    try:
        # 초기화는 한 번만 수행
        initialize_robot()
        print('initialize done')
        # 작업 수행 (한 번만 호출)
        pick_filter()
        
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()