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



def pick_drip_spiral():
    print('performing start!')
    from DSR_ROBOT2 import posx, movej, movel, move_spiral, wait, DR_AXIS_Z, DR_TOOL, DR_BASE
    # move_spiral(rev=3, rmax=30, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)

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

    movej(initial_pose, vel = 30, acc = 30)
    movej(initial_pose_post, vel = 30, acc = 30)
    movej(grip_pose_pre, vel=30, acc=30)
    movej(grip_pose, vel=30, acc=30)
    gripper_closed()
    # wait(2.0)
    
    print('moving...')
    movej(grip_pose_post, vel=50, acc=50)

    movej(drip_pose_pre, vel=50, acc=50)
    # wait(5.0)
    print("first drip....")
    movej(drip_pose_1_2, vel=50, acc=50)
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_pose_pre, vel=50, acc=50)
    wait(5.0)


    movej(drip_pose_2_2, vel=50, acc=50)
    print("second drip....")
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_pose_pre, vel=50, acc=50)
    wait(5.0)


    movej(drip_pose_3_2, vel=50, acc=50)
    print("third drip....")
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_all, vel=50, acc=50)
    movej(drip_pose_pre, vel=50, acc=50)
    movej(grip_pose_post, vel=50, acc=50)
    movej(grip_pose, vel=50, acc=50)
    wait(1.0)
    gripper_open()

    movej(grip_pose_pre, vel=50, acc=50)

    movej(initial_pose, vel = 30, acc = 30)
    print("task done.")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("pick_drip_spiral", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node
    gripper_open()

    try:
        # 초기화는 한 번만 수행
        initialize_robot()
        print('initialize done')
        # 작업 수행 (한 번만 호출)
        pick_drip_spiral()
        
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()