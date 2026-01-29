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

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def perform_task():
    """원하는 로봇 동작을 작성하는 부분"""

    from DSR_ROBOT2 import (
        movej, movel, move_periodic,
        posj, posx,
        DR_AXIS_Z
    )

    print("\n### Performing Task... ###\n")

    # 1) Joint pose로 이동
    target_j = posj(0, -30, 90, 0, 90, 0)
    movej(target_j, vel=VELOCITY, acc=ACC)

    # 2) Cartesian pose로 이동
    target_x = posx(400, 0, 300, 180, 0, 90)
    movel(target_x, vel=VELOCITY, acc=ACC)

    # 3) Move Periodic (Z축 상하 진동)
    move_periodic(
        amp=[0, 0, 20, 0, 0, 0],   # Z축 20mm 진동
        period=2.0,
        repeat=3
    )

    # 4) 홈 포지션 이동
    home = posj(0, 0, 0, 0, 0, 0)
    movej(home, vel=VELOCITY, acc=ACC)

    print("\n### Task Completed ###\n")




def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_periodic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
