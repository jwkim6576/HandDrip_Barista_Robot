import rclpy
import DR_init

# ==========================================
# 1. 설정 상수 (Global Configuration)
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 속도 및 가속도 기본값
VELOCITY = 100
ACC = 100

# DR_init 필수 설정 (전역 변수)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ==========================================
# 2. 로봇 제어 함수들
# ==========================================

def initialize_robot():
    """
    로봇 초기화 함수
    - DSR_ROBOT2 모듈은 반드시 노드 생성 후(함수 내부)에서 임포트해야 합니다.
    """
    # [중요] 함수 내부에서 임포트 (Delayed Import)
    from DSR_ROBOT2 import set_tool, set_tcp, set_velx, set_accx

    print("\n" + "#" * 50)
    print(f"Initialize Robot: {ROBOT_ID} ({ROBOT_MODEL})")
    print(f"Tool: {ROBOT_TOOL}, TCP: {ROBOT_TCP}")
    print("#" * 50)

    # Tool 및 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    # (옵션) 전역 속도 설정이 필요하면 여기서 수행
    # set_velx(VELOCITY)
    # set_accx(ACC)


def control_gripper(mode='open'):
    """
    그리퍼 제어 통합 함수
    mode: 'open' 또는 'close'
    """
    from DSR_ROBOT2 import set_digital_output, wait
    
    # 핀 설정
    PIN_CLOSE = 1
    PIN_OPEN = 2
    ON = 1
    OFF = 0

    if mode == 'open':
        print('[Gripper] Opening...')
        set_digital_output(index=PIN_CLOSE, val=OFF)
        set_digital_output(index=PIN_OPEN, val=ON)
    elif mode == 'close':
        print('[Gripper] Closing...')
        set_digital_output(index=PIN_CLOSE, val=ON)
        set_digital_output(index=PIN_OPEN, val=OFF)
    
    # 물리적 동작 대기
    wait(2.0)


def pick_drip_spiral():
    """
    주요 작업 시퀀스: 이동 -> 잡기 -> 나선형 드립 -> 복귀
    """
    print('\n[Task] Starting Spiral Task...')
    # [중요] 필요한 모든 모션 함수를 여기서 임포트
    from DSR_ROBOT2 import posx, movej, movel, move_spiral, wait, DR_AXIS_Z, DR_TOOL, DR_BASE

    # --- 1. 준비 위치 이동 ---
    JReady = [0, 0, 45, 0, 90, 0]
    print("Moving to JReady...")
    movej(JReady, vel=50, acc=50)
    
    # 그리퍼 열기 (함수 재사용)
    control_gripper('open')

    # --- 2. 좌표 정의 ---
    # (주신 좌표 그대로 사용)
    drip_pose_start = [-55.913, 57.317, 82.598, 43.408, 119.651, -61.692]
    grip_pose       = [-49.777, 54.884, 91.819, 43.958, 115.185, -72.678]
    grip_pose_up    = [-44.397, 31.812, 78.936, 69.851, 123.289, -25.682]
    drip_pose_ready = [-41.227, 48.359, 70.841, 70.342, 125.053, -40.584]
    
    drip_pose_1     = [-35.066, 58.746, 34.683, 84.607, 126.321, 22.368]
    drip_pose_2     = [-34.865, 55.335, 33.099, 82.686, 125.111, 29.795]
    drip_pose_3     = [-33.742, 55.432, 29.414, 87.071, 120.21, 51.093]

    # --- 3. 접근 및 파지 (Pick) ---
    movej(drip_pose_start, vel=30, acc=30)
    movej(grip_pose, vel=50, acc=50)

    # 그리퍼 닫기
    control_gripper('close')
    wait(1.0) # 확실한 파지를 위해 추가 대기

    # 들어올리기
    movej(grip_pose_up, vel=50, acc=50)

    # --- 4. 드립 모션 (Spiral) 수행 ---
    print('[Motion] Start Dripping Sequence')
    
    # 첫 번째 드립
    movej(drip_pose_1, vel=50, acc=50)
    print(" -> 1st Drip Spiral")
    move_spiral(rev=3, rmax=50, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)
    movej(drip_pose_ready, vel=50, acc=50)
    wait(2.0) # 대기 시간 조금 최적화 (5초 -> 2초, 필요시 변경)

    # 두 번째 드립
    movej(drip_pose_2, vel=50, acc=50)
    print(" -> 2nd Drip Spiral")
    move_spiral(rev=3, rmax=50, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)
    movej(drip_pose_ready, vel=50, acc=50)
    wait(2.0)

    # 세 번째 드립
    movej(drip_pose_3, vel=50, acc=50)
    print(" -> 3rd Drip Spiral")
    move_spiral(rev=3, rmax=50, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)
    
    # --- 5. 복귀 ---
    movej(grip_pose_up, vel=50, acc=50)
    movej(grip_pose, vel=50, acc=50)
    
    control_gripper('open')
    wait(1.0)
    
    # 안전하게 약간 물러나기 (선택 사항)
    movej(drip_pose_start, vel=30, acc=30)

    print("[Task] All tasks completed successfully.")


# ==========================================
# 3. 메인 실행부
# ==========================================

def main(args=None):
    rclpy.init(args=args)
    
    # 노드 생성
    node = rclpy.create_node("move_periodic_task", namespace=ROBOT_ID)
    
    # [핵심] DR_init에 노드 연결 (이게 되어야 DSR_ROBOT2 기능 사용 가능)
    DR_init.__dsr__node = node

    try:
        # 1. 초기화 (Tool, TCP 설정) - 가장 먼저 수행 권장
        initialize_robot()
        print('Initialization done.')
        
        # 2. 작업 수행
        # 초기 상태로 그리퍼 열기
        control_gripper('open') 
        
        # 메인 시퀀스 실행
        pick_drip_spiral()
        
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        # 에러 발생 시 로그를 명확히 보기 위해 traceback 사용 가능
        import traceback
        traceback.print_exc()
    finally:
        # 종료 처리
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()