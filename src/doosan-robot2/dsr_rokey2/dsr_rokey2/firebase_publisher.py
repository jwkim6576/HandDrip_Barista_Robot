import rclpy
import DR_init
import time
import firebase_admin
from firebase_admin import credentials, db
import json # 데이터 로깅을 위해 사용

# ----------------------------------------------------
# 1. Firebase 설정
# ----------------------------------------------------
# 🚨 로봇 PC의 실제 서비스 계정 키 파일 경로로 변경하세요.
FIREBASE_CERT_PATH = "/home/wook/cobot1_ws/src/m0609_monitor/config/serviceAccountKey.json" 
DATABASE_URL = "https://rokey-baristar-robot-default-rtdb.asia-southeast1.firebasedatabase.app"

# ----------------------------------------------------
# 2. 로봇 및 ROS 설정
# ----------------------------------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ----------------------------------------------------
# 3. Firebase 업데이트 함수
# ----------------------------------------------------

def firebase_update_data(node, dsr_node, db_ref, get_current_posx, get_current_posj):
    """
    로봇의 현재 상태를 Firebase DB에 업데이트하는 함수
    """
    try:
        # DSR 함수를 사용하여 현재 관절/위치 값 얻기
        current_posj = get_current_posj()
        current_posx = get_current_posx()[0] # [0]을 사용하여 위치/자세 값만 추출

        # 관절 데이터 매핑
        joints_data = {f'joint{i+1}': round(current_posj[i], 3) for i in range(6)}
        
        # Pose 데이터 매핑 (X, Y, Z, Rx, Ry, Rz)
        pose_keys = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        pose_data = {key: round(current_posx[i], 3) for i, key in enumerate(pose_keys)}
        
        # 전송할 최종 데이터 패키지
        data_to_update = {
            **joints_data,
            **pose_data,
            'last_update_timestamp': time.time(),
            # 'completed_jobs': (이 값은 로봇 프로그램의 다른 로직이 업데이트한다고 가정)
        }

        # Firebase에 데이터 쓰기 (update)
        db_ref.update(data_to_update)
        
        node.get_logger().info(f'Firebase updated successfully: {json.dumps(data_to_update)}')

    except Exception as e:
        node.get_logger().error(f'Firebase update failed: {e}')


def main(args=None):
    # ROS 노드 초기화
    rclpy.init(args=args)
    node = rclpy.create_node("firebase_monitor_publisher", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR 로봇 함수 임포트
    try:
        from DSR_ROBOT2 import get_current_posx, get_current_posj
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        return

    # 4. Firebase 초기화 및 DB 참조 설정
    try:
        cred = credentials.Certificate(FIREBASE_CERT_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        db_ref = db.reference('robot_status')
        node.get_logger().info('Firebase Admin SDK initialized.')
    except Exception as e:
        node.get_logger().error(f'Firebase initialization failed: {e}')
        return

    # 5. 주기적 업데이트 타이머 설정
    UPDATE_INTERVAL_SEC = 0.5 # 500ms마다 업데이트
    node.create_timer(
        UPDATE_INTERVAL_SEC, 
        lambda: firebase_update_data(
            node, 
            DR_init.__dsr__node, 
            db_ref, 
            get_current_posx, 
            get_current_posj
        )
    )

    node.get_logger().info(f'DSR Firebase Publisher Node running. Updating every {UPDATE_INTERVAL_SEC}s.')
    
    # 노드 실행
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()