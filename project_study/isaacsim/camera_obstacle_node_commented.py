#!/usr/bin/env python3

# =============================================================================
# camera_obstacle_node.py
# =============================================================================
# 역할:
#   object_3d_projector_node가 만든 base_link 기준 3D detection 결과
#   (/detection/objects_3d_base)를 받아서,
#   AMR 주행 회피에 쓰기 쉬운 2D 장애물 목록(/obstacles/camera_detected)과
#   RViz 시각화 marker(/obstacles/camera_markers)를 발행하는 노드임.
#
# 전체 데이터 흐름:
#   /detection/objects_3d_base
#       -> class/confidence/position 필터링함
#       -> base_link 기준 x,y 장애물 PoseArray로 변환함
#       -> RViz MarkerArray로 시각화함
#
# 이 노드의 위치:
#   YOLO + depth + TF로 얻은 객체 위치를 navigation/local avoidance에서 쓰기 쉬운 형태로 바꾸는
#   perception-to-navigation adapter 역할임.
#
# 좌표계 기준:
#   이 노드는 입력과 출력을 모두 base_link 기준으로 다룸.
#   base_link 기준에서 일반적으로 x는 로봇 전방, y는 로봇 왼쪽, z는 위쪽임.
#   local MPC/CBF 회피에서는 좌/우/전방 판단이 중요하므로 base_link 기준이 자연스러움.
#
# 주의:
#   이 노드는 아직 LiDAR confirmation이나 tracking을 수행하지 않음.
#   즉 YOLO false positive가 들어오면 obstacle로 발행될 수 있음.
#   W5에서는 confidence threshold, depth 품질 필터링, LiDAR fusion, tracking 등을 추가해
#   더 실무적인 obstacle pipeline으로 확장 가능함.
# =============================================================================

import math
import time

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray


class CameraObstacleNode(Node):
    def __init__(self):
        # ROS2 노드 이름을 camera_obstacle_node로 설정함.
        super().__init__("camera_obstacle_node")

        # ---------------------------------------------------------------------
        # ROS parameter 선언부
        # ---------------------------------------------------------------------
        # 이 노드는 config/camera_obstacle.yaml에서 입력/출력 토픽과 필터 조건을 받음.
        # 실험하면서 class 목록, confidence threshold, 거리 제한 등을 쉽게 바꾸기 위한 구조임.
        # ---------------------------------------------------------------------

        # 입력 토픽임.
        # object_3d_projector_node가 camera frame에서 base_link로 변환한 Detection3DArray가 들어옴.
        self.declare_parameter("input_topic", "/detection/objects_3d_base")

        # 출력 1: 다른 노드가 쓰기 쉬운 장애물 위치 목록임.
        # PoseArray는 간단히 여러 pose를 담을 수 있어서 local obstacle 후보 전달에 편함.
        self.declare_parameter("output_obstacles_topic", "/obstacles/camera_detected")

        # 출력 2: RViz 시각화용 marker임.
        # 장애물 위치를 cylinder와 text로 확인하기 위한 용도임.
        self.declare_parameter("output_markers_topic", "/obstacles/camera_markers")

        # 카메라 detection 중 어떤 class를 장애물로 볼지 설정함.
        # person은 동적 장애물/안전거리 확대 대상으로 중요함.
        # box, bottle, chair 등은 물류창고 객체나 일반 장애물 후보로 둔 예시임.
        self.declare_parameter("target_classes", ["person", "box", "bottle", "chair"])

        # base_link 기준 x 범위 필터임.
        # x는 로봇 전방 거리임.
        # min_x보다 작으면 로봇 뒤/너무 가까운 잘못된 값으로 보고 제외함.
        # max_x보다 크면 너무 먼 detection으로 local 회피에 당장 필요 없거나 신뢰도가 낮을 수 있어 제외함.
        self.declare_parameter("min_x", 0.1)
        self.declare_parameter("max_x", 8.0)

        # base_link 기준 y 범위 필터임.
        # y는 로봇 좌우 방향임.
        # 너무 좌우로 먼 객체는 현재 주행 경로에 직접적인 회피 대상이 아닐 수 있어 제외함.
        self.declare_parameter("max_abs_y", 3.0)

        # confidence 필터임.
        # YOLO가 낮은 confidence로 검출한 false positive를 줄이기 위한 1차 필터임.
        # 반사된 사람처럼 confidence가 낮은 detection은 이 값으로 걸러낼 수 있음.
        self.declare_parameter("min_confidence", 0.45)

        # RViz marker cylinder 반지름임.
        # 실제 사람 크기를 정확히 표현하기보다는 obstacle 위치 확인용 marker 크기임.
        self.declare_parameter("marker_radius", 0.25)

        # 로그 출력 주기임.
        # 매 callback마다 로그를 찍으면 터미널이 너무 지저분해지므로 1초에 한 번 정도만 출력함.
        self.declare_parameter("log_period", 1.0)

        # ---------------------------------------------------------------------
        # parameter 값을 멤버 변수로 읽어옴.
        # ---------------------------------------------------------------------
        self.input_topic = self.get_parameter("input_topic").value
        self.output_obstacles_topic = self.get_parameter("output_obstacles_topic").value
        self.output_markers_topic = self.get_parameter("output_markers_topic").value

        self.target_classes = list(self.get_parameter("target_classes").value)
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.max_abs_y = float(self.get_parameter("max_abs_y").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.marker_radius = float(self.get_parameter("marker_radius").value)
        self.log_period = float(self.get_parameter("log_period").value)

        # 마지막으로 로그를 출력한 wall-clock time임.
        self.last_log_time = 0.0

        # ---------------------------------------------------------------------
        # Detection3DArray subscriber 생성함.
        # ---------------------------------------------------------------------
        # 입력 메시지는 이미 base_link 기준으로 변환된 3D detection임.
        # 따라서 이 노드는 추가 TF 변환을 하지 않고, 필터링 후 2D 장애물로 변환함.
        # ---------------------------------------------------------------------
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            self.input_topic,
            self.detection_callback,
            10,
        )

        # ---------------------------------------------------------------------
        # PoseArray publisher 생성함.
        # ---------------------------------------------------------------------
        # /obstacles/camera_detected는 후단 MPC/CBF/local planner가 쓰기 쉬운 단순 형태임.
        # 각 pose.position.x/y는 base_link 기준 장애물의 2D 위치임.
        # z는 local planar avoidance를 위해 0으로 둠.
        # ---------------------------------------------------------------------
        self.obstacle_pub = self.create_publisher(
            PoseArray,
            self.output_obstacles_topic,
            10,
        )

        # ---------------------------------------------------------------------
        # MarkerArray publisher 생성함.
        # ---------------------------------------------------------------------
        # RViz에서 장애물 위치와 class/confidence를 눈으로 확인하기 위한 시각화 토픽임.
        # 실제 제어에는 MarkerArray를 쓰지 않음.
        # ---------------------------------------------------------------------
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.output_markers_topic,
            10,
        )

        self.get_logger().info("Camera obstacle node started")
        self.get_logger().info(f"Subscribe: {self.input_topic}")
        self.get_logger().info(f"Publish obstacles: {self.output_obstacles_topic}")
        self.get_logger().info(f"Publish markers  : {self.output_markers_topic}")
        self.get_logger().info(f"Target classes   : {self.target_classes}")

    def detection_callback(self, msg: Detection3DArray):
        # ---------------------------------------------------------------------
        # base_link 기준 Detection3DArray를 카메라 기반 obstacle 후보로 변환함.
        # ---------------------------------------------------------------------
        # 입력:
        #   Detection3DArray
        #   - det.results[0].hypothesis.class_id : class name, 예: person
        #   - det.results[0].hypothesis.score    : confidence
        #   - det.bbox.center.position           : base_link 기준 3D 위치
        #
        # 출력:
        #   PoseArray
        #   - pose.position.x/y : base_link 기준 2D obstacle 위치
        #   - pose.position.z   : local 회피용으로 0으로 설정함
        #
        #   MarkerArray
        #   - RViz에서 cylinder/text로 표시함.
        # ---------------------------------------------------------------------

        # PoseArray 생성함.
        # header는 입력 detection과 같은 시간 정보를 유지하되,
        # frame_id는 base_link로 명시함.
        obstacle_msg = PoseArray()
        obstacle_msg.header = msg.header
        obstacle_msg.header.frame_id = "base_link"

        marker_array = MarkerArray()

        valid_count = 0

        for det in msg.detections:
            # Detection3D 안에 hypothesis가 없으면 class/confidence를 알 수 없으므로 skip함.
            if len(det.results) == 0:
                continue

            class_id = det.results[0].hypothesis.class_id
            score = float(det.results[0].hypothesis.score)

            # -----------------------------------------------------------------
            # class 필터링함.
            # -----------------------------------------------------------------
            # 모든 YOLO class를 obstacle로 볼 필요는 없음.
            # 예를 들어 person은 회피 대상이지만, 특정 class는 무시하고 싶을 수 있음.
            # target_classes에 포함된 class만 통과시킴.
            # -----------------------------------------------------------------
            if class_id not in self.target_classes:
                continue

            # confidence가 너무 낮으면 false positive 가능성이 크므로 제외함.
            if score < self.min_confidence:
                continue

            p = det.bbox.center.position

            x = float(p.x)
            y = float(p.y)
            z = float(p.z)

            # -----------------------------------------------------------------
            # 수치 유효성 검사함.
            # -----------------------------------------------------------------
            # depth/projection 과정에서 NaN, inf가 생길 수 있음.
            # 이런 값이 MPC/CBF에 들어가면 제어 입력이 망가질 수 있으므로 반드시 제거함.
            # -----------------------------------------------------------------
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue

            # -----------------------------------------------------------------
            # base_link 기준 x 거리 필터링함.
            # -----------------------------------------------------------------
            # x < min_x:
            #   로봇 뒤쪽 또는 너무 가까운 불안정 detection일 수 있음.
            # x > max_x:
            #   너무 먼 객체라 local avoidance에 당장 필요 없거나 depth 신뢰도가 낮을 수 있음.
            # -----------------------------------------------------------------
            if x < self.min_x or x > self.max_x:
                continue

            # 좌우로 너무 먼 객체는 현재 local 회피 대상에서 제외함.
            if abs(y) > self.max_abs_y:
                continue

            # -----------------------------------------------------------------
            # PoseArray에 넣을 2D obstacle pose 생성함.
            # -----------------------------------------------------------------
            # AMR local avoidance에서는 보통 바닥 평면의 장애물 위치(x,y)가 중요함.
            # 객체 높이 z는 이 단계에서 0으로 내리고, obstacle point/cylinder처럼 취급함.
            # 실제 사람의 높이 정보는 Detection3D나 debug marker에서 확인 가능함.
            # -----------------------------------------------------------------
            pose = Pose()
            pose.position.x = x
            pose.position.y = y

            # AMR 회피에서는 바닥 평면 장애물로 다루기 위해 z는 0으로 둠
            # 실제 detection 높이는 marker text나 debug에서 확인 가능함
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            obstacle_msg.poses.append(pose)

            # -----------------------------------------------------------------
            # RViz 시각화용 cylinder marker 생성함.
            # -----------------------------------------------------------------
            # cylinder는 obstacle 위치를 눈으로 확인하기 위한 용도임.
            # marker_id는 valid_count를 사용해서 한 프레임 안에서 고유하게 만듦.
            # -----------------------------------------------------------------
            marker_array.markers.append(
                self.make_obstacle_marker(
                    marker_id=valid_count,
                    class_id=class_id,
                    score=score,
                    x=x,
                    y=y,
                    z=0.0,
                    stamp=msg.header.stamp,
                )
            )

            # text marker는 class와 confidence를 표시함.
            marker_array.markers.append(
                self.make_text_marker(
                    marker_id=1000 + valid_count,
                    class_id=class_id,
                    score=score,
                    x=x,
                    y=y,
                    z=0.45,
                    stamp=msg.header.stamp,
                )
            )

            valid_count += 1

        # ---------------------------------------------------------------------
        # marker 삭제 처리함.
        # ---------------------------------------------------------------------
        # RViz marker는 새 메시지가 없으면 이전 marker가 화면에 남아 있을 수 있음.
        # detection이 0개인 경우 DELETEALL marker를 보내서 이전 marker를 제거함.
        # 단, 현재 구현은 valid_count==0일 때만 DELETEALL을 보냄.
        # detection 개수가 줄어드는 경우 일부 marker id가 남을 수 있으므로,
        # 더 견고하게 만들려면 매 프레임 DELETEALL 후 ADD를 보내는 방식도 가능함.
        # ---------------------------------------------------------------------
        if valid_count == 0:
            marker_array.markers.append(self.make_delete_all_marker(msg.header.stamp))

        # 필터링된 obstacle 목록과 RViz marker를 발행함.
        self.obstacle_pub.publish(obstacle_msg)
        self.marker_pub.publish(marker_array)

        # ---------------------------------------------------------------------
        # 주기적으로 로그 출력함.
        # ---------------------------------------------------------------------
        # obstacle 개수와 base_link 기준 x/y 위치를 확인할 수 있음.
        # W4 검증에서는 정면: y≈0, 왼쪽: y>0, 오른쪽: y<0인지 확인함.
        # ---------------------------------------------------------------------
        now = time.time()
        if now - self.last_log_time >= self.log_period:
            self.get_logger().info(
                f"Camera obstacles: {valid_count} | "
                f"frame={obstacle_msg.header.frame_id}"
            )

            for i, pose in enumerate(obstacle_msg.poses):
                self.get_logger().info(
                    f"  obstacle[{i}] base_xy=({pose.position.x:.3f}, "
                    f"{pose.position.y:.3f}) m"
                )

            self.last_log_time = now

    def make_obstacle_marker(self, marker_id, class_id, score, x, y, z, stamp):
        # ---------------------------------------------------------------------
        # RViz에 표시할 cylinder marker를 생성함.
        # ---------------------------------------------------------------------
        # Marker 기본 개념:
        #   header.frame_id : marker 좌표가 어떤 frame 기준인지 나타냄.
        #   ns/id           : marker를 구분하는 namespace와 id임.
        #   type            : CYLINDER, CUBE, SPHERE, TEXT 등 시각화 형태임.
        #   action          : ADD, DELETE, DELETEALL 등이 있음.
        #   pose/scale      : 위치/자세/크기임.
        #   color           : RGBA 색상임.
        #   lifetime        : marker 유지 시간임.
        # ---------------------------------------------------------------------
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.ns = "camera_obstacles"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # base_link 기준 장애물 위치에 cylinder를 둠.
        # z + 0.15는 cylinder 중심 높이임. scale.z=0.30이므로 바닥에서 0~0.3m 정도로 보이게 됨.
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.15
        marker.pose.orientation.w = 1.0

        # cylinder 지름은 marker_radius*2로 설정함.
        marker.scale.x = self.marker_radius * 2.0
        marker.scale.y = self.marker_radius * 2.0
        marker.scale.z = 0.30

        # 색상은 사람과 일반 물체를 구분하기 위함임.
        # 사람은 빨간색 계열, 일반 물체는 파란색 계열로 표시함.
        if class_id == "person":
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.8
        else:
            marker.color.r = 0.2
            marker.color.g = 0.6
            marker.color.b = 1.0
            marker.color.a = 0.8

        # lifetime=1초로 두면 detection이 끊겼을 때 오래 남지 않음.
        marker.lifetime.sec = 1
        return marker

    def make_text_marker(self, marker_id, class_id, score, x, y, z, stamp):
        # ---------------------------------------------------------------------
        # RViz에 class/confidence text marker를 생성함.
        # ---------------------------------------------------------------------
        # TEXT_VIEW_FACING은 카메라 시점 방향으로 항상 글자가 보이도록 하는 marker 타입임.
        # class_id와 score를 함께 표시하면 false positive/debug 확인에 유용함.
        # ---------------------------------------------------------------------
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.ns = "camera_obstacle_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        # TEXT marker는 scale.z가 글자 높이 역할을 함.
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"{class_id} {score:.2f}"
        marker.lifetime.sec = 1
        return marker

    def make_delete_all_marker(self, stamp):
        # ---------------------------------------------------------------------
        # RViz marker 전체 삭제 요청을 만드는 helper 함수임.
        # ---------------------------------------------------------------------
        # detection이 사라졌는데 이전 cylinder/text가 계속 남아 있으면 사용자가 착각할 수 있음.
        # DELETEALL action은 해당 marker display에서 남아 있는 marker를 지우는 데 사용함.
        # ---------------------------------------------------------------------
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.ns = "camera_obstacles"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker


def main(args=None):
    # rclpy 초기화함.
    rclpy.init(args=args)
    node = CameraObstacleNode()

    try:
        # subscriber callback을 계속 처리함.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 종료 처리함.
        pass
    finally:
        # 노드 자원 정리 후 rclpy 종료함.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
