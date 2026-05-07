#!/usr/bin/env python3

# 이 파일은 YOLO로 얻은 2D bounding box와 RGB-D 카메라의 depth image를 결합해서
# 객체의 3D 위치를 추정하는 ROS2 perception 노드임
#
# 전체 데이터 흐름:
#   /detection/objects      (YOLO 2D bbox 결과)
#   /camera/depth           (픽셀별 거리 정보)
#   /camera/camera_info     (카메라 내부 파라미터 fx, fy, cx, cy)
#       ↓
#   object_3d_projector_node
#       ↓
#   /detection/objects_3d   (camera_link 또는 camera optical frame 기준 3D detection)
#
# 이 버전은 초기 디버깅용 구조임
# bbox 중심 주변 ROI depth를 사용해 3D 위치를 계산하고,
# bbox 내부 여러 샘플 depth도 출력해서 "중심 픽셀이 배경을 찍는 문제"를 확인할 수 있게 함
#
# 주의:
# - 이 파일은 최종 base_link 변환/foreground percentile 버전 이전의 디버깅 버전 성격임
# - 그래도 RGB-D 기반 2D→3D projection 원리를 공부하기 좋음
# - 코드 로직은 유지하고 주석만 추가함
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2DArray,
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
)
from cv_bridge import CvBridge


# rclpy 기반 ROS2 노드 클래스임
# Node를 상속하면 publisher/subscriber/parameter/logger 등을 사용할 수 있음
#
# 이 노드의 역할:
#   1. YOLO가 발행한 Detection2DArray를 받음
#   2. 각 bbox 중심 픽셀 주변에서 depth 값을 읽음
#   3. CameraInfo의 intrinsic K 행렬로 pixel 좌표를 camera 3D 좌표로 역투영함
#   4. Detection3DArray 메시지로 변환해 발행함
#
# 로보틱스 관점:
#   perception detector가 "무엇이 보이는지"를 알려준다면,
#   이 projector는 "그 물체가 카메라 기준 어디에 있는지"를 알려주는 계층임
class Object3DProjectorNode(Node):
    def __init__(self):
        super().__init__("object_3d_projector_node")

        # ROS2 parameter를 쓰는 이유:
        #   토픽명, depth 필터 범위, debug 옵션 등을 코드 수정 없이 launch/yaml에서 바꿀 수 있게 하기 위함임
        #   실험 조건이 자주 바뀌는 perception pipeline에서는 parameter화가 매우 중요함

        # 입력/출력 토픽 파라미터 선언함
        # YOLO 2D detection, depth image, camera_info를 받아서 3D detection으로 변환하기 위함임
        #
        # detections_topic:
        #   yolo_detector_node가 발행하는 vision_msgs/Detection2DArray 토픽임
        #   class_id, confidence, bbox center, bbox size가 들어 있음
        #
        # depth_topic:
        #   Isaac Sim Camera Helper에서 발행하는 depth image 토픽임
        #   각 픽셀마다 카메라로부터의 거리값을 가짐
        #
        # camera_info_topic:
        #   카메라 내부 파라미터를 담는 토픽임
        #   2D pixel을 3D point로 바꾸려면 fx, fy, cx, cy가 필요함
        #
        # output_topic:
        #   계산된 3D detection을 발행하는 토픽임
        #   이 버전에서는 camera frame 기준 위치가 들어감
        self.declare_parameter("detections_topic", "/detection/objects")
        self.declare_parameter("depth_topic", "/camera/depth")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("output_topic", "/detection/objects_3d")

        # bbox 중심 1픽셀만 depth로 쓰면 노이즈나 빈 공간에 매우 취약함
        # 예:
        #   사람의 bbox 중심이 다리 사이, 팔 사이, 배경, 반사된 벽에 걸리면 depth가 실제 사람 거리가 아니라 배경 거리가 됨
        # 그래서 중심 주변 작은 ROI에서 유효 depth들의 median을 사용함
        # median은 평균(mean)보다 outlier에 강해서 depth noise 완화에 유리함
        self.declare_parameter("roi_radius", 3)

        # depth 유효 범위 설정함
        # 너무 가까운 값, 너무 먼 값, inf/nan은 버리기 위함임
        #
        # min_depth:
        #   0 또는 센서 바로 앞의 잘못된 값 제거용임
        #
        # max_depth:
        #   카메라가 볼 수는 있지만 로봇 회피/피킹에 의미 없는 먼 배경값 제거용임
        self.declare_parameter("min_depth", 0.05)
        self.declare_parameter("max_depth", 20.0)

        # 로그 출력 주기 제한용임
        self.declare_parameter("log_period", 1.0)

        # 디버그 로그 활성화 여부임
        # true이면 bbox 중심 픽셀, ROI depth 통계, camera_info, depth encoding 등을 출력함
        #
        # 이 로그는 다음 문제를 구분하는 데 도움됨:
        #   - depth image 자체가 안 들어오는 문제
        #   - camera_info K 행렬이 이상한 문제
        #   - bbox 중심 픽셀이 배경을 찍는 문제
        #   - timestamp가 depth와 detection 사이에 크게 어긋나는 문제
        self.declare_parameter("debug", True)

        # bbox 중심점이 사람/물체 표면이 아니라 배경을 찍는지 확인하기 위한 추가 샘플링 옵션임
        # true이면 bbox 중심뿐 아니라 bbox 내부 여러 지점의 depth도 같이 출력함
        self.declare_parameter("debug_bbox_samples", True)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.roi_radius = int(self.get_parameter("roi_radius").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.debug = bool(self.get_parameter("debug").value)
        self.debug_bbox_samples = bool(self.get_parameter("debug_bbox_samples").value)

        # CvBridge는 ROS Image 메시지와 OpenCV/numpy 배열 사이 변환을 담당함
        # ROS의 sensor_msgs/Image는 bytes buffer 형태라서 바로 numpy 연산을 하기가 불편함
        # depth crop, median, min/max/std 계산을 위해 numpy array로 변환해야 함
        self.bridge = CvBridge()

        # 최신 depth image와 camera_info를 캐싱해두고 detection callback에서 사용함
        #
        # 현재 버전은 간단한 prototype이므로 timestamp 동기화 대신 latest cache 방식 사용함
        # 즉, detection callback이 들어온 순간의 "가장 최근 depth"를 사용함
        #
        # 실무 고도화 단계에서는 message_filters로 RGB/depth/camera_info 동기화하는 게 더 좋음
        # 이유:
        #   사람이 움직이는 장면에서는 detection과 depth의 시간이 다르면 위치 추정 오차가 생길 수 있음
        #   ApproximateTimeSynchronizer 같은 방식으로 시간 차이를 제한하는 것이 더 안정적임
        self.latest_depth = None
        self.latest_depth_encoding = None
        self.latest_depth_stamp = None
        self.latest_depth_frame_id = None
        self.latest_camera_info = None
        self.latest_camera_info_stamp = None

        self.last_log_time = 0.0
        self.last_warn_time = 0.0

        # Isaac Sim sensor topic은 BEST_EFFORT QoS인 경우가 많음
        # QoS가 맞지 않으면 topic list에는 보여도 callback이 안 들어올 수 있음
        #
        # ROS2 QoS 개념:
        #   RELIABLE:
        #     메시지 전달을 최대한 보장하지만, 센서 고주파 데이터에서는 지연/버퍼 문제가 생길 수 있음
        #   BEST_EFFORT:
        #     일부 메시지가 유실될 수 있지만 최신 센서 데이터를 빠르게 받기 좋음
        #
        # 카메라/depth/LiDAR 같은 센서 데이터는 보통 최신값이 중요하므로 BEST_EFFORT가 자연스러움
        self.sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # depth image 구독자임
        # /camera/depth가 들어올 때마다 depth_callback이 호출되고 최신 depth를 저장함
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            self.sensor_qos,
        )

        # camera_info 구독자임
        # camera_info는 카메라 intrinsic 정보를 제공함
        # 해상도나 카메라 모델이 바뀌면 K 행렬도 바뀔 수 있으므로 토픽으로 받음
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            self.sensor_qos,
        )

        # YOLO 2D detection 구독자임
        # 이 callback에서 depth와 camera_info를 결합해 최종 3D detection을 계산함
        self.detections_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        # 3D detection 발행자임
        # 출력 메시지는 vision_msgs/Detection3DArray이며,
        # 각 Detection3D에는 class/confidence와 3D 위치가 들어감
        self.objects_3d_pub = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            10,
        )

        self.get_logger().info("Object 3D projector node started")
        self.get_logger().info(f"Subscribe detections : {self.detections_topic}")
        self.get_logger().info(f"Subscribe depth      : {self.depth_topic}")
        self.get_logger().info(f"Subscribe camera info: {self.camera_info_topic}")
        self.get_logger().info(f"Publish              : {self.output_topic}")
        self.get_logger().info(
            f"Debug={self.debug}, bbox_samples={self.debug_bbox_samples}, "
            f"roi_radius={self.roi_radius}, depth_range=[{self.min_depth}, {self.max_depth}] m"
        )

    def depth_callback(self, msg: Image):
        # depth image를 수신하는 callback임
        # 이 함수에서는 계산을 많이 하지 않고, 최신 depth image와 메타데이터를 저장만 함
        # 실제 bbox별 depth 추정은 detections_callback에서 수행함
        try:
            # depth image는 32FC1, 16UC1 등 encoding이 다를 수 있으므로 passthrough로 받음
            #
            # 32FC1:
            #   32-bit float, channel 1개
            #   보통 meter 단위 depth가 그대로 들어옴
            #
            # 16UC1:
            #   16-bit unsigned integer, channel 1개
            #   RealSense 등에서는 mm 단위로 들어오는 경우가 흔함
            #
            # passthrough:
            #   encoding을 임의로 바꾸지 않고 원본 값을 보존함
            #   depth를 bgr8 같은 영상 encoding으로 바꾸면 거리값이 망가짐
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # numpy array로 저장함
            # 이후 ROI crop, finite mask, median/std 계산을 numpy로 빠르게 처리하기 위함임
            self.latest_depth = np.array(depth_image)
            self.latest_depth_encoding = msg.encoding
            self.latest_depth_stamp = msg.header.stamp
            self.latest_depth_frame_id = msg.header.frame_id

        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        # CameraInfo를 수신하는 callback임
        # CameraInfo에는 intrinsic matrix K가 포함되어 있음
        #
        # K = [fx, 0,  cx,
        #      0,  fy, cy,
        #      0,  0,  1]
        #
        # fx, fy:
        #   focal length를 pixel 단위로 나타낸 값
        # cx, cy:
        #   principal point, 보통 이미지 중심 근처임
        self.latest_camera_info = msg
        self.latest_camera_info_stamp = msg.header.stamp

    def detections_callback(self, msg: Detection2DArray):
        # YOLO 2D detection을 받아 3D detection으로 변환하는 메인 callback임
        #
        # 처리 순서:
        #   1. 최신 depth image가 있는지 확인함
        #   2. 최신 camera_info가 있는지 확인함
        #   3. camera intrinsic fx/fy/cx/cy를 읽음
        #   4. 각 detection bbox 중심 픽셀 주변 ROI에서 depth median을 구함
        #   5. pinhole camera model로 3D point를 계산함
        #   6. Detection3DArray로 발행함
        if self.latest_depth is None:
            self.throttled_warn("Depth image not received yet")
            return

        if self.latest_camera_info is None:
            self.throttled_warn("CameraInfo not received yet")
            return

        # 카메라 내부 파라미터 가져옴
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        #
        # pinhole camera model:
        #   u = fx * X/Z + cx
        #   v = fy * Y/Z + cy
        #
        # 역투영:
        #   X = (u - cx) * Z / fx
        #   Y = (v - cy) * Z / fy
        #   Z = depth
        #
        # 여기서 u, v는 이미지 픽셀 좌표이고,
        # X, Y, Z는 camera optical frame 기준 3D 좌표임
        k = self.latest_camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.throttled_warn(f"Invalid camera intrinsic fx/fy: fx={fx}, fy={fy}")
            return

        # 출력 메시지 생성함
        # header는 입력 detection의 header를 그대로 사용함
        # frame_id가 camera_link 또는 camera_rgb_optical_frame으로 유지되어야 좌표계 해석이 가능함
        out_msg = Detection3DArray()
        out_msg.header = msg.header

        debug_records = []

        for det2d in msg.detections:
            if len(det2d.results) == 0:
                continue

            # YOLO bbox 중심과 크기를 읽음
            # u, v:
            #   이미지 좌표계의 bbox 중심 픽셀
            # bbox_w, bbox_h:
            #   bbox의 가로/세로 크기(pixel)
            u = float(det2d.bbox.center.position.x)
            v = float(det2d.bbox.center.position.y)
            bbox_w = float(det2d.bbox.size_x)
            bbox_h = float(det2d.bbox.size_y)

            # bbox 중심 픽셀 주변 ROI에서 depth 통계값을 계산함
            # depth_info["median"]을 최종 깊이로 사용함
            # 이 방식은 중심 1픽셀보다 안정적이지만,
            # bbox 중심이 배경을 찍으면 ROI median도 배경을 따라갈 수 있음
            depth_info = self.get_depth_info_at_pixel(u, v)
            if depth_info is None:
                debug_records.append(
                    {
                        "valid": False,
                        "label": self.get_label(det2d),
                        "score": self.get_score(det2d),
                        "u": u,
                        "v": v,
                        "bbox_w": bbox_w,
                        "bbox_h": bbox_h,
                        "reason": "no valid depth at bbox center ROI",
                    }
                )
                continue

            depth_m = depth_info["median"]

            # pinhole camera model 기반 역투영임
            # 여기서 x/y/z는 optical frame 관례를 가정함
            #   x: 이미지 오른쪽 방향
            #   y: 이미지 아래 방향
            #   z: 카메라 전방 깊이 방향
            #
            # 이 좌표는 아직 로봇 base_link 기준이 아님
            # base_link 기준으로 사용하려면 TF 변환이 추가로 필요함
            x = (u - cx) * depth_m / fx
            y = (v - cy) * depth_m / fy
            z = depth_m

            det3d = Detection3D()
            det3d.header = msg.header

            # class/confidence 그대로 복사하고, hypothesis pose에도 3D 위치를 넣어둠
            #
            # ObjectHypothesisWithPose 구조:
            #   hypothesis.class_id : "person", "box" 같은 클래스 이름
            #   hypothesis.score    : confidence
            #   pose.pose.position  : 이 객체의 3D 위치 추정값
            #
            # 이렇게 해두면 downstream node가 class와 위치를 함께 사용할 수 있음
            for result in det2d.results:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = result.hypothesis.class_id
                hyp.hypothesis.score = result.hypothesis.score
                hyp.pose.pose.position.x = x
                hyp.pose.pose.position.y = y
                hyp.pose.pose.position.z = z
                hyp.pose.pose.orientation.w = 1.0
                det3d.results.append(hyp)

            # Detection3D의 bbox 중심에도 3D 위치를 넣음
            # 실제 3D bounding box 크기는 아직 모르므로 size는 0으로 둠
            #
            # size를 0으로 두는 이유:
            #   현재는 객체의 정확한 3D 크기/방향을 추정하지 않음
            #   단지 "대표 위치"만 필요함
            #
            # 추후 확장:
            #   depth crop이나 segmentation mask를 이용해 3D bbox size를 추정할 수 있음
            det3d.bbox.center.position.x = x
            det3d.bbox.center.position.y = y
            det3d.bbox.center.position.z = z
            det3d.bbox.center.orientation.w = 1.0
            det3d.bbox.size.x = 0.0
            det3d.bbox.size.y = 0.0
            det3d.bbox.size.z = 0.0

            out_msg.detections.append(det3d)

            # bbox 내부 여러 지점의 depth를 샘플링함
            # 이 값은 최종 3D 위치 계산에 직접 쓰이지 않고,
            # bbox 중심 depth가 실제 객체인지 배경인지 판단하기 위한 디버그 용도임
            sample_info = None
            if self.debug_bbox_samples:
                sample_info = self.sample_depths_inside_bbox(u, v, bbox_w, bbox_h)

            debug_records.append(
                {
                    "valid": True,
                    "label": self.get_label(det2d),
                    "score": self.get_score(det2d),
                    "u": u,
                    "v": v,
                    "bbox_w": bbox_w,
                    "bbox_h": bbox_h,
                    "depth_info": depth_info,
                    "sample_info": sample_info,
                    "xyz": (x, y, z),
                }
            )

        # 계산된 Detection3DArray를 발행함
        # detection이 하나도 유효하지 않으면 빈 Detection3DArray가 발행될 수 있음
        self.objects_3d_pub.publish(out_msg)

        now = time.time()
        if now - self.last_log_time >= self.log_period:
            self.print_debug_log(
                out_msg=out_msg,
                debug_records=debug_records,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                detection_header=msg.header,
            )
            self.last_log_time = now

    def get_depth_info_at_pixel(self, u: float, v: float):
        # 특정 픽셀(u, v) 주변 ROI에서 depth 통계값을 계산하는 함수임
        #
        # 입력:
        #   u, v: 이미지 픽셀 좌표
        #
        # 출력:
        #   median, mean, min, max, std, valid_count 등 depth 품질 정보
        #
        # 이 함수는 bbox 중심 주변 작은 영역을 보기 때문에 계산이 빠르고 단순함
        # 단, bbox 중심이 배경을 찍는 경우에는 근본적으로 실패할 수 있음
        if self.latest_depth is None:
            return None

        # depth image 크기 확인함
        # h: 이미지 높이, w: 이미지 너비
        h, w = self.latest_depth.shape[:2]

        # float 픽셀 좌표를 가장 가까운 integer 픽셀 인덱스로 변환함
        # YOLO bbox 중심은 float로 나올 수 있으므로 round를 사용함
        ui = int(round(u))
        vi = int(round(v))

        if ui < 0 or ui >= w or vi < 0 or vi >= h:
            return None

        # ROI 반경임
        # roi_radius=3이면 중심 포함 7x7 영역을 확인함
        # 작은 영역을 보는 이유는 bbox 중심 근처의 depth noise를 median으로 완화하기 위함임
        r = self.roi_radius

        x_min = max(0, ui - r)
        x_max = min(w, ui + r + 1)
        y_min = max(0, vi - r)
        y_max = min(h, vi + r + 1)

        # depth image에서 중심 주변 ROI를 잘라냄
        # numpy slicing은 [row, col] = [v, u] 순서임
        roi_raw = self.latest_depth[y_min:y_max, x_min:x_max]
        roi_m = self.depth_to_meter_array(roi_raw)

        # NaN/Inf가 아닌 depth만 먼저 선택함
        # 시뮬레이터/센서에서는 invalid depth가 NaN, Inf, 0, max range 등으로 표현될 수 있음
        finite = roi_m[np.isfinite(roi_m)]
        # 실제로 사용할 depth 범위만 선택함
        # 너무 작은 값이나 너무 먼 배경값은 제외함
        valid = finite[(finite > self.min_depth) & (finite < self.max_depth)]

        if valid.size == 0:
            return None

        # ROI median과 별도로 중심 픽셀 depth도 기록함
        # 디버깅 시 center_raw/center_m과 roi median을 비교하면 중심 픽셀이 튀는지 확인 가능함
        center_raw = self.latest_depth[vi, ui]
        center_m = self.depth_to_meter_value(center_raw)

        return {
            "median": float(np.median(valid)),
            "mean": float(np.mean(valid)),
            "min": float(np.min(valid)),
            "max": float(np.max(valid)),
            "std": float(np.std(valid)),
            "valid_count": int(valid.size),
            "total_count": int(roi_m.size),
            "valid_ratio": float(valid.size / max(1, roi_m.size)),
            "center_raw": float(center_raw) if np.isscalar(center_raw) else center_raw,
            "center_m": center_m,
            "roi_bounds": (x_min, x_max, y_min, y_max),
            "image_size": (w, h),
        }

    def sample_depths_inside_bbox(self, u: float, v: float, bbox_w: float, bbox_h: float):
        # bbox 안의 대표 지점 몇 곳에서 depth를 샘플링하는 디버그 함수임
        #
        # 왜 필요한가?
        #   사람 bbox 중심은 항상 사람 몸 표면이 아닐 수 있음
        #   예를 들어 팔을 벌린 사람, 다리 사이 공간, 반사된 벽 등이 bbox 안에 섞일 수 있음
        #
        # 이 함수는 center/upper/lower/left/right depth를 비교해서
        # "중심 depth가 배경인지"를 빠르게 확인할 수 있게 함
        # bbox 중심 depth가 배경을 찍는지 확인하기 위한 샘플링임
        # 사람 bbox에서는 중심점이 몸통이 아니라 빈 공간/배경에 걸릴 수 있으므로 여러 점을 확인함
        # bbox 내부 5개 지점 정의함
        # upper/lower/left/right는 bbox 중심에서 bbox 크기의 25%만큼 이동한 지점임
        points = {
            "center": (u, v),
            "upper": (u, v - 0.25 * bbox_h),
            "lower": (u, v + 0.25 * bbox_h),
            "left": (u - 0.25 * bbox_w, v),
            "right": (u + 0.25 * bbox_w, v),
        }

        samples = {}
        for name, (px, py) in points.items():
            info = self.get_depth_info_at_pixel(px, py)
            if info is None:
                samples[name] = None
            else:
                samples[name] = info["median"]

        # None이 아닌 depth 샘플만 모음
        # robust_median은 디버그 참고값이며, 최종 출력 위치 계산에는 사용하지 않음
        valid_values = [value for value in samples.values() if value is not None]

        if len(valid_values) == 0:
            robust_median = None
        else:
            robust_median = float(np.median(np.array(valid_values, dtype=np.float32)))

        return {
            "samples": samples,
            "robust_median": robust_median,
        }

    def depth_to_meter_array(self, depth_array):
        # depth image crop을 meter 단위 numpy array로 변환하는 함수임
        #
        # 32FC1:
        #   이미 meter 단위 float라고 보고 그대로 사용함
        #
        # 16UC1:
        #   mm 단위로 들어오는 경우가 많아 1000으로 나눠 meter로 변환함
        # Isaac Sim depth가 32FC1이면 보통 m 단위 float로 들어옴
        # 16UC1이면 일반적으로 mm 단위일 수 있으므로 m로 변환함
        arr = depth_array.astype(np.float32)

        if self.latest_depth_encoding == "16UC1":
            arr = arr / 1000.0

        return arr

    def depth_to_meter_value(self, depth_value):
        # 단일 depth 값을 meter 단위 float로 변환하는 함수임
        # center pixel depth 같은 단일 값을 처리할 때 사용함
        try:
            value = float(depth_value)
        except Exception:
            return math.nan

        if self.latest_depth_encoding == "16UC1":
            value = value / 1000.0

        return value

    def get_label(self, det2d):
        # Detection2D에서 class label을 꺼내는 편의 함수임
        # results가 비어 있으면 unknown으로 처리함
        if len(det2d.results) == 0:
            return "unknown"
        return det2d.results[0].hypothesis.class_id

    def get_score(self, det2d):
        # Detection2D에서 confidence score를 꺼내는 편의 함수임
        # results가 비어 있으면 0.0으로 처리함
        if len(det2d.results) == 0:
            return 0.0
        return float(det2d.results[0].hypothesis.score)

    def stamp_to_float(self, stamp):
        # ROS timestamp(sec, nanosec)를 초 단위 float로 바꾸는 함수임
        # detection-depth 시간 차이를 계산하기 위함임
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def get_stamp_delta(self, stamp_a, stamp_b):
        # 두 timestamp의 차이를 초 단위로 계산함
        # detection stamp - depth stamp 값이 너무 크면 서로 다른 시점의 데이터를 섞고 있을 수 있음
        if stamp_a is None or stamp_b is None:
            return None
        return self.stamp_to_float(stamp_a) - self.stamp_to_float(stamp_b)

    def throttled_warn(self, text):
        # 경고 로그를 너무 자주 찍지 않도록 제한하는 함수임
        # 센서가 아직 안 들어오는 초기 구간에서 매 callback마다 warn이 찍히면 로그가 매우 지저분해짐
        now = time.time()
        if now - self.last_warn_time >= 1.0:
            self.get_logger().warn(text)
            self.last_warn_time = now

    def print_debug_log(self, out_msg, debug_records, fx, fy, cx, cy, detection_header):
        # 디버그 로그 출력 함수임
        #
        # 이 로그로 확인할 수 있는 것:
        #   - depth encoding이 32FC1인지 16UC1인지
        #   - depth frame과 detection frame이 일치하는지
        #   - camera intrinsic 값이 정상인지
        #   - detection-depth-camera_info timestamp 차이가 어느 정도인지
        #   - bbox 중심 depth와 주변 샘플 depth가 서로 다른지
        #   - 최종 camera_xyz가 거리 변화에 맞게 변하는지
        if len(debug_records) == 0:
            self.get_logger().info("3D detections: 0")
            return

        depth_shape = None
        if self.latest_depth is not None:
            depth_shape = self.latest_depth.shape

        det_depth_dt = self.get_stamp_delta(detection_header.stamp, self.latest_depth_stamp)
        det_info_dt = self.get_stamp_delta(detection_header.stamp, self.latest_camera_info_stamp)

        if self.debug:
            # 카메라/깊이/시간 동기화 관련 기본 정보를 출력함
            # det-depth dt가 0에 가까울수록 detection과 depth가 같은 시점의 데이터에 가까움
            self.get_logger().info(
                "DEBUG camera/depth | "
                f"depth_encoding={self.latest_depth_encoding}, "
                f"depth_frame={self.latest_depth_frame_id}, "
                f"depth_shape={depth_shape}, "
                f"det_frame={detection_header.frame_id}, "
                f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}, "
                f"det-depth dt={det_depth_dt}, det-info dt={det_info_dt}"
            )

        if len(out_msg.detections) == 0:
            self.get_logger().info("3D detections: 0 valid after depth filtering")

        for rec in debug_records:
            # depth가 없거나 유효하지 않아 3D projection을 skip한 detection 로그임
            if not rec["valid"]:
                self.get_logger().info(
                    f"3D detection skipped | class={rec['label']}, score={rec['score']:.2f}, "
                    f"pixel=({rec['u']:.1f}, {rec['v']:.1f}), "
                    f"bbox=({rec['bbox_w']:.1f} x {rec['bbox_h']:.1f}), "
                    f"reason={rec['reason']}"
                )
                continue

            x, y, z = rec["xyz"]
            depth_info = rec["depth_info"]
            sample_info = rec["sample_info"]

            # 유효한 3D detection 로그임
            # depth_median:
            #   ROI median depth이며 현재 위치 추정에 사용된 값임
            # depth_center:
            #   bbox 중심 픽셀의 depth임
            # camera_xyz:
            #   pinhole 역투영으로 계산한 camera optical frame 기준 3D 좌표임
            self.get_logger().info(
                f"3D detection | class={rec['label']}, score={rec['score']:.2f}, "
                f"pixel=({rec['u']:.1f}, {rec['v']:.1f}), "
                f"bbox=({rec['bbox_w']:.1f} x {rec['bbox_h']:.1f}), "
                f"depth_median={depth_info['median']:.3f} m, "
                f"depth_center={depth_info['center_m']:.3f} m, "
                f"valid={depth_info['valid_count']}/{depth_info['total_count']} "
                f"({depth_info['valid_ratio']:.2f}), "
                f"depth_min/max/std=({depth_info['min']:.3f}, {depth_info['max']:.3f}, {depth_info['std']:.3f}), "
                f"camera_xyz=({x:.3f}, {y:.3f}, {z:.3f}) m"
            )

            # bbox 내부 여러 지점의 depth를 출력함
            # center와 upper/left/right가 크게 다르면 bbox 중심이 배경을 찍고 있을 가능성이 큼
            if self.debug and sample_info is not None:
                samples = sample_info["samples"]
                self.get_logger().info(
                    "DEBUG bbox depth samples | "
                    f"center={self.format_depth(samples.get('center'))}, "
                    f"upper={self.format_depth(samples.get('upper'))}, "
                    f"lower={self.format_depth(samples.get('lower'))}, "
                    f"left={self.format_depth(samples.get('left'))}, "
                    f"right={self.format_depth(samples.get('right'))}, "
                    f"robust_median={self.format_depth(sample_info.get('robust_median'))}"
                )

    def format_depth(self, value):
        # depth 값을 로그 문자열로 포맷하는 함수임
        # None이면 "None"으로 표시해 invalid depth를 쉽게 확인하게 함
        if value is None:
            return "None"
        return f"{value:.3f} m"


def main(args=None):
    # ROS2 Python 노드의 표준 실행 진입점임
    # rclpy.init()으로 ROS2 통신을 초기화하고,
    # Object3DProjectorNode를 만든 뒤 rclpy.spin()으로 callback이 계속 실행되게 함
    # Ctrl+C가 들어오면 KeyboardInterrupt를 통해 정상 종료함
    rclpy.init(args=args)
    node = Object3DProjectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
