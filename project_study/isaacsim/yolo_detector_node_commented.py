#!/usr/bin/env python3

# =============================================================================
# yolo_detector_node.py
# =============================================================================
# 역할:
#   Isaac Sim 카메라에서 발행되는 ROS2 Image 토픽(/camera/image_raw)을 구독하고,
#   Ultralytics YOLO 모델로 2D 객체 검출을 수행한 뒤,
#   1) 사람이 눈으로 확인할 수 있는 bbox 시각화 이미지(/detection/image)
#   2) 다른 ROS2 노드가 사용할 수 있는 구조화된 2D 검출 결과(/detection/objects)
#   를 발행하는 perception front-end 노드임.
#
# 전체 데이터 흐름:
#   /camera/image_raw
#       -> CvBridge로 ROS Image를 OpenCV 이미지로 변환함
#       -> YOLOv8m inference 수행함
#       -> bbox가 그려진 이미지 발행함
#       -> vision_msgs/Detection2DArray 형태로 bbox/class/confidence 발행함
#
# 이 노드의 위치:
#   W4 perception pipeline에서 가장 앞단에 있는 AI inference 노드임.
#   이 노드는 객체의 3D 위치를 계산하지 않음.
#   3D 위치 계산은 후단 object_3d_projector_node.py에서 depth + camera_info와 결합해 수행함.
#
# 실무 관점:
#   제어/MPC/CBF 같은 실시간 제어부는 C++/rclcpp로 작성하는 경우가 많지만,
#   YOLO, PyTorch, Ultralytics 같은 AI inference는 Python 생태계가 훨씬 빠르고 편함.
#   그래서 이 프로젝트에서는 perception inference를 rclpy로 구현하고,
#   후단 navigation/control stack과 ROS2 topic으로 연결하는 구조를 사용함.
# =============================================================================

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)


class YoloDetectorNode(Node):
    def __init__(self):
        # ROS2 노드 이름을 yolo_detector_node로 설정함.
        # launch 파일이나 ros2 node list에서 이 이름으로 확인 가능함.
        super().__init__("yolo_detector_node")

        # ---------------------------------------------------------------------
        # ROS parameter 선언부
        # ---------------------------------------------------------------------
        # ROS2에서는 declare_parameter()로 파라미터를 먼저 선언해야
        # launch 파일이나 yaml 파일에서 값을 주입받을 수 있음.
        # 이렇게 해두면 코드 수정 없이 모델, topic, threshold, FPS 등을 바꿀 수 있음.
        # 예:
        #   ros2 launch perception perception.launch.py
        #   config/yolo.yaml에서 model_path, confidence_threshold 등을 변경 가능함.
        # ---------------------------------------------------------------------

        # 입력 RGB 이미지 토픽임.
        # Isaac Sim ROS2 Camera Helper에서 발행하는 카메라 이미지가 들어옴.
        self.declare_parameter("image_topic", "/camera/image_raw")

        # bbox가 그려진 시각화 이미지를 발행할 토픽임.
        # rqt_image_view나 RViz Image display에서 확인하기 위한 용도임.
        self.declare_parameter("output_image_topic", "/detection/image")

        # 구조화된 객체 검출 결과를 발행할 토픽임.
        # 후단 object_3d_projector_node가 이 토픽을 구독해서 depth와 결합함.
        self.declare_parameter("output_objects_topic", "/detection/objects")

        # 사용할 YOLO 모델 가중치 파일임.
        # yolov8n.pt는 빠르지만 검출력이 약하고, yolov8m.pt는 더 무겁지만 정확도가 나음.
        # 현재 프로젝트에서는 물류창고 사람/박스 검출 안정성을 위해 yolov8m.pt 사용함.
        self.declare_parameter("model_path", "yolov8m.pt")

        # YOLO confidence threshold임.
        # 이 값보다 낮은 confidence의 검출 결과는 YOLO 후처리에서 제거됨.
        # 값이 낮으면 false positive가 늘 수 있고, 값이 높으면 놓치는 객체가 늘 수 있음.
        self.declare_parameter("confidence_threshold", 0.45)

        # NMS에서 사용하는 IoU threshold임.
        # 같은 객체에 여러 bbox가 겹쳐 나왔을 때 얼마나 겹치면 하나로 정리할지 결정함.
        # IoU는 Intersection over Union의 약자임.
        self.declare_parameter("iou_threshold", 0.45)

        # YOLO 입력 이미지 크기임.
        # 640은 일반적으로 정확도와 속도의 균형이 좋은 기본값임.
        self.declare_parameter("image_size", 640)

        # inference device임.
        # "cpu"면 CPU 사용, "0"이면 첫 번째 CUDA GPU 사용함.
        # RTX GPU를 쓸 수 있다면 "0"이 보통 훨씬 빠름.
        self.declare_parameter("device", "0")

        # YOLO 처리 최대 FPS임.
        # 카메라가 50Hz로 들어와도 모든 프레임에 inference를 돌리면 부담이 큼.
        # 따라서 max_fps로 inference 호출 빈도를 제한함.
        self.declare_parameter("max_fps", 10.0)

        # ---------------------------------------------------------------------
        # parameter 값을 실제 멤버 변수로 읽어옴.
        # ---------------------------------------------------------------------
        # get_parameter().value로 값을 읽음.
        # yaml에서 주입된 값이 있으면 그 값이 들어오고, 없으면 위 default가 들어옴.
        # ---------------------------------------------------------------------
        self.image_topic = self.get_parameter("image_topic").value
        self.output_image_topic = self.get_parameter("output_image_topic").value
        self.output_objects_topic = self.get_parameter("output_objects_topic").value

        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.device = self.get_parameter("device").value
        self.max_fps = float(self.get_parameter("max_fps").value)

        # ---------------------------------------------------------------------
        # CvBridge 객체 생성함.
        # ---------------------------------------------------------------------
        # ROS의 sensor_msgs/Image는 OpenCV가 바로 처리할 수 있는 numpy 이미지가 아님.
        # CvBridge는 ROS Image <-> OpenCV 이미지(np.ndarray) 변환을 담당함.
        #
        # 여기서 중요한 encoding:
        #   - ROS Image는 rgb8, bgr8, rgba8 등 다양한 encoding을 가질 수 있음.
        #   - OpenCV는 기본적으로 BGR 순서를 많이 사용함.
        #   - YOLO/Ultralytics는 numpy image를 받아 내부에서 처리 가능함.
        # ---------------------------------------------------------------------
        self.bridge = CvBridge()

        # ---------------------------------------------------------------------
        # YOLO 모델 로드함.
        # ---------------------------------------------------------------------
        # Ultralytics YOLO는 Python에서 model = YOLO("yolov8m.pt")처럼 쉽게 로드 가능함.
        # 모델 파일이 로컬에 없으면 최초 실행 시 자동 다운로드될 수 있음.
        # 실험 재현성을 높이려면 나중에는 /home/lyj/models/yolov8m.pt처럼 절대 경로로 고정 가능함.
        # ---------------------------------------------------------------------
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLO model loaded")

        # ---------------------------------------------------------------------
        # QoS 설정함.
        # ---------------------------------------------------------------------
        # ROS2 QoS(Quality of Service)는 publisher/subscriber 사이의 통신 정책임.
        # Isaac Sim 카메라 같은 sensor data는 보통 BEST_EFFORT로 발행되는 경우가 많음.
        # subscriber가 RELIABLE 기본 QoS를 쓰면 토픽은 보이는데 callback이 안 들어올 수 있음.
        # 그래서 sensor_qos를 BEST_EFFORT + KEEP_LAST(depth=1)로 맞춤.
        #
        # depth=1의 의미:
        #   오래된 이미지를 쌓아두지 않고 최신 프레임 위주로 처리함.
        #   실시간 perception에서는 오래된 이미지보다 최신 이미지가 중요함.
        # ---------------------------------------------------------------------
        self.sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---------------------------------------------------------------------
        # 입력 이미지 subscriber 생성함.
        # ---------------------------------------------------------------------
        # /camera/image_raw가 들어올 때마다 image_callback()이 호출됨.
        # 콜백 안에서 FPS 제한, cv_bridge 변환, YOLO inference, 결과 발행을 모두 수행함.
        # ---------------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.sensor_qos,
        )

        # ---------------------------------------------------------------------
        # bbox 시각화 이미지 publisher 생성함.
        # ---------------------------------------------------------------------
        # 사람이 디버깅할 때 rqt_image_view로 확인하는 용도임.
        # 이 토픽은 navigation/control 노드가 직접 쓰기보다는 시각화용임.
        # ---------------------------------------------------------------------
        self.image_pub = self.create_publisher(
            Image,
            self.output_image_topic,
            10,
        )

        # ---------------------------------------------------------------------
        # Detection2DArray publisher 생성함.
        # ---------------------------------------------------------------------
        # 후단 노드는 이미지 위에 그려진 bbox가 아니라 구조화된 숫자 데이터가 필요함.
        # Detection2DArray에는 각 객체별 bbox center, bbox size, class_id, confidence가 들어감.
        # ---------------------------------------------------------------------
        self.objects_pub = self.create_publisher(
            Detection2DArray,
            self.output_objects_topic,
            10,
        )

        # ---------------------------------------------------------------------
        # FPS 제한 및 로그용 상태 변수임.
        # ---------------------------------------------------------------------
        # last_process_time:
        #   마지막으로 YOLO inference를 수행한 wall-clock time임.
        # frame_count:
        #   로그 주기 동안 몇 프레임을 처리했는지 셈.
        # last_log_time:
        #   마지막으로 FPS 로그를 출력한 시간임.
        # ---------------------------------------------------------------------
        self.last_process_time = 0.0
        self.frame_count = 0
        self.last_log_time = time.time()

        self.get_logger().info("YOLO detector node started")
        self.get_logger().info(f"Subscribe: {self.image_topic}")
        self.get_logger().info(f"Publish image  : {self.output_image_topic}")
        self.get_logger().info(f"Publish objects: {self.output_objects_topic}")
        self.get_logger().info(f"Model          : {self.model_path}")
        self.get_logger().info(f"Device         : {self.device}")
        self.get_logger().info(f"Max FPS        : {self.max_fps}")

    def image_callback(self, msg: Image):
        # ---------------------------------------------------------------------
        # 카메라 이미지가 들어올 때마다 호출되는 callback임.
        # ---------------------------------------------------------------------
        # 처리 순서:
        #   1. max_fps 기준으로 너무 빠른 프레임은 skip함
        #   2. ROS Image를 OpenCV 이미지로 변환함
        #   3. YOLO inference 수행함
        #   4. bbox가 그려진 이미지를 /detection/image로 발행함
        #   5. 구조화된 Detection2DArray를 /detection/objects로 발행함
        # ---------------------------------------------------------------------
        now = time.time()

        # ---------------------------------------------------------------------
        # inference FPS 제한함.
        # ---------------------------------------------------------------------
        # Isaac Sim 카메라가 약 50Hz로 들어오더라도 YOLOv8m을 모든 프레임에 돌릴 필요는 없음.
        # W4/W5 목표는 실시간성이 어느 정도 있는 perception pipeline 구축임.
        # 따라서 max_fps=10이면 최소 0.1초 간격으로만 inference 수행함.
        # skipped frame은 그냥 버림. 최신 프레임이 계속 들어오기 때문에 실시간 관점에서는 합리적임.
        # ---------------------------------------------------------------------
        if self.max_fps > 0.0:
            min_dt = 1.0 / self.max_fps
            if (now - self.last_process_time) < min_dt:
                return

        self.last_process_time = now

        try:
            # -----------------------------------------------------------------
            # ROS Image -> OpenCV BGR 이미지 변환함.
            # -----------------------------------------------------------------
            # desired_encoding="bgr8"을 지정해 OpenCV 표준 BGR 채널 순서로 받음.
            # 이 단계에서 encoding mismatch가 있으면 cv_bridge 예외가 발생할 수 있음.
            # 예외 발생 시 노드를 죽이지 않고 로그만 남기고 해당 프레임을 skip함.
            # -----------------------------------------------------------------
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        try:
            start_time = time.time()

            # -----------------------------------------------------------------
            # YOLO inference 수행함.
            # -----------------------------------------------------------------
            # source=cv_image:
            #   OpenCV numpy 이미지를 직접 입력함.
            # conf:
            #   confidence threshold임. 낮으면 더 많이 검출하나 false positive도 늘어남.
            # iou:
            #   NMS IoU threshold임. 중복 bbox 제거에 영향 줌.
            # imgsz:
            #   모델 입력 해상도임.
            # device:
            #   "0"이면 GPU 0번 사용, "cpu"면 CPU 사용함.
            # verbose=False:
            #   매 프레임마다 Ultralytics가 콘솔에 출력하는 것을 막음.
            # -----------------------------------------------------------------
            results = self.model.predict(
                source=cv_image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.image_size,
                device=self.device,
                verbose=False,
            )

            inference_time_ms = (time.time() - start_time) * 1000.0

            # Ultralytics predict 결과는 list 형태임.
            # 여기서는 한 장의 이미지에 대한 inference이므로 results[0]만 사용함.
            result = results[0]

            # -----------------------------------------------------------------
            # 1) bbox가 그려진 시각화 이미지 생성 및 발행함.
            # -----------------------------------------------------------------
            # result.plot()은 YOLO가 검출한 bbox, class name, confidence를 이미지 위에 그림.
            # 이 이미지는 사람이 rqt_image_view에서 확인하기 위한 debug/visualization output임.
            # -----------------------------------------------------------------
            annotated_image = result.plot()
            out_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")

            # 입력 이미지 header를 그대로 복사함.
            # stamp와 frame_id가 유지되어 후단에서 같은 시점/프레임의 이미지라는 것을 알 수 있음.
            out_img_msg.header = msg.header
            self.image_pub.publish(out_img_msg)

            # -----------------------------------------------------------------
            # 2) 구조화된 detection 결과 생성 및 발행함.
            # -----------------------------------------------------------------
            # bbox 이미지 자체는 사람이 보기엔 좋지만, 로봇 제어 노드가 쓰기엔 불편함.
            # 그래서 class, score, bbox center/size를 Detection2DArray로 발행함.
            # 후단 object_3d_projector_node는 이 값을 depth와 결합해 3D 위치를 계산함.
            # -----------------------------------------------------------------
            objects_msg = self.build_detection_msg(result, msg.header)
            self.objects_pub.publish(objects_msg)

            self.frame_count += 1

            # -----------------------------------------------------------------
            # 1초마다 처리 FPS와 inference time을 출력함.
            # -----------------------------------------------------------------
            # YOLO FPS:
            #   실제로 inference가 수행되어 output이 발행된 FPS임.
            # inference:
            #   단일 프레임에 대한 YOLO predict 소요 시간임.
            # detections:
            #   현재 프레임에서 threshold를 통과한 bbox 개수임.
            # -----------------------------------------------------------------
            if (now - self.last_log_time) >= 1.0:
                elapsed = now - self.last_log_time
                fps = self.frame_count / elapsed
                num_boxes = len(objects_msg.detections)

                self.get_logger().info(
                    f"YOLO FPS: {fps:.2f}, "
                    f"inference: {inference_time_ms:.1f} ms, "
                    f"detections: {num_boxes}"
                )

                self.frame_count = 0
                self.last_log_time = now

        except Exception as e:
            # YOLO inference나 결과 변환 중 예외가 나도 노드 전체를 죽이지 않음.
            # perception pipeline은 실시간으로 계속 들어오는 센서 데이터를 처리하므로,
            # 한 프레임 실패는 skip하고 다음 프레임을 처리하는 방식이 안전함.
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

    def build_detection_msg(self, result, header):
        # ---------------------------------------------------------------------
        # Ultralytics YOLO 결과를 vision_msgs/Detection2DArray로 변환함.
        # ---------------------------------------------------------------------
        # YOLO 내부 result.boxes에는 bbox 좌표, class index, confidence가 들어 있음.
        # ROS2 후단 노드가 모델 라이브러리에 의존하지 않게 하려면,
        # YOLO 고유 자료구조를 ROS 표준 메시지에 가까운 형태로 바꿔 발행하는 게 좋음.
        #
        # Detection2DArray 구성:
        #   header:
        #     image frame/time 정보 유지함.
        #   detections[]:
        #     Detection2D 객체 목록임.
        #   Detection2D.bbox:
        #     2D bbox 중심점과 크기임.
        #   Detection2D.results[]:
        #     class hypothesis와 confidence score가 들어감.
        # ---------------------------------------------------------------------
        detections_msg = Detection2DArray()
        detections_msg.header = header

        # boxes가 None이면 검출된 객체가 없다는 뜻임.
        # 빈 Detection2DArray를 그대로 발행함.
        if result.boxes is None:
            return detections_msg

        # result.names는 class index -> class name mapping임.
        # 예: {0: 'person', 1: 'bicycle', ...}
        names = result.names

        for box in result.boxes:
            # -----------------------------------------------------------------
            # xyxy format 읽음.
            # -----------------------------------------------------------------
            # YOLO bbox 좌표는 보통 xyxy 또는 xywh 형태로 제공됨.
            # 여기서는 xyxy를 사용함.
            #   x1, y1: 좌상단 픽셀 좌표
            #   x2, y2: 우하단 픽셀 좌표
            # 이미지 좌표계는 보통 왼쪽 위가 원점이고, u/x는 오른쪽, v/y는 아래쪽으로 증가함.
            # -----------------------------------------------------------------
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # class index와 confidence score 읽음.
            # box.cls, box.conf는 torch tensor일 수 있으므로 item()으로 Python scalar로 변환함.
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())

            # class index를 사람이 읽기 쉬운 class name으로 바꿈.
            # 혹시 names에 해당 id가 없으면 숫자 문자열로 fallback함.
            class_name = names.get(class_id, str(class_id))

            # Detection2D는 bbox 중심점과 bbox 크기를 사용함.
            # xyxy를 center/width/height로 변환함.
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            width = x2 - x1
            height = y2 - y1

            detection = Detection2D()
            detection.header = header

            # bbox 중심과 크기 저장함.
            # 여기서 x/y는 3D 좌표가 아니라 이미지 pixel 좌표임.
            # 후단 3D projector는 이 pixel 좌표와 depth image를 이용해 3D 위치로 역투영함.
            detection.bbox.center.position.x = float(cx)
            detection.bbox.center.position.y = float(cy)
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)

            # class name과 confidence를 hypothesis로 저장함.
            # ObjectHypothesisWithPose 메시지는 이름상 pose도 포함하지만,
            # 이 단계에서는 2D detection만 있으므로 pose는 기본값으로 둠.
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_name)
            hypothesis.hypothesis.score = confidence

            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)

        return detections_msg


def main(args=None):
    # rclpy 초기화함.
    # 모든 rclpy 노드는 init() 이후 생성되어야 함.
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        # rclpy.spin()은 callback event loop임.
        # subscriber callback, timer callback 등이 여기서 계속 처리됨.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 종료를 정상 종료로 처리함.
        pass
    finally:
        # 노드 자원 정리 후 rclpy shutdown함.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
