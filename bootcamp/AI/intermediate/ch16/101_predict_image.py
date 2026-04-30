from ultralytics import YOLO
import cv2
import os

# --- 설정 (Configuration) ---
# 1. 모델 로드 (가중치 파일 경로를 수정)
# 'yolov10n.pt' 파일이 현재 경로의 './weights/' 폴더에 있다고 가정합니다.
model_name = './weights/yolov10n.pt'
try:
    model = YOLO(model_name)
    print(f"모델 로드 완료: {model_name}")
except FileNotFoundError:
    print(f" 오류: 모델 파일 '{model_name}'을 찾을 수 없습니다. 경로를 확인하거나 수동으로 다운로드하세요.")
    exit()

# 사용할 이미지 파일 경로
image_path = './images/class.jpg'
if not os.path.exists(image_path):
    print(f" 오류: 이미지 파일 '{image_path}'을 찾을 수 없습니다. 경로를 확인하세요.")
    exit()

# --- 2. 이미지 불러오기 및 추론 실행 ---
print(f"이미지 경로: {image_path}")
img = cv2.imread(image_path)
if img is None:
    print(" 오류: OpenCV로 이미지를 로드할 수 없습니다.")
    exit()

print("추론 시작...")
# predict 메소드를 실행하되, save=True는 더 이상 사용하지 않습니다.
results = model.predict(
    source=img,  # 이미지를 직접 전달
    conf=0.25,
    verbose=False  # 출력 간소화
)
print("추론 완료.")

# --- 3. 결과 시각화 (OpenCV 사용) ---
if results:
    result = results[0] # 첫 번째 이미지 결과
    num_detections = len(result.boxes)
    print(f"\n✅ 검출된 총 객체 수: {num_detections}")

    # 검출된 결과에서 바운딩 박스, 클래스 ID, 신뢰도 추출
    boxes = result.boxes.xyxy.cpu().numpy()  # 바운딩 박스 좌표 (x1, y1, x2, y2)
    classes = result.boxes.cls.cpu().numpy().astype(int)  # 클래스 ID
    confidences = result.boxes.conf.cpu().numpy()  # 신뢰도

    # 시각화 설정
    thickness = 2
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7

    # 원본 이미지에 바운딩 박스 및 레이블 그리기
    for box, cls_id, conf in zip(boxes, classes, confidences):
        x1, y1, x2, y2 = map(int, box)
        class_name = model.names[cls_id]

        # 색상 설정 (클래스 ID 기반으로 고유한 색상 생성)
        color = (255, 0,0) #tuple(int(c) for c in model.model.colors[cls_id])
        
        # 바운딩 박스 그리기
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

        # 레이블 텍스트 생성: "클래스 이름 (신뢰도%)"
        label = f"{class_name} ({conf:.2f})"
        
        # 텍스트 크기 계산
        (w, h), _ = cv2.getTextSize(label, font, font_scale, thickness)
        
        # 텍스트 배경을 위한 사각형 그리기
        # 텍스트를 바운딩 박스 위에 표시
        cv2.rectangle(img, (x1, y1 - h - 5), (x1 + w, y1), color, -1)
        
        # 텍스트 그리기
        cv2.putText(img, label, (x1, y1 - 5), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        
        print(f"  - 클래스: {class_name}, 신뢰도: {conf:.2f}, 좌표: ({x1}, {y1})~({x2}, {y2})")

    # 4. 결과 이미지 화면에 표시
    cv2.imshow('YOLOv10 Custom Detection', img)
    cv2.waitKey(0)  # 사용자가 key 누를때까지 프로그램 일시 정지 
    cv2.destroyAllWindows()
    
    # (선택 사항) 결과를 파일로 저장
    output_dir = 'yolov10_output_cv'
    os.makedirs(output_dir, exist_ok=True)
    # exist_ok=True : 디렉토리가 이미 있어도 오류 발생하지 않음
    output_path = os.path.join(output_dir, os.path.basename(image_path).replace('.', '_cv_output.'))
    cv2.imwrite(output_path, img)
    print(f"\n결과 이미지는 다음 경로에 저장되었습니다: \n  -> {output_path}")

else:
    print("검출된 객체가 없습니다.")