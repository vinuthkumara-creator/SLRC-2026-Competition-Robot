import cv2
import numpy as np
from pupil_apriltags import Detector

detector = Detector(
    families="tagStandard52h13",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera not found")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(gray)

    for tag in detections:
        corners = tag.corners.astype(int)

        for i in range(4):
            cv2.line(frame,
                     tuple(corners[i]),
                     tuple(corners[(i+1)%4]),
                     (0,255,0), 2)

        center = tuple(tag.center.astype(int))
        cv2.circle(frame, center, 5, (0,0,255), -1)

        cv2.putText(frame,
                    f"ID: {tag.tag_id}",
                    (center[0], center[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255,0,0), 2)

        print("Detected Tag ID:", tag.tag_id)

    cv2.imshow("AprilTag Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
