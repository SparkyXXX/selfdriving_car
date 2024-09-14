import cv2
import numpy as np

cap = cv2.VideoCapture(0)


def undistort(frame):
    fx = 1311.94228326091
    cx = 937.984968117315
    fy = 1310.63631268594
    cy = 514.783585422419
    k1, k2, p1, p2, k3 = -0.469785052535390, 0.274212670963307, 0.0, 0.0, 0.0

    # 相机坐标系到像素坐标系的转换矩阵
    k = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    # 畸变系数
    d = np.array([
        k1, k2, p1, p2, k3
    ])
    h, w = frame.shape[:2]
    mapx, mapy = cv2.initUndistortRectifyMap(k, d, None, k, (w, h), 5)
    return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)


fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('output.mp4',fourcc, 20.0, (640,480))

while (cap.isOpened()):
    ret, img = cap.read()
    if ret == True:
        out.write(undistort(img))
        cv2.imshow('Image', undistort(img))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
out.release()
cv2.destroyAllWindows()