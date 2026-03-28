# Useful Commands

## Setup

```bash
python -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt
```

## Run the robot

```bash
source .venv/bin/activate
python -m src.main
```

## Find USB camera index

```bash
python3 -c "
import cv2
for i in range(10):
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    if cap.isOpened():
        ret, frame = cap.read()
        print(f'Index {i}: opened=True, ret={ret}')
        cap.release()
    else:
        print(f'Index {i}: failed to open')
"
```

## Test camera feed

```bash
python3 -c "
import cv2
cap = cv2.VideoCapture(5)
while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
"
```

## Check camera hardware

```bash
rpicam-hello --list-cameras
dmesg | grep -i imx
v4l2-ctl --list-devices
lsusb
```
