import cv2

# 打开相机
cap = cv2.VideoCapture(0)

# 检查相机是否成功打开
if not cap.isOpened():
    print("Failed to open camera")
else:
    print("Camera opened successfully")

# 手动设置白平衡和焦距（根据相机支持调整）
# cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4000)  # 设置蓝色平衡（假设这个控制有效）
cap.set(cv2.CAP_PROP_FOCUS, 0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 100)
# 获取当前焦距值（如果相机支持）
focus = cap.get(cv2.CAP_PROP_FOCUS)
print(f"Current focus (manual or auto): {focus}")

# 显示视频流
while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("Camera", frame)
    focus = cap.get(cv2.CAP_PROP_FOCUS)
    print(f"Current focus (manual or auto): {focus}")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
