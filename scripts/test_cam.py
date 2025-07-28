import cv2

pipeline = (
    "v4l2src device=/dev/video2 ! "
    "image/jpeg, width=720, height=480, framerate=25/1 ! "
    "jpegdec ! videoconvert ! appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Erreur : échec d'ouverture du flux.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erreur de lecture")
        break
    cv2.imshow("Flux GStreamer", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
