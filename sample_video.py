import cv2

cap=cv2.VideoCapture(0)
cap.release()
cap = cv2.VideoCapture(0)

while True:
    ret,frame=cap.read()
    hand=cv2.resize(frame,(300,300))
    cv2.imshow("frame",hand)
    if cv2.waitKey(1)==13:
        break
cap.release()
cv2.destroyAllWindows()
