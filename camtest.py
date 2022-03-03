import cv2 as cv
import time

cap = cv.VideoCapture(0)

t = time.time()
k = 0
while 1:
	_, img = cap.read()
	
	cv.imshow('img', img)
	
	if cv.waitKey(1) == ord('q'):
		break
	
	k += 1
	# print(time.time() - t, k, k / (time.time() - t))
		
	# if time.time() - t > 10:
      #  break

print(k / 10)
cap.release()

cv.destroyAllWindows()
	
