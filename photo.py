import cv2 as cv
import time

cap = cv.VideoCapture(0)

t = time.time()
k = 0
name = 1
while 1:
	_, img = cap.read()
	
	cv.imshow('img', img)

	key = cv.waitKey(1)
	
	if key == ord('p'):
            lt = time.localtime()
            print('sample' + str(name) + '.png')
            cv.imwrite('sample' + str(name) + '.png', img)
            name += 1
            time.sleep(0.6)

	if key == ord('q'):
		break
	
	k += 1
	# print(time.time() - t, k, k / (time.time() - t))
		
	# if time.time() - t > 10:
      #  break

print(k / 10)
cap.release()

cv.destroyAllWindows()
	
