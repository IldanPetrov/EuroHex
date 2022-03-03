import numpy as np
import cv2
import cv2.aruco as aruco
import math
from math import sin, cos
import time

frame = np.array([])
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 4x4 dictionary to find markers
parameters = aruco.DetectorParameters_create()  # Marker detection parameters


def acc(arr, k = 1, ac = 1):
    # print(arr, arr.shape, len(arr.shape))
    if len(arr.shape) == 1:
        r = np.array([round(el * k, ac) for el in arr])
        # print(arr, r)
        return r
    else:
        r = np.array([acc(el, k, ac) for el in arr])
        # print(r)
        return r


def calCamAngle(v):
    zA = math.asin(v[0] / ((v[0] ** 2 + v[1] ** 2) ** 0.5))
    xA = math.asin(v[2] / ((v[1] ** 2 + v[2] ** 2) ** 0.5))
    return np.array([xA, 0, zA * -1])


def rotate(rvec, matrix):
    rotM = np.zeros(shape=(3, 3))
    cv2.Rodrigues(rvec, rotM, jacobian = 0)
    newMatrix = np.dot(rotM, matrix.T).T
    return newMatrix
    

def vecAngle(v):
    vecA = math.acos(v[2] / (np.dot(v, v) ** 0.5)) * 180 / math.pi
    return vecA * (int(v[0] >= 0) * 2 - 1)


def track(matrix_coefficients, distortion_coefficients):
	global frame
	# cap = cv2.VideoCapture(0)  # Get the camera sourceeeeee
	# waiting = time.time()
	# while time.time() - waiting < 0.15:
		# ret, frame = cap.read()
		# cv2.waitKey(3)
	
	# operations on the frame come here
	# cap.release()
	ret, frame = cap.read()
	cv2.waitKey(3)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
	# aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5 dictionary to find markers
	# parameters = aruco.DetectorParameters_create()  # Marker detection parameters
	# lists of ids and the corners beloning to each id
	corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)
	
	hexes = np.array([])
	
	if np.all(ids is not None):  # If there are markers found by detector
		hexes = np.array([[None] * 4 for i in range(len(ids))])
		for i in range(0, len(ids)):  # Iterate in markers
			print()
			# Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
			rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.05, matrix_coefficients, distortion_coefficients)
			(rvec - tvec).any()  # get rid of that nasty numpy value array error 
			# markerPoints = np.resize(markerPoints, (4, 3))
			# markerPoints = rotate(rvec[0][0][1], rvec[0][0][2], rvec[0][0][0], markerPoints)
			# markerPoints = markerPoints + tvec[0][0]
			
			arucoVec = np.array([[0.025, 0, 0], [0, 0.025, 0], [0, 0, 0.025]])
			arucoVec = rotate(rvec, arucoVec)
			camAngle = calCamAngle(arucoVec[2])
			# print(acc(arucoVec[2], 1, 4), acc(camAngle, 1, 4))
			arucoVec = rotate(camAngle, arucoVec)
			# print(acc(arucoVec, 100, 2))
			
			tvec = rotate(camAngle, tvec[0])[0]
			rvec = rvec + camAngle
			
			driveTo = rotate(rvec, positions) + tvec
			angles2 = np.array([vecAngle(tvec - v) for v in driveTo])
			
			fromRob = driveTo - robPos
			fromRob = np.array([[v[0], 0, v[2]] for v in fromRob])
			distances = np.array([np.dot(v, v) ** 0.5 for v in fromRob])
			
			angles1 = np.array([vecAngle(v) for v in fromRob])
			
			# print(ids[i])
			# print(acc(distances, 100, 1))
			# print(acc(angles1))
			# print(acc(angles2))
			
			thisHex = np.array([[ids[i][0], distances[ind], angles1[ind], angles2[ind]] for ind in range(6)])
			hexes[i] = min(thisHex, key = lambda el: el[1])
			aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
			# aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
			
			# Draw Axis
			# cv2.putText(frame, 'Transition vector: ',
			# (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
			# cv2.putText(frame, ' '.join(map(lambda i: str(round(i * 100, 1)), tvec[0][0])),
			# (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			# cv2.putText(frame, 'Rotation vector: ',
			# (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
			# cv2.putText(frame, ' '.join(map(lambda i: str(round(i * 180 / math.pi, 1)), rvec[0][0])),
			# (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
	
	# Drawing camera axis
	# cv2.line(frame, (10, 10), (10, 40), (0, 255, 0), 3)
	# cv2.line(frame, (10, 10), (40, 10), (0, 0, 255), 3)
	# cv2.line(frame, (8, 8), (10, 10), (255, 0, 0), 5)
	hexes = hexes[hexes[:, 1].argsort()]
	print(hexes)


def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


positions = np.array([rotate(np.array([0, 0, 60 / 180 * math.pi]) * i, np.array([0.26, 0, 0])) for i in range(6)])
robPos = np.array([0, 0.095, -0.165])

cap = cv2.VideoCapture(0)
waiting = time.time()
while time.time() - waiting < 0.15:
    ret, frame = cap.read()
    cv2.waitKey(3)
    
camera_matrix, dist_matrix = load_coefficients('/home/pi/save_file.YML')
track(camera_matrix, dist_matrix)
cap_reset = time.time()
while 1:
    cv2.imshow('frame', frame)
    time.sleep(0.2)
    key = cv2.waitKey(5)
    if key == ord('e'):
        break
    
    track(camera_matrix, dist_matrix)


cv2.destroyAllWindows()
cap.release()

