import numpy as np
import cv2
import cv2.aruco as aruco
import math
from math import sin, cos
import time

frame = np.array([])
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5 dictionary to find markers
parameters = aruco.DetectorParameters_create()  # Marker detection parameters

def rotate(xA, yA, zA, matrix):
    xA *= 1
    yA *= 1
    zA *= -1
    xAxis = np.array([[1, 0, 0],
                      [0, cos(xA), -sin(xA)],
                      [0, sin(xA), cos(xA)]])
    yAxis = np.array([[cos(yA), 0, sin(yA)],
                      [0, 1, 0],
                      [-sin(yA), 0, cos(yA)]])
    zAxis = np.array([[cos(zA), -sin(zA), 0],
                      [sin(zA), cos(zA), 0],
                      [0, 0, 1]])
    matrix = matrix.T
    rotMatrix = np.dot(np.dot(xAxis.T, yAxis.T).T, zAxis.T)
    newMatrix = np.dot(rotMatrix.T, matrix).T
    return newMatrix


def rotate_new(rvec, matrix):
    rotM = np.zeros(shape=(3, 3))
    cv2.Rodrigues(rvec, rotM, jacobian = 0)
    newMatrix = np.dot(rotM, matrix.T).T
    return newMatrix
    

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
    
    if np.all(ids is not None):  # If there are markers found by detector
        for i in range(0, len(ids)):  # Iterate in markers
            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.05, matrix_coefficients, distortion_coefficients)
            (rvec - tvec).any()  # get rid of that nasty numpy value array error 
            # markerPoints = np.resize(markerPoints, (4, 3))
            # markerPoints = rotate(rvec[0][0][1], rvec[0][0][2],
            #                       rvec[0][0][0], markerPoints)
            markerPoints = np.array([[0.025, 0, 0], [0, 0.025, 0], [0, 0, 0.025]])
            markerPoints = rotate_new(rvec, markerPoints)
            print(markerPoints)
            # markerPoints = markerPoints + tvec[0][0]
            
            for p in range(0):
                text = ' '.join(map(lambda val: str(round(val * 100, 1)), markerPoints[p]))
                cv2.putText(frame, text, tuple(map(int, corners[0][i][p])),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 20, 255), 2)
            
            aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
            # aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
            
            # Draw Axis
            cv2.putText(frame, 'Transition vector: ',
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            cv2.putText(frame, ' '.join(map(lambda i: str(round(i * 100, 1)), tvec[0][0])),
            (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, 'Rotation vector: ',
            (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            cv2.putText(frame, ' '.join(map(lambda i: str(round(i * 180 / math.pi, 1)), rvec[0][0])),
            (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    
    # Drawing camera axis
    cv2.line(frame, (10, 10), (10, 40), (0, 255, 0), 3)
    cv2.line(frame, (10, 10), (40, 10), (0, 0, 255), 3)
    cv2.line(frame, (8, 8), (10, 10), (255, 0, 0), 5)


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

