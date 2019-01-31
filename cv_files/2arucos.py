#!/usr/bin/env python3
import cv2
import urllib.request
import numpy as np
import cv2.aruco as aruco
import sys, time
print('Finding position')

id_to_find = 256
marker_size = 35

camera_matrix = np.loadtxt('camera_matrix.txt')
distortion_matrix = np.loadtxt('distortion_coefficients.txt')

R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1
R_flip[1, 1] = -1
R_flip[2, 2] = -1

font = cv2.FONT_HERSHEY_SIMPLEX

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

def use_images(left, right, num):
    """
    Use the 2 frames from the 2 stereo pi-cameras
q
    :param right: right camera video (still) frame
    :param left: left camera video (still) frame
    :return:
    """
    # global(id_to_find, marker_size, camera_matrix, distortion_matrix, R_flip, font, aruco_dict, parameters)
    img = left
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(image = gray, dictionary = aruco_dict, parameters = parameters, cameraMatrix=camera_matrix, distCoeff=distortion_matrix)
    str_position256 = 'None'
    str_position512 = 'None'
    str_position128 = "None"
    if type(ids) != type(None):
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_matrix)
        for i in range(len(ids)):
            if ids[i] == 256:
                rvec, tvec = ret[0][i, 0, :], ret[1][i, 0, :]
                aruco.drawDetectedMarkers(img, corners)
                aruco.drawAxis(img, camera_matrix, distortion_matrix, rvec, tvec, 10)
                str_position256 = 'MARKER {} Position x = {} y = {}, z = {}'.format(ids[i], round(tvec[0], 4), round(tvec[1], 4), round(tvec[2], 4))
                pos = [0, 0, 0]
                place = []
                # print(tvec[1])
                # print(tvec[0]<-0.0252, tvec[1]<0.11)
                if tvec[0]/1000 > -0.0252:
                    pos[0] = -1*tvec[0]/1000 - 0.0252
                    pos[1] = 1 * tvec[1] / 1000 + 0.017#0.2874
                    pos[2] = 0.085
                    place.append(1)
                    # print('a')
                elif tvec[0]/1000 < -0.0252:
                    azimuth = np.arctan(tvec[1] / tvec[0])
                    error = 0
                    pos[0] = -1*(-1*tvec[0]/1000 - 0.015 + error*np.cos(azimuth))
                    pos[1] = 1 * tvec[1] / 1000 - 0.005 - error*np.sin(azimuth)
                    pos[2] = 0.075
                    place.append(2)
                    # print('b')
                if num%20 == 0:
                    print('Raw position = {}'.format(tvec/1000))
                    print('Transformed position = {}'.format(pos))
                    np.savetxt('coord.txt', np.array(pos)), np.savetxt('tvec_original.txt', tvec/1000), np.savetxt('place.txt', np.array(place))
            if ids[i] == 512:
                rvec, tvec = ret[0][i, 0, :], ret[1][i, 0, :]
                aruco.drawDetectedMarkers(img, corners)
                aruco.drawAxis(img, camera_matrix, distortion_matrix, rvec, tvec, 10)
                str_position512 = 'MARKER {} Position x = {} y = {}, z = {}'.format(ids[i], round(tvec[0], 4), round(tvec[1], 4), round(tvec[2], 4))
                pos = [0, 0, 0]
                place = []
                # print(tvec[1])
                # print(tvec[0]<-0.0252, tvec[1]<0.11)
                if tvec[0]/1000 > -0.0252:
                    pos[0] = -1*tvec[0]/1000 - 0.0252
                    pos[1] = 1 * tvec[1] / 1000 + 0.017#0.2874
                    pos[2] = 0.09
                    place.append(1)
                    # print('a')
                elif tvec[0]/1000 < -0.0252:
                    pos[0] = -1*(-1*tvec[0]/1000 - 0.043)
                    pos[1] = 1 * tvec[1] / 1000 -0.022 #0.2874
                    pos[2] = 0.08
                    place.append(2)
                if num%20 == 0:
                    # print('Raw position = {}'.format(tvec/1000))
                    # print('Transformed position = {}'.format(pos))
                    np.savetxt('coord_place.txt', np.array(pos))
                    np.savetxt('place_place.txt', np.array(place))
                    np.savetxt('tvec_original_place.txt', tvec/1000)
            if ids[i] == 128:
                ret = aruco.estimatePoseSingleMarkers(corners, 25, camera_matrix, distortion_matrix)
                rvec, tvec = ret[0][i, 0, :], ret[1][i, 0, :]
                aruco.drawDetectedMarkers(img, corners)
                aruco.drawAxis(img, camera_matrix, distortion_matrix, rvec, tvec, 10)
                area = np.loadtxt('place.txt')
                # print(area == 2)
                if area == 2:
                    azimuth = np.arctan(tvec[1] / tvec[0])
                    error = 0.05
                    tvec[0] = tvec[0] - error*azimuth
                    tvec[1] = 1 * tvec[1] -error*azimuth
                str_position128 = 'MARKER {} Position x = {} y = {}, z = {}'.format(ids[i], round(tvec[0], 4),                                                                        round(tvec[1], 4),
                                                                                        round(tvec[2], 4))
                np.savetxt('tvec_corr.txt', tvec/1000)
        cv2.putText(img, str_position256, (0, 100), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, str_position512, (0, 200), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, str_position128, (0, 300), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('img', img)

def get_images():
    # open the mjpeg stream
    # Insert your IP here
    stream1 = urllib.request.urlopen('http://192.168.43.250:5000/video_feed')  # Left
    # stream2 = urllib.request.urlopen('http://192.168.43.250:5000/video_feed')  # Right

    img1 = b''
    # img2 = b''

    num = 0
    np.savetxt('area.txt', np.array([0]))

    while True:
        img1 += stream1.read(1024)  # stream 1 - Left
        # img2 += stream2.read(1024)  # stream 2 - Right

        a1 = img1.find(b'\xff\xd8')
        b1 = img1.find(b'\xff\xd9')

        # a2 = img2.find(b'\xff\xd8')
        # b2 = img2.find(b'\xff\xd9')

        # if a1 != -1 and b1 != -1 and a2 != -1 and b2 != -1:
        if a1 != -1 and b1 !=-1:
            jpg1 = img1[a1:b1 + 2]
            img1 = img1[b1 + 2:]

            # jpg2 = img2[a2:b2 + 2]
            # img2 = img2[b2 + 2:]

            # Decode jpeg image to raw for opencv
            frame1 = cv2.imdecode(np.fromstring(jpg1, dtype=np.uint8), cv2.IMREAD_LOAD_GDAL)  # Left
            # frame2 = cv2.imdecode(np.fromstring(jpg2, dtype=np.uint8), cv2.IMREAD_LOAD_GDAL)  # Right
            frame2 = None
            num += 1
            use_images(frame1, frame2, num)
            # time.sleep(5)

            # wait for ___ key
            k = cv2.waitKey(1)
            if k == 0xFF & ord("q"):
                exit(0)


# if __name__ == "__main__":
#     get_images(k)
get_images()
