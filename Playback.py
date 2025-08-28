import asyncio
import time
import cv2
import numpy as np
import math
from sklearn.cluster import DBSCAN
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Detection import TrackerUI, Track, Tracks
from trckerV2 import Tracker
from datetime import datetime
import zmq.asyncio
import msgpack

import socket
import struct
import pickle
import platform
import json

HOST_IP = "127.0.1.1"
HOST_PORT = 5000

tracker = Tracker(merge_threshold=3)

import scipy.io


async def video_trackerV2(publisher1, filepath=""):
    camera_type = "usb"
    if filepath != "":
        cap = cv2.VideoCapture(filepath)
        print("Taking path")
    elif camera_type == "usb":
        try:
            print(f"Initializing connection to server")
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((HOST_IP, HOST_PORT))
            print("Socket opened.")
            data = b''
            payload_size = struct.calcsize("Q")
        except Exception as e:
            print(f"Exception occurred: {e}")
    elif camera_type == "rtsp":
        ip_camera_url = "rtsp://admin:Admin123456@192.168.1.122/live"
        # Set desired resolution
        width = 1280  # Set your desired width
        height = 720  # Set your desired height

        # Create OpenCV VideoCapture object
        cap = cv2.VideoCapture(ip_camera_url)
        # Set resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 300)

    ### Option to get frames satright from usb6
    # elif isUSBCamera:
    #     cap = cv2.VideoCapture("/dev/video6", cv2.CAP_V4L2)

    else:
        return

    file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'matlab3.mat')
    file_path2 = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'fit_new_2.mat')
    mat = scipy.io.loadmat(file_path)
    mat2 = scipy.io.loadmat(file_path2)
    y3 = mat['y3']
    look = mat['look']
    y_h = mat2['y']
    y_r = mat2['y2']
    counter = 0
    gg = np.zeros((480, 640), dtype=np.uint8)
    rr = []
    qq = []
    factt = []
    id = []
    range2 = []
    az2 = []
    vxx = []
    vyy = []
    time8 = []
    quene = []
    trackerUI = TrackerUI()
    trackers = Tracks()
    time1_to_send = 0
    time2_to_send = 0
    # track_counter = 0
    start_time_NEW = time.time()
    folder_path = '/home/mignan/recordings_video/video_to_UI'
    current_datetime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    file_name = f"{current_datetime}.txt"
    file_path = os.path.join(folder_path, file_name)
    file = open(file_path, 'wb')

    # fgbg = cv2.createBackgroundSubtractorMOG2(history=1500,varThreshold=50,detectShadows=False)
    fgbg = cv2.createBackgroundSubtractorMOG2()
    fgbg = cv2.createBackgroundSubtractorKNN(history=100, dist2Threshold=200.0, detectShadows=True)
    # fgbg.setNMixtures(3)
    # fgbg.setBackgroundRatio(0.85)
    count_of_null = 0
    count_number = 0
    while True:
        start_time_NEW_CYCLE = time.time()
        if camera_type == "usb" and filepath == '':
            while len(data) < payload_size:
                packet = client_socket.recv(4 * 1024)
                if not packet: print("No Packets comming")
                data += packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += client_socket.recv(4 * 1024)
                # print(data)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)
            frame = cv2.resize(frame, (640, 512))

        else:

            _, frame = cap.read()

        fgmask = fgbg.apply(frame)
        fgmask = cv2.dilate(fgmask, None, iterations=2)  # Clean up the mask a bit

        # fgmask[1:300, 1:640] = 0

        fgmask[0:221, 0:641] = 0

        fgmask[0:512, 0:70] = 0

        fgmask[0:512, 600:640] = 0

        # Find contours in the mask
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        x4, y4, h4, w4 = [], [], [], []
        count = 0
        MERGE_THRESHOLD = 80

        boxes = []

        for contour in contours:
            if cv2.contourArea(contour) > 65:  # Minimum contour area (>to avoid noise)
                (x, y, w, h) = cv2.boundingRect(contour)
                new_box = (x, y, x + w, y + h)
                merged = False

                # Check and merge with existing boxes if necessary
                for idx, box in enumerate(boxes):
                    bx, by, bx2, by2 = box
                    b_cent_x = (bx + bx2) / 2
                    b_cent_y = (by + by2) / 2

                    new_cent_x = x + w / 2
                    new_cent_y = y + h / 2

                    distance = np.sqrt((b_cent_x - new_cent_x) ** 2 + (b_cent_y - new_cent_y) ** 2)

                    if distance < MERGE_THRESHOLD:
                        # Fusionne les boxes
                        merged_box = (min(bx, x), min(by, y), max(bx2, x + w), max(by2, y + h))
                        if (merged_box[2] - merged_box[0]) < 150:
                            boxes[idx] = merged_box
                            merged = True
                            break

                if not merged:
                    if (new_box[2] - new_box[0]) < 150:
                        boxes.append(new_box)

        # Draw all detected and merged rectangles
        if len(boxes) > 5:
            continue
        x4, y4, h4, w4 = [], [], [], []
        for (x1, y1, w1, h1) in boxes:
            # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            x4.append(x1)
            y4.append(y1)
            h4.append(h1)
            w4.append(w4)
            cv2.rectangle(frame, (x1, y1), (w1, h1), (0, 255, 0), 2)

        for x9, y9, w9, h9 in boxes:

            lk = abs(round(h9 - y9))
            differences = np.abs(y3[0, :] - lk)
            differences2 = np.abs(y_h[0, :] - lk)
            index = np.argmin(differences)
            index2 = np.argmin(differences2)

            # print('index',index)
            # # print(h9[0])
            # print('lk',lk)
            # if counter > 1700:
            #     h_for_fitting.append(lk)
            # r_for_fitting.append(look[0, index])

            # if lk > 35 and counter>404:
            # 35
            # 20
            if lk > 1:
                # print(index)
                # print(look[0,index])
                mw1 = w9
                mx1 = x9
                my1 = y9
                # dis4.append(look[0, index])
                # print(dis4)
                fine_range = look[0, index] - 5 * (look[0, index] / 10)
                # fine_range=y_r[0,(500001-index2)]
                fine_range = y_r[0, (index2)]
                cv2.putText(frame, str((round(fine_range))), (mx1 + 20, my1 + 20), cv2.FONT_HERSHEY_PLAIN, 2,
                            (0, 0, 255),
                            6)
                # if lk<40 and counter>1500:
                #     print(h4)
                #     print(w4)
                #     print(x4)
                #     print(y4)
                #
                #     bm=9
                #
                # if counter == 469:
                #     yt = 8
                # if look[0,index] > 4:
                if 1:
                    az = w9 / 2
                    az = az / 13.333
                    az = az - 24
                    if x9 > 320:
                        az = (x9 - 320) * 0.078
                    else:
                        az = abs(x9 - 320) * -0.078

                    # print('az',az)
                    measurements = [[az, fine_range]]
                    tracker.update(measurements)
                cc = 0
                trackerUI.detections = []
                trackers.tracks = []
                trackers_len = 0
                for track in tracker.tracks:
                    cc = cc + 1
                    # print("Track ID:", track.track_id, "State:", track.state, "Age:", track.age)
                    # id.append(track.track_id)
                    # range2.append(track.state[1])
                    if track.age > 30:
                        # if track.age > 15:                                                                                                                                                                                                                                                            0  :
                        id.append(track.track_id)
                        range2.append(track.state[1])
                        az2.append(track.state[0])
                        vxx.append(track.state[2])
                        vyy.append(track.state[3])
                        # time8.append(milliseconds)
                        cv2.putText(frame, str((round(track.track_id))),
                                    (mx1 + (40 + cc * 6), my1 + 40 + cc * 6),
                                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 6)
                        cv2.rectangle(frame, (x9, y9), (w9, h9), (255, 0, 255), 2)

                        # Send to UI
                        angle_radians = math.radians(track.state[0])
                        sine_value = math.sin(angle_radians)
                        trackZMQ = Track()
                        trackZMQ.setParameters(track.track_id * 1, sine_value * track.state[1],
                                               track.state[1], track.state[2], track.state[3])
                        print(f"tk id: {track.track_id * 1} ", )
                        trackers.add_radar_data(trackZMQ)
                        trackers_len = len(trackers.tracks)
                        trackers.set_length(trackers_len)

                    # else:
                    #     id.append(0)
                    #     range2.append(0)
                    #     az2.append(0)
                    #     vxx.append(0)
                    #     vyy.append(0)
                    #     time8.append(milliseconds)

                    if trackers_len > 0:
                        count_number += 1
                        # print(trackers.to_list())
                        packed_message = msgpack.packb(trackers.to_list(), use_bin_type=True)
                        # file.write(packed_message)
                        print(count_number)
                        # print("TARGET")
                        # start_time = time.perf_counter()
                        end_time = time.perf_counter()
                        # execution_time_ms = (end_time - start_time) * 1000  # Convert to milliseconds
                        # if execution_time_ms>100:
                        # print(datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " New send to UI")
                        await async_send(publisher1, packed_message)
                        # end_time = time.perf_counter()
                        # execution_time_ms = (end_time - start_time) * 1000  # Convert to milliseconds
                        # print(f"Execution time of send(): {execution_time_ms:.2f} ms")
                        # t =datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        # print(f" data to Q", packed_message)
                        # print(datetime.now().strftime("%H:%M:%S.%f")[:-3], "trV2")
                        # counter_ui=counter_ui+1
                    else:
                        file.write(b"")
                        # print("NULL")
        else:
            rr.append(1)
            qq.append(1)
        # cv2.imshow('y',tr)
        cv2.imshow('yt', frame)

        # if counter == 6530:
        #     df = pn.DataFrame({'id': id, 'range': range2,'az2': az2, 'vx': vxx,'vy': vyy})
        #     csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\id125.csv'
        #     df.to_csv(csv_file, index=False)
        #     break
        cv2.waitKey(1)
    # print(f"Cycle run: {time.time() - start_time_NEW_CYCLE}")
    # await asyncio.sleep(0.005)


async def async_send(publisher, packed_message):
    await publisher.send(packed_message)


if __name__ == "__main__":
    context = zmq.asyncio.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://*:6005")  # Sends a string message

    # run video from live

    # run Bozon live
    # asyncio.run(video_trackerV2(publisher))

    # Run recorded video
    asyncio.run(video_trackerV2(publisher, filepath='/home/mignan/recordings_video/daniel_scenraio_0.mp4'))
    # asyncio.run(video_trackerV2(publisher, filepath='/home/mignan/recordings_video/daniel_scenraio_1.mp4'))
    # asyncio.run(video_trackerV2(publisher, False,filepath='/home/mignan/recordings_video/output2_video.mp4'))
    # asyncio.run(video_trackerV2(publisher, filepath='/home/mignan/recordings_video/video_seq2_20250126_145023.mp4'))
    # asyncio.run(video_trackerV2(publisher, filepath='//Home//Downloads//video_seq1_20250126_144416.mp4'))

    # asyncio.run(video_trackerV2(publisher,  False, filepath='C:\\Users\\mignan\\recordings_video\\output2_video.mp4'))