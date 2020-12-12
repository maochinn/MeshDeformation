from face_detection import LM_detector
import cv2
import time
import struct
import socket
import numpy as np

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 3000        # The port used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

def send(landmarks):
    print(landmarks.shape)
    lms = landmarks.flatten()
    data = struct.pack("!" + "f" * len(lms), *lms)
    s.sendall(data)

test_lms = np.array([[1,2] * 68]).astype(np.float32)
send(test_lms)

def start_tracking():
    print("LOADING MODEL")
    # 2d / 3d,  cuda / cpu
    lm_detector = LM_detector(lm_type=int(2), device='cuda', face_detector='sfd')
    print("MODEL LOADED")

    print("OPENING CAMERA")
    cap = cv2.VideoCapture(0)
    print("CAMERA OPEND")

    if(cap.isOpened()):

        start_time = time.time()

        ret, frame = cap.read()

        faces = lm_detector.detect(frame)

        detect_time = time.time()

        if faces == None or len(faces) == 0:
            print("MISSING")
        else:
            send(faces[0])
            # landmarks = faces[0]
            # for lm in landmarks:
            #     cv2.circle(frame, (lm[0], lm[1]), 4, (0,0,255), -1)
            

        print_time = time.time()

        cv2.imshow('frame', frame)

        print(detect_time - start_time, print_time - detect_time, end='       \r')

        if cv2.waitKey(0) & 0xFF == ord('q'):
            print("QUIT")
            #break

    print("CLOSED")

    cap.release()
    cv2.destroyAllWindows()