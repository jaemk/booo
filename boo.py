#!/usr/bin/env python

import sys
import cv2
import numpy as np


CASCADE_FILE = "vendor/opencv-3.1.0/data/haarcascades/haarcascade_frontalface_default.xml"


def error(msg, exit=1):
    print(msg, file=sys.stderr)
    sys.exit(exit)


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        error("Unable to open camera")

    width = int(cap.get(3))
    height = int(cap.get(4))

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.CascadeClassifier(CASCADE_FILE)
        rects = detector.detectMultiScale(
                frame, scaleFactor=1.1, minNeighbors=5,
                minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
        rects = ((int(x), int(y), int(x + w), int(y + h)) for (x, y, w, h) in rects)
        for (start_x, start_y, end_x, end_y) in rects:
            cv2.rectangle(frame, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

