#!/usr/bin/env python
from __future__ import print_function

import sys
import time
import signal
import argparse
from functools import partial
from multiprocessing import Process, Queue
try:
    import queue as queue_lib
except ImportError:
    import Queue as queue_lib

import cv2
import numpy as np
import imutils

import power


CASCADE_FILE = "haarcascade.xml"
MIN_AREA = 500
# raspi
#MAX_ELAPSED_SECS = 2.5
# faster things
MAX_ELAPSED_SECS = 0.5
ACTIVE_PIN = 11
POWER_PIN = 16


class TimeInterval:
    max_elapsed = MAX_ELAPSED_SECS
    def __init__(self):
        self.start_rect = None
        self.last_rect = None
        self.start_time = None
        self.last_time = None
        self.is_valid = False

    def update(self, rect):
        """
        check if the time elapsed from the most recent event is over a limit.
        If it's over, reset all the values. Otherwise update the latest values.
        """
        if self.start_rect is None:
            self.start_rect = rect
            self.last_rect = rect
            self.start_time = time.time()
            self.last_time = time.time()
            self.is_valid = True
        else:
            now = time.time()
            elapsed = now - self.last_time
            if elapsed > self.max_elapsed:
                self.start_rect = None
                self.last_rect = None
                self.start_time = None
                self.last_time = None
                self.is_valid = False
            else:
                self.last_rect = rect
                self.last_time = now
                self.is_valid = True


def error(msg, exit=1):
    print(msg, file=sys.stderr)
    sys.exit(exit)


def power_control(q):
    """
    Power control routine run in a separate Process
    """
    power.init_board()
    power.init_out_pins(POWER_PIN)
    interval = TimeInterval()
    on = False
    while True:
        try:
            msg = q.get_nowait()
            if msg == 'die':
                print("Killing power-control process...")
                return
            if type(msg) == tuple:
                was_valid = interval.is_valid
                interval.update(msg)
                if was_valid != interval.is_valid:
                    if interval.is_valid:
                        power.on(POWER_PIN)
                        on = True
                    elif on:
                        power.off(POWER_PIN)
                        on = False
        except queue_lib.Empty:
            now = time.time()
            if interval.last_time is not None:
                elapsed = now - interval.last_time
                if elapsed > interval.max_elapsed and on:
                    power.off(POWER_PIN)
                    on = False


def sig_handle(signum, frame, q=None, cap=None, proc=None):
    print("Received SIGTERM, signum: {}".format(signum))
    q.put('die')
    cap.release()
    cv2.destroyAllWindows()
    proc.join()
    power.off(POWER_PIN)
    power.off(ACTIVE_PIN)
    power.cleanup()
    print("Bye!")
    sys.exit(0)


def main(args):
    display = args.display

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        error("Unable to open camera")
    print("camera is open")

    detector = cv2.CascadeClassifier(CASCADE_FILE)
    if detector.empty():
        error("cascade file not found: {}".format(CASCADE_FILE))
    print("cascade file loaded")

    power.init_board()
    power.init_out_pins(ACTIVE_PIN)
    power.on(ACTIVE_PIN)

    q = Queue()
    proc = Process(target=power_control, args=(q,))
    proc.start()

    _sig_handle = partial(sig_handle, q=q, cap=cap, proc=proc)
    signal.signal(signal.SIGTERM, _sig_handle)

    try:
        print("Initializing boo-loop")
        first_frame = None
        while True:
            ret, frame = cap.read()
            if not ret:
                error("Error reading frame")

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if args.run_mode == 'face':
                try:
                    flag = cv2.cv.CV_HAAR_SCALE_IMAGE
                except AttributeError:
                    flag = cv2.CASCADE_SCALE_IMAGE
                rects = detector.detectMultiScale(
                        frame, scaleFactor=1.1, minNeighbors=5,
                        minSize=(30, 30), flags=flag)
                rects = ((int(x), int(y), int(x + w), int(y + h)) for (x, y, w, h) in rects)
                #print("n-rects: ", len(list(rects)))

                for (start_x, start_y, end_x, end_y) in rects:
                    q.put((start_x, start_y, end_x, end_y))
                    if display:
                        cv2.rectangle(frame, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
            elif args.run_mode == 'motion':
                #frame = imutils.resize(frame, width=500)
                frame = cv2.GaussianBlur(frame, (21, 21), 0)
                if first_frame is None:
                    first_frame = frame
                    continue
                delta = cv2.absdiff(first_frame, frame)
                thresh = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
                thresh = cv2.dilate(thresh, None, iterations=2)

                (contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in contours:
                    if cv2.contourArea(c) < MIN_AREA:
                        continue

                    (x, y, w, h) = cv2.boundingRect(c)
                    q.put((x, y, w, h))
                    if display:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if display:
                cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass

    print("Shutting down...")
    q.put('die')
    cap.release()
    cv2.destroyAllWindows()
    proc.join()
    power.off(POWER_PIN)
    power.off(ACTIVE_PIN)
    power.cleanup()
    print("Bye!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("run_mode", type=str, choices=['face', 'motion'])
    parser.add_argument("--display", dest="display", action='store_true', default=False)
    args = parser.parse_args()
    main(args)

