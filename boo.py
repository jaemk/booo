#!/usr/bin/env python
from __future__ import print_function

import os
import sys
import time
import signal
import argparse
from functools import partial
from multiprocessing import Process, Queue
from collections import deque
try:
    import queue as queue_lib
except ImportError:
    import Queue as queue_lib

import cv2
import numpy as np

import power


VIDEO_DIR = 'videos'
MAX_ELAPSED_SECS = 0.5
VIDEO_MOTION_DELAY = 2
MIN_VID_TIME = 5
# Motion detection area
MIN_AREA = 250
MAX_FRAME_BUF = 100
ACTIVE_PIN = 11
POWER_PIN = 16


class TimeInterval:
    max_elapsed = MAX_ELAPSED_SECS
    def __init__(self):
        self.reset()

    def reset(self):
        self.start_time = None
        self.last_time = None
        self.is_valid = False

    def update(self):
        """
        check if the time elapsed from the most recent event is over a limit.
        If it's over, reset all the values. Otherwise update the latest values.
        """
        if self.start_time is None:
            self.start_time = time.time()
            self.last_time = time.time()
            self.is_valid = True
        else:
            now = time.time()
            elapsed = now - self.last_time
            if elapsed > self.max_elapsed:
                self.start_time = None
                self.last_time = None
                self.is_valid = False
            else:
                self.last_time = now
                self.is_valid = True


def error(msg, exit=1):
    print(msg, file=sys.stderr)
    sys.exit(exit)


def power_control(q):
    """
    Power control routine run in a separate Process.
    Manages toggling raspberry pi gpio pins
    """
    power.init_board()
    power.init_out_pins(POWER_PIN)
    interval = TimeInterval()
    on = False
    print("Starting power-control process")
    while True:
        try:
            msg = q.get_nowait()
            if msg == 'die':
                print("Killing power-control process...")
                return
            was_valid = interval.is_valid
            interval.update()
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


class VideoSaver:
    def __init__(self):
        self.count = 0

    def get_new_video_filename(self):
        while True:
            name = os.path.join(VIDEO_DIR, "video_{}.avi".format(self.count))
            if os.path.isfile(name):
                self.count += 1
            else:
                return name

    def save_vid(self, frame_buf, motion_time):
        video_filename = self.get_new_video_filename()
        print("saving video: {}, length: {}s".format(video_filename, motion_time))
        video = cv2.VideoWriter(video_filename, cv2.cv.CV_FOURCC(*"XVID"), 20, (640, 480))
        for frame in frame_buf:
            video.write(frame)
        video.release()


def video_control(q, save):
    """
    Video control routine run in a separate Process.
    Manages a frame buffer and saves video files when appropriate.
    """
    interval = TimeInterval()
    video_saver = VideoSaver()
    frame_buf = deque(maxlen=MAX_FRAME_BUF)
    if save:
        print("Starting video-control process")
    while True:
        try:
            msg = q.get_nowait()
            if type(msg) == str and msg == 'die':
                print("Killing video-control process...")
                return
            frame_buf.append(msg)
            interval.update()
            if not interval.is_valid:
                frame_buf.clear()
        except queue_lib.Empty:
            now = time.time()
            if interval.last_time is not None:
                elapsed = now - interval.last_time
                motion_time = interval.last_time - interval.start_time
                if elapsed > (interval.max_elapsed + VIDEO_MOTION_DELAY):
                    if motion_time > MIN_VID_TIME:
                        video_saver.save_vid(frame_buf, motion_time)
                    frame_buf.clear()
                    interval.reset()


def sig_handle(signum, frame, cap=None, q1=None, proc1=None, q2=None, proc2=None):
    print("Received SIGTERM, signum: {}".format(signum))
    q1.put('die')
    q2.put('die')
    cap.release()
    cv2.destroyAllWindows()
    proc1.join()
    proc2.join()
    power.off(POWER_PIN)
    power.off(ACTIVE_PIN)
    power.cleanup()
    print("Bye!")
    sys.exit(0)


def cleanup_pins():
    power.init_board()
    power.init_out_pins(ACTIVE_PIN, POWER_PIN)
    power.off(ACTIVE_PIN)
    power.off(POWER_PIN)
    power.cleanup()


def main(args):
    if args.run_mode == 'clean':
        cleanup_pins()
        return

    display = args.display
    save = args.save

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        error("Unable to open camera")
    print("camera is open")

    power.init_board()
    power.init_out_pins(ACTIVE_PIN)
    power.on(ACTIVE_PIN)

    q = Queue()
    proc = Process(target=power_control, args=(q,))
    proc.start()

    vid_q = Queue()
    vid_proc = Process(target=video_control, args=(vid_q, save))
    vid_proc.start()

    _sig_handle = partial(sig_handle, cap=cap, q1=q, proc1=proc, q2=vid_q, proc2=vid_proc)
    signal.signal(signal.SIGTERM, _sig_handle)

    try:
        print("Initializing boo-loop")
        first_frame = None
        while True:
            ret, capture = cap.read()
            if not ret:
                error("Error reading frame")


            frame = cv2.cvtColor(capture, cv2.COLOR_BGR2GRAY)
            frame = cv2.GaussianBlur(frame, (21, 21), 0)
            if first_frame is None:
                first_frame = frame
                continue
            delta = cv2.absdiff(first_frame, frame)
            first_frame = frame
            thresh = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)

            contour_tup = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # findContours returns (contours, _) for opencv2 and (_, contours, _) for opencv3
            contours = contour_tup[0] if len(contour_tup) == 2 else contour_tup[1]
            for c in contours:
                c_area = cv2.contourArea(c)
                if c_area < MIN_AREA:
                    continue
                #print("found motion, area: ", c_area)
                (x, y, w, h) = cv2.boundingRect(c)
                (start_x, start_y, end_x, end_y) = (x, y, x + w, y + h)
                q.put((start_x, start_y, end_x, end_y))
                if save:
                    vid_q.put(capture)
                if display:
                    cv2.rectangle(capture, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)

            if display:
                cv2.imshow('capture', capture)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass

    print("Shutting down...")
    q.put('die')
    vid_q.put('die')
    cap.release()
    cv2.destroyAllWindows()
    proc.join()
    vid_proc.join()
    power.off(POWER_PIN)
    power.off(ACTIVE_PIN)
    power.cleanup()

    print("Bye!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("run_mode", type=str, choices=['motion', 'clean'])
    parser.add_argument("--display", dest="display", action='store_true', default=False)
    parser.add_argument("--save", dest="save", action='store_true', default=False)
    args = parser.parse_args()
    main(args)

