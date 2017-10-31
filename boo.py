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


# -- general knobs --
#
# max time between two motion events before the interval is toggled back to invalid
MAX_ELAPSED_SECS_BETWEEN_MOTION = 0.5

# minimum time to delay after motion is first detected before toggling interval to valid
MIN_DELAY_SECS_AFTER_MOTION = 1.5

# Queue timeout, may need to be adjusted for rpi to keep cpu usage down.
# using `q.get_nowait()` will pin a core at 100%
QUEUE_TIMEOUT = 0.01

# Minimum area threshold of detected motion
MIN_AREA = 250

# Pins (gpio.BOARD layout)
ACTIVE_PIN = 11
POWER_PIN = 16
SPOOKY_PIN = 18


# -- video things -- (enabled by `--save` arg)
#
# relative directory where video files will be saved
VIDEO_DIR = 'videos'
# How many seconds to wait after motion detected before saving buffer and piping frames to video
VIDEO_MOTION_DELAY_SECS = 2
# Minimum duration of video. Videos are not saved unless they exceed this duration. TODO: Delete temp video file that's created
MIN_VID_SECS = 5
# Max number of frames to keep in a sliding buffer (frames captured prior to starting to save a video)
MAX_FRAME_BUF = 100


class TimeInterval:
    max_elapsed = MAX_ELAPSED_SECS_BETWEEN_MOTION
    min_delay = MIN_DELAY_SECS_AFTER_MOTION
    def __init__(self):
        self.reset()

    def reset(self):
        self.start_time = None
        self.last_time = None
        self.is_valid = False

    def update_last_only(self):
        self.last_time = time.time()

    def update(self):
        """
        check if the time elapsed from the most recent event is over a limit.
        If it's over, reset all the values. Otherwise update the latest values
        and set is_valid to true if the total `on` time has exceeded the min-delay.
        """
        if self.start_time is None:
            self.start_time = time.time()
            self.last_time = time.time()
            self.is_valid = False
        else:
            now = time.time()
            elapsed = now - self.last_time
            if elapsed > self.max_elapsed:
                self.reset()
                return
            self.last_time = now
            if now - self.start_time > self.min_delay:
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
            msg = q.get(timeout=QUEUE_TIMEOUT)
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


def spooky_control(q):
    power.init_board()
    power.init_out_pins(SPOOKY_PIN)
    interval = TimeInterval()
    did_pulse = False
    print("Starting spooky-control process")
    while True:
        try:
            msg = q.get(timeout=QUEUE_TIMEOUT)
            if msg == 'die':
                print("Killing spooky-control process...")
                return
            interval.update()
            if interval.is_valid and not did_pulse:
                power.pulse(SPOOKY_PIN, times=2, delay_secs=2)  # 2sec delay required to get reliable actuation
                interval.update_last_only()
                did_pulse = True
            elif not interval.is_valid:
                did_pulse = False
        except queue_lib.Empty:
            now = time.time()
            if interval.last_time is not None:
                elapsed = now - interval.last_time
                if elapsed > interval.max_elapsed and did_pulse:
                    did_pulse = False



class VideoSaveController:
    def __init__(self):
        self.count = 0

    def get_new_video_filename(self):
        while True:
            name = os.path.join(VIDEO_DIR, "video_{}.avi".format(self.count))
            if os.path.isfile(name):
                self.count += 1
            else:
                return name

    def start_from_buf(self, frame_buf):
        return VideoSaver(frame_buf, self.get_new_video_filename())


class VideoSaver:
    def __init__(self, frame_buf, video_filename):
        self.video_filename = video_filename
        self.video = cv2.VideoWriter(video_filename, cv2.cv.CV_FOURCC(*"XVID"), 40, (640, 480))
        for frame in frame_buf:
            self.video.write(frame)

    def push_frame(self, frame):
        self.video.write(frame)

    def save(self):
        print("Saving video: {}".format(self.video_filename))
        self.video.release()


def video_control(q, save=False):
    """
    Video control routine run in a separate Process.
    Manages a frame buffer and saves video files when appropriate.
    """
    if not save:
        while True:
            msg = q.get()
            if type(msg) == str and msg == 'die':
                print("Killing video-control process...")
                return

    interval = TimeInterval()
    video_save_controller = VideoSaveController()
    video_saver = None
    frame_buf = deque(maxlen=MAX_FRAME_BUF)
    print("Starting video-control process")
    while True:
        try:
            msg = q.get(timeout=QUEUE_TIMEOUT)
            if type(msg) == str and msg == 'die':
                print("Killing video-control process...")
                return
            frame_buf.append(msg)
            was_valid = interval.is_valid
            motion_time = (time.time() - interval.start_time) if interval.start_time is not None else 0
            interval.update()
            if was_valid != interval.is_valid:  # status changed
                if interval.is_valid:
                    if video_saver is None:
                        video_saver = video_save_controller.start_from_buf(frame_buf)
                else:
                    frame_buf.clear()
                    if video_saver is not None:
                        if motion_time > MIN_VID_SECS:
                            video_saver.save()
                        video_saver = None
            else:
                if video_saver is not None:
                    video_saver.push_frame(msg)
        except queue_lib.Empty:
            now = time.time()
            if interval.last_time is not None:
                elapsed = now - interval.last_time
                if elapsed > (interval.max_elapsed + VIDEO_MOTION_DELAY_SECS):
                    frame_buf.clear()
                    motion_time = interval.last_time - interval.start_time
                    if video_saver is not None:
                        if motion_time > MIN_VID_SECS:
                            video_saver.save()
                        video_saver = None
                    interval.reset()


def sig_handle(signum, frame, cap=None, pins=None, q_procs=None):
    pins = pins if pins is not None else []
    q_procs = q_procs if q_procs is not None else []
    print("Received SIGTERM, signum: {}".format(signum))
    for (q, _) in q_procs:
        q.put('die')
    cap.release()
    cv2.destroyAllWindows()
    for (_, proc) in q_procs:
        proc.join()

    for pin in pins:
        power.off(pin)
    power.cleanup()
    print("Bye!")
    sys.exit(0)


def cleanup_pins():
    power.init_board()
    power.init_out_pins(ACTIVE_PIN, POWER_PIN, SPOOKY_PIN)
    power.off(ACTIVE_PIN)
    power.off(POWER_PIN)
    power.off(SPOOKY_PIN)
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

    power_q = Queue()
    power_proc = Process(target=power_control, args=(power_q,))
    power_proc.start()

    vid_q = Queue()
    vid_proc = Process(target=video_control, args=(vid_q, save))
    vid_proc.start()

    spook_q = Queue()
    spook_proc = Process(target=spooky_control, args=(spook_q,))
    spook_proc.start()

    _sig_handle = partial(sig_handle, cap=cap,
            pins=(ACTIVE_PIN, POWER_PIN, SPOOKY_PIN),
            q_procs=((power_q, power_proc), (vid_q, vid_proc)))
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
                power_q.put(True)
                spook_q.put(True)
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
    power_q.put('die')
    vid_q.put('die')
    spook_q.put('die')
    cap.release()
    cv2.destroyAllWindows()
    print("waiting for power-controller to exit...")
    power_proc.join()
    print("waiting for video-controller to exit...")
    vid_proc.join()
    print("waiting for spooky-controller to exit...")
    spook_proc.join()
    power.off(POWER_PIN)
    power.off(ACTIVE_PIN)
    power.off(SPOOKY_PIN)
    power.cleanup()

    print("Bye!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("run_mode", type=str, choices=['motion', 'clean'])
    parser.add_argument("--display", dest="display", action='store_true', default=False)
    parser.add_argument("--save", dest="save", action='store_true', default=False)
    args = parser.parse_args()
    main(args)

