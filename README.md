# BOOO

> motion activated spooky actions

The goal of this project is to use motion detection to activate spooky things.
The assumed target environment is a Raspberry Pi 2 running rasbian jessie, a PS Eye webcam, and a PowerSwitchTail 2.

There are several outputs (with the gpio pins configured in gpio.BOARD mode):

1. Set and keep pin 11 HIGH while program runs.
2. When motion is detected and lasts for at least `MIN_DELAY_SECS_AFTER_MOTION`, while motion exists set pin 16 HIGH.
3. When motion is detected and lasts for at least `MIN_DELAY_SECS_AFTER_MOTION`, pulse pin 18 twice and then wait until motion is
   detected again.


## Setup

**Option 1 (python2, opencv2, easier option):**

- `apt install opencv-python`
- Unfortunately, this requires using the system python2

**Option 2 (python3, opencv3, compiling opencv takes a while, pyhon linking problems on rpi):**

- Setup a virtualenv:
    - `python3 -m virtualenv env`
    - `source env/bin/activate`
- Install [`opencv`](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/)
    - Install dependencies
    - `mkdir vendor && cd vendor` -- download opencv source here


## Running

Note, `sudo` is required to access the webcam and gpio pins when running on an RPi.
If you're running locally, you can probably run without root depending on how your webcam works.

If you installed with a virtual environment:

```
$ sudo env/bin/python boo.py motion
```

----


```
$ sudo ./boo.py motion
```

----

Running with `--display` will show the current captured frames.

Running with `--save` will hold onto the last `MAX_FRAME_BUF` frames in a rolling frame-buffer.
After motion is detected for `VIDEO_MOTION_DELAY_SECS`, the frame-buffer will be dumped to a new video and then
all subsequent frames will be appended to the video. Note: this will use a lot of memory, fast.
Videos are saved in `VIDEO_DIR`.

**Auto run on startup**

- `sudo cp boo.service.sample /lib/systemd/system/boo.service`
- Update the path to your project dir in `/lib/systemd/system/boo.service`
- `sudo systemctl daemon-reload`
- `sudo systemctl enable boo.service`
- `sudo systemctl start boo`
- `sudo systemctl status boo`

