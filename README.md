# BOOO

> motion activated spooky actions

The goal of this project is to get a motion toggled power source to activate spooky things.
The assumed target environment is a Raspberry Pi 2 running rasbian jessie, a PS Eye webcam, and a PowerSwitchTail 2.


## Setup

**Option 1 (python2, opencv2, easier):**

- `apt install opencv-python`
- Unfortunately, this requires using the system python2

**Option 2 (python3, opencv3, long, problems on rpi):**

- Setup a virtualenv:
    - `python3 -m virtualenv env`
    - `source env/bin/activate`
- Install [`opencv`](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/)
    - Install dependencies
    - `mkdir vendor && cd vendor` -- download opencv source here
    - `cp vendor/opencv-3.1.0/data/haarcascades/haarcascade_frontalface_default.xml haarcascade.xml`


## Running

If you installed with a virtual environment:

```
(env) $ ./boo.py motion
```

----

If you installed the simple way with apt opencv-python (sudo is required to access the webcam and gpio pins):

```
$ sudo ./boo.py motion
```

----

if you run with `--save`, a running frame-buffer will saved after motion is no longer detected.

**Auto run on startup**

- `sudo cp boo.service.sample /lib/systemd/system/boo.service`
- Update the path to your project dir in `/lib/systemd/system/boo.service`
- `sudo systemctl daemon-reload`
- `sudo systemctl enable boo.service`
- `sudo systemctl start boo`
- `sudo systemctl status boo`

