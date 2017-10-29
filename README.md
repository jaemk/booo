# BOO

## Setup

**Option 1 (python3, long):**

- Setup a virtualenv:
    - `python3 -m virtualenv env`
    - `source env/bin/activate`
- Install [`opencv`](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/)
    - Install dependencies
    - `mkdir vendor && cd vendor` -- download opencv source here
    - `cp vendor/opencv-3.1.0/data/haarcascades/haarcascade_frontalface_default.xml haarcascade.xml`

**Option 2 (python2, easier):**

- `apt install opencv-python`


## Running

If you installed with a virtual environment:

```
(env) $ ./boo.py motion
```

----

If you installed the simple way with apt opencv-python:

```
$ sudo ./boo.py motion
```

----

**Auto run on startup**

- `sudo cp boo.service.sample /lib/systemd/system/boo.service`
- Update the path to your project dir in `/lib/systemd/system/boo.service`
- `sudo systemctl daemon-reload`
- `sudo systemctl enable boo.service`
- `sudo systemctl start boo`
- `sudo systemctl status boo`

