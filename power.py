from __future__ import print_function


try:
    import RPi.GPIO as pio
    def init_board():
        print("Initializing board mode")
        pio.setmode(pio.BOARD)

    def init_out_pins(*pins):
        print("Initializing pins: {}".format(pins))
        for pin in pins:
            pio.setup(pin, pio.OUT)

    def on(pin):
        print("Toggling ON: PIN-{}".format(pin))
        pio.output(pin, pio.HIGH)

    def off(pin):
        print("Toggling OFF: PIN-{}".format(pin))
        pio.output(pin, pio.LOW)

except ImportError:
    def warn(msg):
        print("[WARNING] RPi module not available: {}".format(msg))

    def init_board():
        warn("Initializing board mode")

    def init_out_pins(*pins):
        warn("Initializing pins: {}".format(pins))

    def on(pin):
        warn("Toggling ON: PIN-{}".format(pin))

    def off(pin):
        warn("Toggling OFF: PIN-{}".format(pin))

