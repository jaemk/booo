
import time

try:
    import RPi.GPIO as pio
    def init_board():
        pio.setmode(pio.BOARD)

    def init_out_pins(*pins):
        for pin in pins:
            pio.setup(pin, pio.OUT)

    def on(pin):
        pio.output(pin, pio.HIGH)

    def off(pin):
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

