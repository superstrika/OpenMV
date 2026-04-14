# Untitled - By: noam2 - Mon Apr 13 2026

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_exposure(True)
# This helps the camera adjust to the average brightness of the field
# sensor.set_get_signals(True)
# Lock the gain but let exposure float to handle brightness changes
sensor.set_auto_gain(False, gain_db=12)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
