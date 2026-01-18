import time, pyb
from machine import LED

# Initialize the USB Virtual COM Port
usb = pyb.USB_VCP()
led = LED("LED_RED")

while(True):
    # Example: Send a detection message or coordinate
    message = "Hello Raspberry Pi! Count: " + str(time.ticks_ms()) + "\n"
    led.toggle()
    # Check if USB is connected before sending
    if usb.isconnected():
        usb.send(message)

    time.sleep_ms(1000)
