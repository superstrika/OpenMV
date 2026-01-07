from machine import UART, LED
from time import sleep

comms = UART(1, 115200)
led = LED("LED_BLUE")
try:
    comms.init(115200, parity=None, stop=1)
except Exception as e:
    print(e)

while True:
    led.on()
    msg = "Hello world!\n"
    comms.write(msg.encode())
    print(msg)
    sleep(1)

