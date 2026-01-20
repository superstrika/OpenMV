import sensor
import time
import math
from machine import Pin, LED
import pyb

class Color():
    RED = 0
    GREEN = 1
    BLUE = 2
    WHITE = 3
    YELLOW = 4
    CYAN = 5
    PURPLE = 6

    def __init__(self, color: int):
        self._value = color

    def __eq__(self, other: int):
        return self._value == other

class CameraDetection:
    def __init__(self, exit_pin: str | int, focal_length: float = 0.0) -> None:
        # Camera setup
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        self.ledR = LED("LED_RED")
        self.ledG = LED("LED_GREEN")
        self.ledB = LED("LED_BLUE")
        self.ledB.on()


        # Color thresholds
        self.thresholds = {
            "orange": (16, 100, 12, 127, 20, 127),
            "blue": (0, 48, -7, 127, -128, -10),
            "yellow": (39, 100, -31, 3, 18, 127),
        }

        # Detection params
        self.ball_pixels_threshold = 50
        self.ball_area_threshold = 50
        self.goal_pixels_threshold = 100
        self.goal_area_threshold = 200

        # Physical / camera params
        self.BALL_DIAMETER_CM = 4.3
        self.GOAL_DIAMETER_CM = 64.0
        self.focal_length = float(focal_length or 0.0)
        self.camera_height_cm = 13.0
        self.MAX_LENGTH = 200 #cm

        # Exit pin
        self.EXIT = False
        self.exit_pin = Pin(exit_pin, Pin.IN, Pin.PULL_DOWN)
        try:
            self.exit_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._exit_handler)
        except Exception as e:
            print(e)

        #Usb connection
        self._usb = pyb.USB_VCP()

        # Working vars
        self.img = None
        self.blob = None
        self.circles = None

        if self.focal_length == 0:
            self.calibrate_focal_length(known_distance_cm=40)

    def _exit_handler(self, pin: Pin) -> None:
        self.EXIT = True

    def calibrate_focal_length(self, known_distance_cm: int, timeout_s: int = 15) -> None:
        """Calibrate focal length using an orange ball at a known distance."""
        start = time.time()
        print("Calibration: place orange ball at {} cm".format(known_distance_cm))
        while time.time() - start < timeout_s and self.focal_length == 0:
            img = sensor.snapshot()
            blobs = img.find_blobs([self.thresholds["orange"]],
                                   pixels_threshold=self.ball_pixels_threshold,
                                   area_threshold=self.ball_area_threshold, merge=True)
            if not blobs:
                continue
            b = max(blobs, key=lambda x: x.pixels())
            if b.elongation() < 0.5:
                perceived = (b.w() + b.h()) / 2.0
                if perceived > 0:
                    self.focal_length = (perceived * known_distance_cm) / self.BALL_DIAMETER_CM
                    print("Calibration done. FOCAL_LENGTH        = {:.2f}".format(self.focal_length))
                    return
        print("Calibration ended. FOCAL_LENGTH = {:.2f}".format(self.focal_length))

    def check_image(self) -> None:
        self.img = sensor.snapshot()



        orange_blobs = self.img.find_blobs([self.thresholds["orange"]],
                                           pixels_threshold=self.ball_pixels_threshold,
                                           area_threshold=self.ball_area_threshold, merge=True)
        blue_blobs = self.img.find_blobs([self.thresholds["blue"]],
                                         pixels_threshold=self.goal_pixels_threshold,
                                         area_threshold=self.goal_area_threshold, merge=True)
        yellow_blobs = self.img.find_blobs([self.thresholds["yellow"]],
                                           pixels_threshold=self.goal_pixels_threshold,
                                           area_threshold=self.goal_area_threshold, merge=True)

        ball_distance = self._process_ball(orange_blobs) if orange_blobs else None
        blue_distance = self._process_goal(blue_blobs) if blue_blobs else None
        yellow_distance = self._process_goal(yellow_blobs) if yellow_blobs else None

        if ball_distance:
            x_cm, y_cm = self.calculate_xy(ball_distance, orange_blobs, self.BALL_DIAMETER_CM)
            self.setLedColor(Color.PURPLE)
        else:
            x_cm, y_cm = 0.0, 0.0
            self.setLedColor(Color.WHITE)

        if blue_distance:
            blueX, blueY = self.calculate_xy(blue_distance, blue_blobs, self.GOAL_DIAMETER_CM)
        else:
            blueX, blueY = 0.0, 0.0

        if yellow_distance:
            yellowX, yellowY = self.calculate_xy(yellow_distance, yellow_blobs, self.GOAL_DIAMETER_CM)
        else:
            yellowX, yellowY = 0.0, 0.0

        # draw center vertical for visualization
        center_x = int(self.img.width() / 2)
        self.img.draw_line(center_x, 0, center_x, self.img.height())

        dis = [x_cm, y_cm, blueX, blueY, yellowX, yellowY]
        for i in range(len(dis)):
            if dis[i] >= self.MAX_LENGTH:
                dis[i] = "0"
            else:
                dis[i] = str(dis[i])

        # print(type(dis))
        data = '#'.join(dis)
        data += "\n"
        #print(data.strip())
        try:
            if self._usb.isconnected():
                self._usb.send(data)
        except Exception as e:
            print(e)

    def _process_ball(self, blobs: list) -> float | None:
        self.blob = max(blobs, key=lambda b: b.pixels())
        if self.blob.elongation() >= 0.5:
            return None

        roi = (self.blob.x(), self.blob.y(), self.blob.w(), self.blob.h())
        roi_img = self.img.copy(roi=roi)
        self.circles = roi_img.find_circles(threshold=2000, x_margin=0, y_margin=0,
                                            r_margin=2, r_min=1, r_max=30, r_step=2)

        if self.circles:
            return self._circle_found()
        return self._circle_not_found()

    def _circle_found(self) -> float:
        c = max(self.circles, key=lambda c: c.r())
        # draw circle on original image (adjust coords)
        self.img.draw_circle(c.x() + self.blob.x(), c.y() + self.blob.y(), c.r())
        perceived = 2 * c.r()
        return (self.BALL_DIAMETER_CM * self.focal_length) / perceived

    def _circle_not_found(self) -> float:
        self.img.draw_rectangle(self.blob.rect())
        self.img.draw_cross(self.blob.cx(), self.blob.cy())
        perceived = (self.blob.w() + self.blob.h()) / 2.0
        return (self.BALL_DIAMETER_CM * self.focal_length) / perceived

    def _process_goal(self, blobs: list) -> float:
        b = max(blobs, key=lambda x: x.pixels())
        roi = (b.x(), b.y(), b.w(), b.h())
        roi_img = self.img.copy(roi=roi)
        rects = roi_img.find_rects(threshold=2000)
        if rects:
            return self._rectangle_found(rects)
        return self._goal_blob_fallback(b)

    def _rectangle_found(self, rects: list) -> float:
        r = max(rects, key=lambda x: x.w())
        self.img.draw_rectangle(r.rect())
        perceived = r.w()
        return (self.GOAL_DIAMETER_CM * self.focal_length) / perceived

    def _goal_blob_fallback(self, b) -> float:
        self.img.draw_rectangle(b.rect())
        perceived = b.w()
        return (self.GOAL_DIAMETER_CM * self.focal_length) / perceived

    def calculate_xy(self, distance_cm: float, blobs, diameter_cm: float) -> tuple:
        blob = max(blobs, key=lambda b: b.pixels())
        d_xy = math.sqrt(max(distance_cm**2 - self.camera_height_cm**2, 0.0))
        screen_center = self.img.width() / 2.0
        ball_x_px = blob.cx() - screen_center
        px_to_cm = (blob.w()) / diameter_cm
        d_x = ball_x_px / px_to_cm
        d_y = math.sqrt(max(d_xy**2 - d_x**2, 0.0))
        return d_x, d_y

    def run(self) -> None:
        # for _ in range(2000):
        #     self.check_image()
        while not self.EXIT:
            # print(self.exit_pin.value())
            self.check_image()
            time.sleep(0.02)

    def setLedColor(self, color: Color):
        if color == Color.WHITE:
            self.ledR.on()
            self.ledG.on()
            self.ledB.on()
        elif color == Color.RED:
            self.ledR.on()
            self.ledG.off()
            self.ledB.off()
        elif color == Color.GREEN:
            self.ledR.off()
            self.ledG.on()
            self.ledB.off()
        elif color == Color.BLUE:
            self.ledR.off()
            self.ledG.off()
            self.ledB.on()
        elif color == Color.YELLOW:
            self.ledR.on()
            self.ledG.on()
            self.ledB.off()
        elif color == Color.CYAN:
            self.ledR.off()
            self.ledG.on()
            self.ledB.on()
        elif color == Color.PURPLE:
            self.ledR.on()
            self.ledG.off()
            self.ledB.on()

def main():
    camera = CameraDetection(exit_pin="P3", focal_length=265.12)
    camera.run()

if __name__ == "__main__":
    main()
