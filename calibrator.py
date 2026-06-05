# python
import sensor
import time
import machine

class Color:
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

class CalibrationCamera:
    def __init__(self, exit_pin: str | int, focal_length: float = 0.0, highestGain: int = 12) -> None:
        # Camera setup
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=2000)

        sensor.set_auto_exposure(True)
        # sensor.set_auto_gain(False, gain_db=highestGain)

        self.highestGain = highestGain

        # self.ledR = LED("LED_RED")
        # self.ledG = LED("LED_GREEN")
        # self.ledB = LED("LED_BLUE")
        # self.ledB.on()

        # Color thresholds
        self.thresholds = {
            # "orange": (13, 100, 40, 10, 20, 101),
            # "orange": (0, 100, -128, 127, 26, 127),
            # "orange": (0, 100, -13, 127, 25, 127),
            # "orange": (0, 100, -11, 127, 12, 127),
            # "orange": (0, 100, 12, 127, 12, 127),
            "orange": (0, 100, 20, 127, 21, 127),
        }

        # Detection params
        self.ball_pixels_threshold = 50
        self.ball_area_threshold = 50

        # Physical / camera params
        self.BALL_DIAMETER_CM = 4.3
        self.GOAL_DIAMETER_CM = 64.0
        self.focal_length = float(focal_length or 0.0)
        self.camera_height_cm = 13.0
        self.MAX_LENGTH = 200 #cm

        # Working vars
        self.img = None
        self.blob = None
        self.circles = None

        if self.focal_length == 0:
            self.calibrate_focal_length(known_distance_cm=40)

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

    def check_image(self) -> bool:
        self.img = sensor.snapshot()

        orange_blobs = self.img.find_blobs([self.thresholds["orange"]],
                                           pixels_threshold=self.ball_pixels_threshold,
                                           area_threshold=self.ball_area_threshold, merge=True)

        ball_distance = self._process_ball(orange_blobs) if orange_blobs else None
        print(ball_distance)
        return (ball_distance != 0 and ball_distance is not None)
        # <>

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

    def run(self, triesPerGain: int = 10) -> None:
        results: dict[int, int] = {}
        for gain in range(self.highestGain, 0, -1):
            sensor.set_auto_gain(False, gain_db=gain)
            count = 0
            for _ in range(triesPerGain):
                try:
                    count += 1 if self.check_image() else 0
                    time.sleep(0.01)
                except ZeroDivisionError as e:
                    print(e)
            print(f"gain: {gain}, count: {count}")
            results[gain] = count

        print(results)

    def setLedColor(self, color: int):
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
    camera = CalibrationCamera(exit_pin="P3", focal_length=265.12, highestGain=30)
    camera.run()

if __name__ == "__main__":
    main()
