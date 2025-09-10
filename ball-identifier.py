import sensor, time
from machine import UART, Pin


class CameraDetection:
    def __init__(self, exitPin: int) -> None:
        # Camera Setup
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)

        # Parameters
        self.orange_threshold = (46, 100, -128, 127, 20, 127)

        self.pixels_threshold = 50
        self.area_threshold = 50

        self.BALL_DIAMETER_CM = 4.3
        self.FOCAL_LENGTH = 311.63  # will be re-set after calibration
        self.clock = time.clock()

        #UART Setup
        self.comms = UART(2, 115200)
        self.comms.init(115200, bits=8, parity=None, stop=1)

        #ExitPin Setup
        self.EXIT = False

        self.exitPin = Pin(exitPin, Pin.IN, Pin.PULL_UP)
        self.exitPin.irq(trigger=Pin.IRQ_FALLING, handler=self.exit_handler)

        #Vars Setup
        self.bolb = None
        self.img = None
        self.distance_cm = None

        #Begin setup
        self.begin()

    def exit_handler(self, pin: Pin) -> None:
        self.EXIT = True

    def calibateFocalLength(self, known_distance_cm: int) -> None:
        print("Calibration started. Place the ball at {} cm from the camera.".format(known_distance_cm))
        while True:
            img = sensor.snapshot()
            blobs = img.find_blobs([self.orange_threshold], pixels_threshold=self.pixels_threshold,
                                    area_threshold=self.area_threshold, merge=True)
            if blobs:
                blob = max(blobs, key=lambda b: b.pixels())
                if blob.elongation() < 0.5:
                    perceived_diameter = (blob.w() + blob.h()) / 2
                    focal_length = (perceived_diameter * known_distance_cm) / self.BALL_DIAMETER_CM
                    print("Calibration complete!")
                    print("Perceived diameter: {:.2f} px".format(perceived_diameter))
                    print("Calculated FOCAL_LENGTH: {:.2f} px".format(focal_length))
                    self.FOCAL_LENGTH = focal_length
            else:
                print("No ball detected. Adjust the position or lighting.")

    def begin(self) -> None:
        if self.FOCAL_LENGTH == 0:
            self.FOCAL_LENGTH = self.calibrate_focal_length(known_distance_cm=40)

    def checkImage(self) -> None:
        self.clock.tick()
        self.img = sensor.snapshot()

        # Color Detection
        self.blobs = self.img.find_blobs([self.orange_threshold], pixels_threshold=self.pixels_threshold,
                                area_threshold=self.area_threshold, merge=True)

        # Find the largest blob
        if self.blobs:
            distance_cm = self.ballDetected()

            if distance_cm is None:
                print("No ball detected")
                self.comms.write("0\n")
            else:
                print("ball detected: {:.2f}\n".format(distance_cm))
                data = "{:.2f}\n".format(distance_cm)
                self.comms.write(data)

    def ballDetected(self) -> int:
        blob = max(self.blobs, key=lambda b: b.pixels())
        if blob.elongation() < 0.5:
            # Crop the blob region to focus circle detection
            roi = (self.blob.x(), self.blob.y(), self.blob.w(), self.blob.h())
            roi_img = self.img.copy(roi=roi)

            # Step 2: Circle detection inside the blob only
            self.circles = roi_img.find_circles(threshold=2000, x_margin=0, y_margin=0,
                                           r_margin=2, r_min=1, r_max=30, r_step=2)

            if self.circles:
                return self.CircleFound()
            else:
                return self.CircleNotFound()
        return None

    def CircleFound(self) -> None:
        largest_circle = max(self.circles, key=lambda c: c.r())

        # Draw circle on original image (adjust coordinates)
        self.img.draw_circle(largest_circle.x() + self.blob.x(), largest_circle.y() + self.blob.y(),
                        largest_circle.r())

        # Distance calculation
        perceived_diameter = 2 * largest_circle.r()
        distance_cm = (self.BALL_DIAMETER_CM * self.FOCAL_LENGTH) / perceived_diameter

        # print("Ball detected at X:", largest_circle.x() + blob.x(),
        #       "Y:", largest_circle.y() + blob.y(),
        #       "Distance: {:.2f} cm".format(distance_cm))

        return distance_cm

    def CircleNotFound(self) -> None:
        # If no circle found inside blob, fallback to blob center
        self.img.draw_rectangle(self.blob.rect())
        self.img.draw_cross(self.blob.cx(), self.blob.cy())
        perceived_diameter = (self.blob.w() + self.blob.h()) / 2
        distance_cm = (self.BALL_DIAMETER_CM * self.FOCAL_LENGTH) / perceived_diameter

        # print("Ball (blob fallback) at X:", blob.cx(), "Y:", blob.cy(),
        #       "Distance: {:.2f} cm".format(distance_cm))

        return distance_cm

    def run(self) -> None:
        while not self.EXIT:
            self.checkImage()


def main() -> None:
    camera = CameraDetection(exitPin=14)
    camera.run()

if __name__ == "__main__":
    main()
