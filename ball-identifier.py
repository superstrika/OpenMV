from curses.textpad import rectangle

import sensor, time
import math
from machine import UART, Pin

class CameraDetection:
    def __init__(self, exitPin: str, focalLength: float=0) -> None:
        # Camera Setup
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)

        # Parameters
        self.orange_threshold = (46, 100, -128, 127, 20, 127)
        self.blue_threshold = (0, 47, -7, 127, -128, -18)
        self.yellow_threshold = (39, 100, -31, 3, 18, 127)

        self.ball_pixels_threshold = 50
        self.ball_area_threshold = 50

        # Goal detection parameters
        self.goal_pixels_threshold = 100  # Higher threshold for larger goal
        self.goal_area_threshold = 200    # Higher area threshold for goal

        self.BALL_DIAMETER_CM = 4.3
        self.GOAL_DIAMETER_CM = 64
        self.FOCAL_LENGTH = focalLength  # will be re-set after calibration
        self.clock = time.clock()

        #UART Setup
        self.comms = UART(3, 9600)
        self.comms.init(9600, bits=8, parity=None, stop=1)

        # ExitPin Setup
        self.EXIT = False

        self.exitPin = Pin(exitPin, Pin.IN, Pin.PULL_UP)
        self.exitPin.irq(trigger=Pin.IRQ_FALLING, handler=self.exit_handler)

        #Vars Setup
        self.bolb = None
        self.bolbs = None
        self.img = None
        self.distance_cm = None

        #Distance Calculation Setup
        self.cameraHeight: int = 13 #cm

        #Begin setup
        self.begin()

    def exit_handler(self, pin: Pin) -> None:
        self.EXIT = True

    def calibateFocalLength(self, known_distance_cm: int) -> None:
        print("Calibration started. Place the ball at {} cm from the camera.".format(known_distance_cm))
        while self.FOCAL_LENGTH != focal_length:
            img = sensor.snapshot()
            blobs = img.find_blobs([self.orange_threshold], pixels_threshold=self.ball_pixels_threshold,
                                    area_threshold=self.ball_area_threshold, merge=True)
            if blobs:
                self.blob = max(blobs, key=lambda b: b.pixels())
                if self.blob.elongation() < 0.5:
                    perceived_diameter = (self.blob.w() + self.blob.h()) / 2
                    focal_length = (perceived_diameter * known_distance_cm) / self.BALL_DIAMETER_CM
                    print("Calibration complete!")
                    print("Perceived diameter: {:.2f} px".format(perceived_diameter))
                    print("Calculated FOCAL_LENGTH: {:.2f} px".format(focal_length))
                    self.FOCAL_LENGTH = focal_length
            else:
                print("No ball detected. Adjust the position or lighting.")

    def begin(self) -> None:
        if self.FOCAL_LENGTH == 0:
            self.calibateFocalLength(known_distance_cm=40)

    def checkImage(self) -> None:
        self.clock.tick()
        self.img = sensor.snapshot()


        # Color Detection
        self.blobs = self.img.find_blobs([self.orange_threshold], pixels_threshold=self.ball_pixels_threshold,
                                area_threshold=self.ball_area_threshold, merge=True)


        self.blueGoalBlobs = self.img.find_blobs([self.blue_threshold],
                                            pixels_threshold=self.goal_pixels_threshold,
                                            area_threshold=self.goal_area_threshold,
                                            merge=True)

        self.yellowGoalBlobs = self.img.find_blobs([self.yellow_threshold],
                                           pixels_threshold=self.goal_pixels_threshold,
                                           area_threshold=self.goal_area_threshold,
                                           merge=True)


        # Find the largest blob
        if self.blobs:
            ball_distance_cm = self.ballDetected() #distance from camera to ball

            if ball_distance_cm is not None:
                distanceXY: tuple = self.CalculateXY(ball_distance_cm)
        else:
            distanceXY: tuple = 0,0

        if self.blueGoalBlobs:
            blue_goal_distance_cm = self.goalDetected()
        else:
            blue_goal_distance_cm = 0

        if self.yellowGoalBlobs:
            yellow_goal_distance_cm = self.goalDetected()
        else:
            yellow_goal_distance_cm = 0

        data = "{:.2f}#{:.2f}#{:.2f}#{:.2f}\n".format(*distanceXY, blue_goal_distance_cm, yellow_goal_distance_cm)
        print(data)
        self.comms.write(data)


    def ballDetected(self) -> int:
        self.blob = max(self.blobs, key=lambda b: b.pixels())
        if self.blob.elongation() < 0.5:
            # Crop the blob region to focus circle detection
            roi = (self.blob.x(), self.blob.y(), self.blob.w(), self.blob.h())
            roi_img = self.img.copy(roi=roi)

            # Step 2: Circle detection inside the blob only
            self.circles = roi_img.find_circles(threshold=2000, x_margin=0, y_margin=0,
                                           r_margin=2, r_min=1, r_max=30, r_step=2)

            screenCenter = self.img.width() / 2
            self.img.draw_line(int(screenCenter), 0, int(screenCenter), self.img.height())


            if self.circles:
                return self.CircleFound()
            # else:
                # return self.CircleNotFound()
        return None

    def goalDetected(self, bolb) -> int:
        if not bolb:
            return None

        bolb = max(bolb, key=lambda b: b.pixels())

        roi = (bolb.x(), bolb.y(), bolb.w(), bolb.h())
        roi_img = self.img.copy(roi=roi)

        rectangles = roi_img.find_rects(threshold=2000)
        if rectangles:
            return self.RectangleFound(rectangles)
        else:
            # Fallback: use blob dimensions if no rectangle found
            print("No rectangle found in goal blob, using blob dimensions")
            return self.GoalBlobFallback(bolb)

    def CircleFound(self) -> int:
        largest_circle = max(self.circles, key=lambda c: c.r())

        # Draw circle on original image (adjust coordinates)
        self.img.draw_circle(largest_circle.x() + self.blob.x(), largest_circle.y() + self.blob.y(),
                        largest_circle.r())

        # Distance calculation
        perceived_diameter = 2 * largest_circle.r()
        distance_cm = (self.BALL_DIAMETER_CM * self.FOCAL_LENGTH) / perceived_diameter

        # Angle calculation

        # img_center_x = self.img.width() / 2

        # print("Ball detected at X:", largest_circle.x() + self.blob.x(),
        #       "Y:", largest_circle.y() + self.blob.y())

        return distance_cm

    def RectangleFound(self, rectangles) -> int:
        largest_rectangle = max(rectangles, key=lambda r: r.w())

        # Draw rectangle on original image (adjust coordinates)
        self.img.draw_rectangle(largest_rectangle.rect())

        # Distance calculation
        perceived_diameter = largest_rectangle.w()
        distance_cm = (self.GOAL_DIAMETER_CM * self.FOCAL_LENGTH) / perceived_diameter

        return distance_cm

    def GoalBlobFallback(self, bolb) -> int:
        # Fallback method when rectangle detection fails
        # Use blob dimensions for distance calculation
        self.img.draw_rectangle(bolb.rect())

        # Use the width of the blob as perceived diameter
        perceived_diameter = bolb.w()
        distance_cm = (self.GOAL_DIAMETER_CM * self.FOCAL_LENGTH) / perceived_diameter

        return distance_cm

    def CircleNotFound(self) -> None:
        # If no circle found inside blob, fallback to blob center
        self.img.draw_rectangle(self.blob.rect())
        self.img.draw_cross(self.blob.cx(), self.blob.cy())
        perceived_diameter = (self.blob.w() + self.blob.h()) / 2
        distance_cm = (self.BALL_DIAMETER_CM * self.FOCAL_LENGTH) / perceived_diameter

        # print("Ball (blob fallback) at X:", self.blob.cx(), "Y:", self.blob.cy())

        return distance_cm

    def CalculateXY(self, distance: float) -> tuple[]:
        dXY: float = math.sqrt(distance**2 - self.cameraHeight**2)

        screenCenter: float = self.img.width() / 2
        ballX: float = self.blob.cx() - screenCenter

        PxToCmRatio = (self.blob.w()) / self.BALL_DIAMETER_CM

        dX: float = ballX / PxToCmRatio
        dY: float = math.sqrt(dXY**2 - dX**2)

        return dX,dY

    def run(self) -> None:
        # self.checkImage()
        while not self.EXIT:
            self.checkImage()


def main() -> None:
    camera = CameraDetection(exitPin="P0", focalLength=265.12)
    camera.run()

if __name__ == "__main__":
    main()
