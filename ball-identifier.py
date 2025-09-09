import sensor, image, time

# ---------------------------
# Camera Setup
# ---------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# ---------------------------
# Parameters
# ---------------------------
orange_threshold = (30, 80, 15, 70, 30, 80)
pixels_threshold = 50
area_threshold = 50

BALL_DIAMETER_CM = 4.3
FOCAL_LENGTH = 0  # will be set after calibration
clock = time.clock()

# ---------------------------
# Calibration Function
# ---------------------------
def calibrate_focal_length(known_distance_cm):
    print("Calibration started. Place the ball at {} cm from the camera.".format(known_distance_cm))
    while True:
        img = sensor.snapshot()
        blobs = img.find_blobs([orange_threshold], pixels_threshold=pixels_threshold,
                                area_threshold=area_threshold, merge=True)
        if blobs:
            blob = max(blobs, key=lambda b: b.pixels())
            if blob.elongation() < 0.5:
                perceived_diameter = (blob.w() + blob.h()) / 2
                focal_length = (perceived_diameter * known_distance_cm) / BALL_DIAMETER_CM
                print("Calibration complete!")
                print("Perceived diameter: {:.2f} px".format(perceived_diameter))
                print("Calculated FOCAL_LENGTH: {:.2f} px".format(focal_length))
                return focal_length
        else:
            print("No ball detected. Adjust the position or lighting.")

# ---------------------------
# Run Calibration
# ---------------------------
if FOCAL_LENGTH == 0:
    FOCAL_LENGTH = calibrate_focal_length(known_distance_cm=40)

# ---------------------------
# Main Loop
# ---------------------------
while True:
    clock.tick()
    img = sensor.snapshot()
    ball_found = False

    # Step 1: Color Detection
    blobs = img.find_blobs([orange_threshold], pixels_threshold=pixels_threshold,
                            area_threshold=area_threshold, merge=True)

    if blobs:
        # Take the largest roughly circular blob
        blob = max(blobs, key=lambda b: b.pixels())
        if blob.elongation() < 0.5:
            # Crop the blob region to focus circle detection
            roi = (blob.x(), blob.y(), blob.w(), blob.h())
            roi_img = img.copy(roi=roi)

            # Step 2: Circle detection inside the blob only
            circles = roi_img.find_circles(threshold=2000, x_margin=0, y_margin=0,
                                           r_margin=2, r_min=5, r_max=30, r_step=2)

            if circles:
                # Take the largest circle inside the blob
                largest_circle = max(circles, key=lambda c: c.r())

                # Draw circle on original image (adjust coordinates)
                img.draw_circle(largest_circle.x() + blob.x(), largest_circle.y() + blob.y(),
                                largest_circle.r())

                # Distance calculation
                perceived_diameter = 2 * largest_circle.r()
                distance_cm = (BALL_DIAMETER_CM * FOCAL_LENGTH) / perceived_diameter

                print("Ball detected at X:", largest_circle.x() + blob.x(),
                      "Y:", largest_circle.y() + blob.y(),
                      "Distance: {:.2f} cm".format(distance_cm))
                ball_found = True
            else:
                # If no circle found inside blob, fallback to blob center
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
                perceived_diameter = (blob.w() + blob.h()) / 2
                distance_cm = (BALL_DIAMETER_CM * FOCAL_LENGTH) / perceived_diameter
                print("Ball (blob fallback) at X:", blob.cx(), "Y:", blob.cy(),
                      "Distance: {:.2f} cm".format(distance_cm))
                ball_found = True

    if not ball_found:
        print("No ball detected")
