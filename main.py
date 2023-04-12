from picar import front_wheels, back_wheels
from picar.SunFounder_PCA9685 import Servo
import picar
import time
from time import sleep
import cv2
import numpy as np
import picar
import os
import toml
from datetime import datetime
import shutil
import board
import neopixel


picar.setup()
pixels = neopixel.NeoPixel(
    board.D12, 15, brightness=0.2, auto_write=False, pixel_order=neopixel.GRB
)

# Get configuration values
config = toml.load('config.toml')

img_dimension = config['image']['dimension']
friction_threshold = config['image']['threshold']
max_low_friction = config['runway']['percent']

runway_length = config['runway']['length']
runway_width = config['runway']['width']
runway_units = config['runway']['units']
if(runway_units == 'feet'):
    runway_length /= 12
    runway_width /= 12


# Create rubber values array and images array from runway dimensions in terms of image size
num_imgs_length = (runway_length) // img_dimension
num_imgs_width = (runway_width) // img_dimension
vals_arr = [[-1] * num_imgs_length for i in range(num_imgs_width)]
imgs_arr = [[-1] * num_imgs_length for i in range(num_imgs_width)]

# Create new directory for images to be saved to named the current date and time
cwd = os.getcwd()
img_dir = os.path.join(cwd, "images")

start_img_dir = os.path.join(img_dir, "start")

dt_string = datetime.now().strftime("%Y-%m-%d_%H-%M")
new_img_dir = os.path.join(img_dir, dt_string)


# Show image captured by camera, True to turn on, you will need #DISPLAY and it also slows the speed of tracking
show_image_enable   = False
draw_circle_enable  = False
scan_enable         = False
rear_wheels_enable  = True
front_wheels_enable = True
pan_tilt_enable     = True

# if (show_image_enable or draw_circle_enable) and "DISPLAY" not in os.environ:
#     print('Warning: Display not found, turn off "show_image_enable" and "draw_circle_enable"')
#     show_image_enable   = False
#     draw_circle_enable  = False

kernel = np.ones((5,5),np.uint8)
cam = cv2.VideoCapture(0)

# if not cam.isOpened:
#     print("not open")
# else:
#     print("open")

# SCREEN_WIDTH = 160
# SCREEN_HIGHT = 120
# cam.set(3,SCREEN_WIDTH)
# cam.set(4,SCREEN_HIGHT)
# CENTER_X = SCREEN_WIDTH/2
# CENTER_Y = SCREEN_HIGHT/2
# BALL_SIZE_MIN = SCREEN_HIGHT/10
# BALL_SIZE_MAX = SCREEN_HIGHT/3

# Filter setting, DONOT CHANGE
hmn = 12
hmx = 37
smn = 96
smx = 255
vmn = 186
vmx = 255

# camera follow mode:
# 0 = step by step(slow, stable),
# 1 = calculate the step(fast, unstable)
follow_mode = 1

CAMERA_STEP = 2
CAMERA_X_ANGLE = 20
CAMERA_Y_ANGLE = 20

MIDDLE_TOLERANT = 5
PAN_ANGLE_MAX   = 170
PAN_ANGLE_MIN   = 10
TILT_ANGLE_MAX  = 150
TILT_ANGLE_MIN  = 70
FW_ANGLE_MAX    = 90+30
FW_ANGLE_MIN    = 90-30

SCAN_POS = [[20, TILT_ANGLE_MIN], [50, TILT_ANGLE_MIN], [90, TILT_ANGLE_MIN], [130, TILT_ANGLE_MIN], [160, TILT_ANGLE_MIN],
            [160, 80], [130, 80], [90, 80], [50, 80], [20, 80]]

bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
pan_servo = Servo.Servo(1)
tilt_servo = Servo.Servo(2)
picar.setup()

fw.offset = 0
pan_servo.offset = 10
tilt_servo.offset = 0

bw.speed = 0
bw_default_speed = 50
fw.turn(90)
pan_servo.write(90)
tilt_servo.write(90)

motor_speed = 60

def nothing(x):
    pass

def main():

    print("Program starting")

    print("Ready to begin analysis?")
    value = input()
    while value != 'y':
        print("Ready to begin analysis?")
        value = input()

    turn_on_leds()
    print("LEDS on")

    # Make new directory for current images
    os.mkdir(new_img_dir)
    print(f"Writing images to {new_img_dir}")

    low_friction_ctr = 0

    # Gather images and analyze the grayscale values of each
    for i in range(len(vals_arr)):
        print(f"Taking images of column {i+1} of {num_imgs_width}")
        col = i + 1

        for j in range(len(vals_arr[i])):
            # Account for serpentine pattern in row number
            if i % 2 == 1:
                row = num_imgs_length - j
            else:
                row = j + 1

            # Get image of clean runway at current point
            start_img_path = os.path.join(start_img_dir, f"w{col}_l{row}.jpg")
            start = cv2.imread(start_img_path)

            # TEMPORARY - should take image
            # current_path = os.path.join(img_dir, "current.jpg")
            # current = cv2.imread(current_path)

            # Take image of current state of runway at current point and save it to file
            ret, current = cam.read()
            if not ret:
                print("Picture failed, program ending")
                return

            img_name = f"w{col}_l{row}.jpg"
            img_path = os.path.join(new_img_dir, img_name)
            cv2.imwrite(img_path, current)

            # Subtract current image from start image, invert, and convert to grayscale (single value per pixel)
            subtracted = cv2.subtract(start,current)
            subtracted = cv2.bitwise_not(subtracted)
            grayscale = cv2.cvtColor(subtracted, cv2.COLOR_BGR2GRAY)

            # Save image to array to current point
            imgs_arr[col - 1][row - 1] = grayscale

            # Calculate average gray value of picture and save to values array at current point
            mean_val = cv2.mean(grayscale)[0]
            vals_arr[col-1][row-1] = mean_val

            # If gray value is less than acceptable level, add to counter
            # Higher numbers mean less black in image, therefore less difference from clean runway
            if mean_val < friction_threshold:
                low_friction_ctr += 1

            print(f"\tImage {j+1} of {num_imgs_length} in column -> {mean_val}")

            # Move forward to next point if not at end of runway
            if j != (len(vals_arr[i]) - 1):
                move_straight(0.9)

        print(f"Finished column {col}\n")
        if i != (len(vals_arr) - 1):
            print("Please move robot to next column and enter y when ready: ")
            value = input()
            while value != 'y':
                print("Please enter y when ready: ")
                value = input()
    print(f"Finished taking images\n")


    # Analyze images
    runway_area = runway_length * runway_width
    if runway_units == 'feet':
        runway_area *= 144

    img_area = img_dimension * img_dimension

    low_friction_area = img_area * low_friction_ctr
    low_friction_percentage = low_friction_area / runway_area

    print(f"{low_friction_percentage * 100}% of the runway is above the acceptable rubber deposit limit.")
    print(f"This is above the set threshold of {max_low_friction * 100}%.")
    print(f"It is recommended to use a friction measuring device to confirm these readings and consider cleaning the runway.")






    #### Original code
    # pan_angle = 90              # initial angle for pan
    # tilt_angle = 90             # initial angle for tilt
    # fw_angle = 90

    # scan_count = 0
    # print("Begin!")
    # while True:
    #     x = 0             # x initial in the middle
    #     y = 0             # y initial in the middle
    #     r = 0             # ball radius initial to 0(no balls if r < ball_size)

    #     for _ in range(10):
    #         (tmp_x, tmp_y), tmp_r = find_blob()
    #         if tmp_r > BALL_SIZE_MIN:
    #             x = tmp_x
    #             y = tmp_y
    #             r = tmp_r
    #             break

    #     print(x, y, r)

    #     # scan:
    #     if r < BALL_SIZE_MIN:
    #         bw.stop()
    #         if scan_enable:
    #             #bw.stop()
    #             pan_angle = SCAN_POS[scan_count][0]
    #             tilt_angle = SCAN_POS[scan_count][1]
    #             if pan_tilt_enable:
    #                 pan_servo.write(pan_angle)
    #                 tilt_servo.write(tilt_angle)
    #             scan_count += 1
    #             if scan_count >= len(SCAN_POS):
    #                 scan_count = 0
    #         else:
    #             sleep(0.1)

    #     elif r < BALL_SIZE_MAX:
    #         if follow_mode == 0:
    #             if abs(x - CENTER_X) > MIDDLE_TOLERANT:
    #                 if x < CENTER_X:                              # Ball is on left
    #                     pan_angle += CAMERA_STEP
    #                     #print("Left   ", )
    #                     if pan_angle > PAN_ANGLE_MAX:
    #                         pan_angle = PAN_ANGLE_MAX
    #                 else:                                         # Ball is on right
    #                     pan_angle -= CAMERA_STEP
    #                     #print("Right  ",)
    #                     if pan_angle < PAN_ANGLE_MIN:
    #                         pan_angle = PAN_ANGLE_MIN
    #             if abs(y - CENTER_Y) > MIDDLE_TOLERANT:
    #                 if y < CENTER_Y :                             # Ball is on top
    #                     tilt_angle += CAMERA_STEP
    #                     #print("Top    " )
    #                     if tilt_angle > TILT_ANGLE_MAX:
    #                         tilt_angle = TILT_ANGLE_MAX
    #                 else:                                         # Ball is on bottom
    #                     tilt_angle -= CAMERA_STEP
    #                     #print("Bottom ")
    #                     if tilt_angle < TILT_ANGLE_MIN:
    #                         tilt_angle = TILT_ANGLE_MIN
    #         else:
    #             delta_x = CENTER_X - x
    #             delta_y = CENTER_Y - y
    #             #print("x = %s, delta_x = %s" % (x, delta_x))
    #             #print("y = %s, delta_y = %s" % (y, delta_y))
    #             delta_pan = int(float(CAMERA_X_ANGLE) / SCREEN_WIDTH * delta_x)
    #             #print("delta_pan = %s" % delta_pan)
    #             pan_angle += delta_pan
    #             delta_tilt = int(float(CAMERA_Y_ANGLE) / SCREEN_HIGHT * delta_y)
    #             #print("delta_tilt = %s" % delta_tilt)
    #             tilt_angle += delta_tilt

    #             if pan_angle > PAN_ANGLE_MAX:
    #                 pan_angle = PAN_ANGLE_MAX
    #             elif pan_angle < PAN_ANGLE_MIN:
    #                 pan_angle = PAN_ANGLE_MIN
    #             if tilt_angle > TILT_ANGLE_MAX:
    #                 tilt_angle = TILT_ANGLE_MAX
    #             elif tilt_angle < TILT_ANGLE_MIN:
    #                 tilt_angle = TILT_ANGLE_MIN

    #         if pan_tilt_enable:
    #             pan_servo.write(pan_angle)
    #             tilt_servo.write(tilt_angle)
    #         sleep(0.01)
    #         fw_angle = 180 - pan_angle
    #         if fw_angle < FW_ANGLE_MIN or fw_angle > FW_ANGLE_MAX:
    #             fw_angle = ((180 - fw_angle) - 90)/2 + 90
    #             if front_wheels_enable:
    #                 fw.turn(fw_angle)
    #             if rear_wheels_enable:
    #                 bw.speed = motor_speed
    #                 bw.backward()
    #         else:
    #             if front_wheels_enable:
    #                 fw.turn(fw_angle)
    #             if rear_wheels_enable:
    #                 bw.speed = motor_speed
    #                 bw.forward()
    #     else:
    #         bw.stop()



def destroy():
    bw.stop()
    cam.release()

def test():
    front_wheels_test()

    back_wheels_test()

def front_wheels_test():
    print("turn_left")
    fw.turn_left()
    time.sleep(1)
    print("turn_straight")
    fw.turn_straight()
    time.sleep(1)
    print("turn_right")
    fw.turn_right()
    time.sleep(1)
    print("turn_straight")
    fw.turn_straight()
    time.sleep(1)

def back_wheels_test():
    # bw = back_wheels.Back_Wheels()
    DELAY = 0.01
    try:
        bw.forward()
        for i in range(0, 100):
            bw.speed = i
            print("Forward, speed =", i)
            time.sleep(DELAY)
        for i in range(100, 0, -1):
            bw.speed = i
            print("Forward, speed =", i)
            time.sleep(DELAY)

        bw.backward()
        for i in range(0, 100):
            bw.speed = i
            print("Backward, speed =", i)
            time.sleep(DELAY)
        for i in range(100, 0, -1):
            bw.speed = i
            print("Backward, speed =", i)
            time.sleep(DELAY)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, motor stop")
        bw.stop()
    finally:
        print("Finished, motor stop")
        bw.stop()

def drive_loop():
    try:
        print("Move forward")
        bw.backward()
        bw.speed = 100

        for i in range(5):
            print("Loop" + str(i) + ":")
            print("Turn left")
            fw.turn_left()
            time.sleep(3)
            print("Turn right")
            fw.turn_right()
            time.sleep(3)
            print("\n")

    except KeyboardInterrupt:
            print("KeyboardInterrupt, motor stop")
            bw.stop()
            fw.turn_straight();
    finally:
            print("Finished, motor stop")
            bw.stop()
            fw.turn_straight();

def turn_90(dir):
    if(dir == "left"):
        print("Turn left")
        fw.turn_left()
    elif(dir == "right"):
        print("Turn right")
        fw.turn_right()
    else:
        print("Not a valid direction")
        return
    time.sleep(0.5)
    bw.forward()
    bw.speed = bw_default_speed
    time.sleep(3.4)
    bw.stop()
    fw.turn_straight()
    time.sleep(0.5)

def move_straight(t):
    bw.forward()
    bw.speed = bw_default_speed
    time.sleep(t)
    bw.stop()
    time.sleep(0.5)

def setup_start_imgs():

    # Create new directory for images to be saved to named the current date and time
    cwd = os.getcwd()
    img_dir = os.path.join(cwd, "images")

    start_img_dir = os.path.join(img_dir, "start")

    if os.path.isdir(start_img_dir):
        print(f"Do you want to empty the directory {start_img_dir} (y/n): ")
        value = input()
        while value != 'y' and value != 'n':
            print("Please enter y or n: ")
            value = input()

        if value == 'n':
            return False

        shutil.rmtree(start_img_dir)

    elif os.path.isfile(start_img_dir):
        print(f"Do you want to delete the file {start_img_dir} (y/n): ")
        value = input()
        while value != 'y' and value != 'n':
            print("Please enter y or n: ")
            value = input()

        if value == 'n':
            return False

        os.remove('file_path')

    print(f"Making directory {start_img_dir}")
    os.mkdir(start_img_dir)

    print(f"Writing images to {start_img_dir}")

    start_img = os.path.join(img_dir, "start.jpg")
    start_img = cv2.imread(start_img)


    for i in range(num_imgs_width):
        for j in range(num_imgs_length):
            start_img_path = os.path.join(start_img_dir, f"w{i+1}_l{j+1}.jpg")
            cv2.imwrite(start_img_path, start_img)


def turn_on_leds():
    for i in range(500):
        if i % 10 == 0:
            print(f"iteration {i}")
        pixels.fill((255, 255, 255))
        pixels.show()
        time.sleep(0.001)

    print("iterations done")
    pixels.fill((255, 255, 255))
    pixels.show()


# def find_ blob() :
    # radius = 0
    # # Load input image
    # #_, bgr_image = cam.read()
    # ret, bgr_image = cam.read()
    # if ret == False:
    #     print("Failed to read image")

    # orig_image = bgr_image

    # bgr_image = cv2.medianBlur(bgr_image, 3)

    # # Convert input image to HSV
    # hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    # # Threshold the HSV image, keep only the red pixels
    # lower_red_hue_range = cv2.inRange(hsv_image, (0, 100, 100), (10, 255, 255))
    # upper_red_hue_range = cv2.inRange(hsv_image, (160, 100, 100), (179, 255, 255))
    # # Combine the above two images
    # red_hue_image = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)

    # red_hue_image = cv2.GaussianBlur(red_hue_image, (9, 9), 2, 2)

    # # Use the Hough transform to detect circles in the combined threshold image
    # circles = cv2.HoughCircles(red_hue_image, cv2.HOUGH_GRADIENT, 1, 120, 100, 20, 10, 0)

    # if circles is not None:
    #     circles = np.uint16(np.around(circles))

    # # Loop over all detected circles and outline them on the original image
    #     all_r = np.array([])
    # # print("circles: %s"%circles)
    #     try:
    #         for i in circles[0,:]:
    #             # print("i: %s"%i)
    #             all_r = np.append(all_r, int(round(i[2])))
    #         closest_ball = all_r.argmax()
    #         center=(int(round(circles[0][closest_ball][0])), int(round(circles[0][closest_ball][1])))
    #         radius=int(round(circles[0][closest_ball][2]))
    #         if draw_circle_enable:
    #             cv2.circle(orig_image, center, radius, (0, 255, 0), 5)
    #     except IndexError:
    #         pass
    #         #print("circles: %s"%circles)

    # # Show images
    # if show_image_enable:
    #     cv2.namedWindow("Threshold lower image", cv2.WINDOW_AUTOSIZE)
    #     cv2.imshow("Threshold lower image", lower_red_hue_range)
    #     cv2.namedWindow("Threshold upper image", cv2.WINDOW_AUTOSIZE)
    #     cv2.imshow("Threshold upper image", upper_red_hue_range)
    #     cv2.namedWindow("Combined threshold images", cv2.WINDOW_AUTOSIZE)
    #     cv2.imshow("Combined threshold images", red_hue_image)
    #     cv2.namedWindow("Detected red circles on the input image", cv2.WINDOW_AUTOSIZE)
    #     cv2.imshow("Detected red circles on the input image", orig_image)

    # k = cv2.waitKey(5) & 0xFF
    # if k == 27:
    #     return (0, 0), 0
    # if radius > 3:
    #     return center, radius
    # else:
    #     return (0, 0), 0

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        destroy()
