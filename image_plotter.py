from PIL import Image
import numpy as np
import math
from machine import ADC, PWM, Pin
from time import sleep_ms
import cv2

# Load the image
def load_image(file_path):
    # Assuming the image is in the current directory, otherwise provide the full path
    image = Image.open(file_path)
    image = image.convert('L')  # Convert to grayscale
    image = image.point(lambda x: 0 if x < 128 else 255, '1')  # Threshold to binary
    return image

# Detect lines using Hough Transform
def detect_lines(image):
    # Convert the image to a format suitable for line detection
    image_for_detection = np.array(image)
    edges = cv2.Canny(image_for_detection, 50, 150, apertureSize=3)

    # Use Hough Transform to detect lines
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

    return lines

# Convert detected lines to brachiograph coordinates
def lines_to_brachiograph_coords(lines):
    brachiograph_lines = []
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        # Convert line endpoints to brachiograph coordinates
        brachiograph_lines.append(((x1, y1), (x2, y2)))

    return brachiograph_lines

# Plot the lines with the brachiograph
def plot_lines(brachiograph_lines, shoulder_servo, elbow_servo, wrist_servo):
    for line in brachiograph_lines:
        start_point, end_point = line

        # Move to the start point
        shoulder_angle, elbow_angle = xy_to_servos(*start_point)
        shoulder_servo.duty_u16(translate(shoulder_angle))
        elbow_servo.duty_u16(translate(elbow_angle))

        # Lower the pen
        wrist_servo.duty_u16(translate(90))  # Pen down

        # Draw the line to the end point
        shoulder_angle, elbow_angle = xy_to_servos(*end_point)
        shoulder_servo.duty_u16(translate(shoulder_angle))
        elbow_servo.duty_u16(translate(elbow_angle))

        # Raise the pen
        wrist_servo.duty_u16(translate(0))  # Pen up
        sleep_ms(20)  # Adjust as needed

# Detect lines using Hough Transform
def detect_lines(image):
    # Convert the image to a format suitable for line detection
    image_for_detection = np.array(image)
    edges = cv2.Canny(image_for_detection, 50, 150, apertureSize=3)

    # Use Hough Transform to detect lines
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

    return lines

# Convert detected lines to brachiograph coordinates
def lines_to_brachiograph_coords(lines):
    brachiograph_lines = []
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        # Convert line endpoints to brachiograph coordinates
        brachiograph_lines.append(((x1, y1), (x2, y2)))

    return brachiograph_lines

# Plot the lines with the brachiograph
def plot_lines(brachiograph_lines, shoulder_servo, elbow_servo, wrist_servo):
    for line in brachiograph_lines:
        start_point, end_point = line

        # Move to the start point
        shoulder_angle, elbow_angle = xy_to_servos(*start_point)
        shoulder_servo.duty_u16(translate(shoulder_angle))
        elbow_servo.duty_u16(translate(elbow_angle))

        # Lower the pen
        wrist_servo.duty_u16(translate(90))  # Pen down

        # Draw the line to the end point
        shoulder_angle, elbow_angle = xy_to_servos(*end_point)
        shoulder_servo.duty_u16(translate(shoulder_angle))
        elbow_servo.duty_u16(translate(elbow_angle))

        # Raise the pen
        wrist_servo.duty_u16(translate(0))  # Pen up
        sleep_ms(20)  # Adjust as needed

# Main function to orchestrate the plotting
def main():
    # Initialize servos 
    shoulder_servo = init_servo('GPIO0')
    elbow_servo = init_servo('GPIO1')
    wrist_servo = init_servo('GPIO2')

    # Load the image and detect lines
    processed_image = load_image('clu-soh-oga9Xg0KVnU-unsplash.jpg')
    lines = detect_lines(processed_image)
    brachiograph_lines = lines_to_brachiograph_coords(lines)

    # Plot the lines
    plot_lines(brachiograph_lines, shoulder_servo, elbow_servo, wrist_servo)

    # Clean up
    shoulder_servo.deinit()
    elbow_servo.deinit()
    wrist_servo.deinit()

if __name__ == "__main__":
    main()
