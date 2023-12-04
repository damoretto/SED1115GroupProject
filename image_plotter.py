from PIL import Image
import numpy as np
import math
from machine import ADC, PWM, Pin
from time import sleep_ms

# Image Processing and Edge Detection Functions
def load_and_process_image(file_path):
    image = Image.open(file_path)
    image = image.convert('L')  # Convert to grayscale
    image = image.resize((216, 279))  # Resize to match plotter area
    image = image.point(lambda x: 0 if x < 128 else 255, '1')  # Convert to binary
    return image

def detect_edges(image):
    image_array = np.array(image)
    edges = np.zeros_like(image_array)

    for y in range(1, image_array.shape[0] - 1):
        for x in range(1, image_array.shape[1] - 1):
            if image_array[y, x] != image_array[y, x+1] or image_array[y, x] != image_array[y+1, x]:
                edges[y, x] = 255
    return edges

def convert_to_plotter_coords(edges):
    coords = []
    for y in range(edges.shape[0]):
        for x in range(edges.shape[1]):
            if edges[y, x] == 255:
                plotter_x, plotter_y = convert_pixel_to_plotter(x, y)
                coords.append((plotter_x, plotter_y))
    return coords

def convert_pixel_to_plotter(x, y):
    # Convert pixel coordinates to plotter coordinates
    plotter_x = x  # Modify according to the plotter's coordinate system and scale
    plotter_y = y  # Modify according to the plotter's coordinate system and scale
    return plotter_x, plotter_y

# Brachiograph Control Functions 
def xy_to_servos(x_value, y_value):#Estelle
    #(float, float)->int, int

    #initialize the variables containing both servo values
    shoulder_angle = 0
    elbow_angle = 0
    print(x_value, y_value)
    #turn the duty cycle into a value /31
    # input 0 - 65535
    # x 0 - 216
    # y 0 - 279
    x_value = (x_value/65535)*216
    y_value = (y_value/65535)*279
    print(x_value, y_value)
    #Calculate values of smaller components of the big formulas
    AbaseC = math.sqrt((-50-x_value)**2 + y_value**2)
    lAC = math.sqrt((-50-x_value)**2+((139.5-y_value)**2))
    aBAC = math.acos((seg1len**2 + lAC**2 - seg2len**2)/(2*seg1len*lAC))
    aYAC = math.acos((139.5**2 + lAC**2 - AbaseC**2)/(2*139.5*lAC))
    aACB = math.asin((seg1len * math.sin(aBAC))/seg2len)
    #use inverse kinematics to turn the x and y values into proportional angles between 0 and 180 degrees
    elbow_angle = aBAC + aACB
    shoulder_angle =  aBAC + aYAC
    elbow_angle = 150 - math.degrees(elbow_angle)
    shoulder_angle = math.degrees(shoulder_angle) - 75
    #maybe separate the conversion to degrees from the inverse kinematics to make it easier to read
    #call translate from lab6 to turn the values into safe values for the servos
    servo_shoulder = translate(shoulder_angle)
    servo_elbow = translate(elbow_angle)
    #use sleep_ms to slow the program so that python can keep up and also be proportional to servo frequency
    sleep_ms(20)#1/50th of a second

    return servo_shoulder, servo_elbow

def init_servo(PWM_id):#Nathan
    #(int,int)->None

    #set PWM Pin, frequency and duty cycle
    pin = Pin(PWM_id)
    pwm_object = PWM(pin)
    pwm_object.freq(50)
    return pwm_object

def translate(angle):
    #(int)->int
    if not isinstance(angle, int) and not isinstance(angle, float):
        print("The angle should be an int or float value")
    #error handling
    if angle > 180:
        print("angle lowered to 180 because higher angles are not supported")
        angle = 180
    elif angle < 0:
        print("angle highered to 0 because negative angles are not supported")
        angle = 0

    #turn the angle into a duty cycle that can be taken by the duty_u16 method
    duty_cycle = int(((500+(2000)*angle/180)/20000)*65535)

    if duty_cycle > 8191:
        duty_cycle = 8191
    elif duty_cycle < 1639:
        duty_cycle = 1639
    else:
        pass

    return duty_cycle

# Plotting Function
def plot_image(coords, shoulder_servo, elbow_servo, wrist_servo):
    for x, y in coords:
        shoulder_angle, elbow_angle = xy_to_servos(x, y)
        shoulder_servo.duty_u16(translate(shoulder_angle))
        elbow_servo.duty_u16(translate(elbow_angle))
        wrist_servo.duty_u16(translate(90))  # Pen down
        sleep_ms(20)  # Adjust as needed
    wrist_servo.duty_u16(translate(0))  # Pen up

# Main Script
def main():
    # Initialize servos
    shoulder_servo = init_servo('GPIO0')
    elbow_servo = init_servo('GPIO1')
    wrist_servo = init_servo('GPIO2')

    # Load and process the image
    image_path = "path_to_your_image.jpg"  # Replace with the image path
    processed_image = load_and_process_image(image_path)
    edges = detect_edges(processed_image)
    coords = convert_to_plotter_coords(edges)

    # Plot the image
    plot_image(coords, shoulder_servo, elbow_servo, wrist_servo)

    # Clean up
    shoulder_servo.deinit()
    elbow_servo.deinit()
    wrist_servo.deinit()

if __name__ == "__main__":
    main()
