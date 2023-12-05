#This program is used to read data from potentiometers linked to a micropico and to communicate some x and y values
#obtained through this process to two servos that are part of an "arm" that has the ultimate goal of drawing something on paper
#Submitted int the context of the SED1115 class, fall 2023 semester

from servo_translator import translate
from time import sleep_ms, sleep
from machine import Pin, PWM, ADC
import math
from pen_control import debounce, toggle_pen_state
from PIL import Image
import numpy as np

DEBOUNCE_TIME = 50  # milliseconds for button debouncing

#This function will get input from each potentiometer (called once for each)
#potentiometer id is GPIO number
def read_potentiometers(potentiometer_id):#Dane
    #()->float
    #initialize the variable that will contain the value from the potentiometer
    potentiometer_value = 0
    #set potentiometer pin -> documentation for all that -> https://projects.raspberrypi.org/en/projects/introduction-to-the-pico/11
    #picozero is already imported as Pot
    #convert analog to digital AKA voltage to number using built in ADC converter
    adc = ADC(Pin(potentiometer_id))
    #read potentiometer (.value) as a 16 bit value and assign to variable
    potentiometer_value = adc.read_u16()
    #turn duty cycle into degrees
    #return potentiometer value variable
    return potentiometer_value

#This function will use both values obtained from the potentiometers (x and y values) and turn them into
#proportionally correct versions for the servos
def xy_to_servos(x_value, y_value):#Estelle
    #(float, float)->int, int

    #initialize the variables containing both servo values
    shoulder_angle = 0
    elbow_angle = 0
    print(x_value, y_value)
    #turn the duty cycle into a value
    # input 0 - 65535 (duty cycle is in in u16)
    # x 0 - 216 (width of the paper)
    # y 0 - 279 (height of the paper)
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
    #call translate from lab6 to turn the values into safe values for the servos
    servo_shoulder = translate(shoulder_angle)
    servo_elbow = translate(elbow_angle)
    #use sleep_ms to slow the program so that python can keep up and also be proportional to servo frequency
    sleep_ms(20)#1/50th of a second

    return servo_shoulder, servo_elbow

#This function will individually send their value to each servo (called twice)
def init_servo(PWM_id):#Nathan
    #(int,int)->None
    #set PWM Pin, frequency and duty cycle
    pin = Pin(PWM_id)
    pwm_object = PWM(pin)
    pwm_object.freq(50)
    return pwm_object
    
#This function will get the choice of the user on the provenance of the image
def get_choice():
    #()->int
    #initialize  the variable that will contain the user's choice of provenance of the image
    choice = -1
    #this is for the user to choose where their image comes from (url or file)
    while choice != 0 and choice != 1 and choice != 2:
        choice = int(input("Where is your image coming from? file (1), potentiometers (2): "))
        if choice != 0 and choice != 1 and choice != 2:
            print("wrong input. Make sure you choose between 1 and 2")
    return choice

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

# Plotting Function
def plot_image(coords, shoulder_servo, elbow_servo, wrist_servo):
    for x, y in coords:
        shoulder_angle, elbow_angle = xy_to_servos(x, y)
        shoulder_servo.duty_u16(translate(shoulder_angle))
        elbow_servo.duty_u16(translate(elbow_angle))
        wrist_servo.duty_u16(translate(90))  # Pen down
        sleep_ms(20)  # Adjust as needed
    wrist_servo.duty_u16(translate(0))  # Pen up

def debounce(button):
    """
    Debounces the button to ensure stable press detection.
    Returns the stable state of the button (pressed or not).
    """
    first_reading = button.value()
    sleep(DEBOUNCE_TIME / 1000.0)
    second_reading = button.value()
    if first_reading == second_reading:
        return second_reading
    else:
        return None

def toggle_pen_state(current_state, button):
    """
    Toggles the pen state based on the button press.
    """
    if button.value() == 1:  # Button pressed
        return not current_state
    return current_state

'''main'''

#initialize hardware IDs (documentation: https://randomnerdtutorials.com/raspberry-pi-pico-w-pinout-gpios/)
x_potentiometer_id = 'GP27'   #x value potentiometer
y_potentiometer_id = 'GP26'   #y value potentiometer
button_id = 'GPIO22'          #pen state button

#initialize all servos
shoulder_servo = init_servo('GPIO0')
elbow_servo = init_servo('GPIO1')
wrist_servo = init_servo('GPIO2')

#Get both segments lenghts from user
seg1len = 155
seg2len = 155

#this variable will contain the state of the button at all times (bool)
pen_state = False   #at first not touching the paper
try:
    if get_choice() == 2:
        try:
            while True:
                x_value = read_potentiometers(x_potentiometer_id)   #get the x_value from potentiometers
                y_value = read_potentiometers(y_potentiometer_id)   #get the y value from potentiometers
                #turn these values into angles for the servos with inverse kinematics and turn into a duty cycle
                servo_shoulder_duty, servo_elbow_duty = xy_to_servos(x_value, y_value) 
                elbow_servo.duty_u16(servo_elbow_duty)  #send the pwm to the elbow servo
                shoulder_servo.duty_u16(servo_shoulder_duty)    #send the pwm to the shoulder servo
                button_state = debounce(button_id)  #check the state of the button
                if button_state is not None:
                    pen_state = toggle_pen_state(pen_state, button_id)
                    if pen_state:
                        wrist_servo.duty_u16(translate(90))  # Adjust angle for pen down
                    else:
                        wrist_servo.duty_u16(translate(0))   # Adjust angle for pen up

        except KeyboardInterrupt:
            wrist_servo.duty_u16(translate(30)) #disable servo by raising wrist
            print("The program was interrupted by the user")
    elif get_choice == 1:
        image = load_and_process_image('file_path')
        edges_array = detect_edges(image)
        coordinates_list = convert_to_plotter_coords(edges_array)
        plot_image(coordinates_list, shoulder_servo, elbow_servo, wrist_servo)

finally:
    #deinit all servos
    shoulder_servo.deinit()
    elbow_servo.deinit()
    wrist_servo.deinit()