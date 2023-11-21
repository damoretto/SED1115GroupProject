#This program is used to read data from potentiometers linked to a micropico and to communicate some x and y values
#obtained through this process to two servos that are part of an "arm" that has the ultimate goal of drawing something on paper
#Submitted int the context of the SED1115 class, fall 2023 semester

from picozero import Pot    #pip install picozero   (in terminal, will take some time)
from time import sleep_ms
from machine import Pin, PWM

#This function will get input from each potentiometer (called once for each)
def read_potentiometers(potentiometer_id):#Dane
    #()->float

    #set potentiometer pin -> documentation for all that -> https://projects.raspberrypi.org/en/projects/introduction-to-the-pico/11

    #read potentiometer

    #return potentiometer value (.value)


#This function will use both value obtained from the potentiometers (x and y values) and turn them into
#proportionally correct versions for the servos
def xy_to_servos(x_value, y_value):#Estelle
    #(float, float)->int, int

    #initialize the variables containing both servo values

    #use inverse kinematics to turn the x and y values into the proportional values for the servos

    #call float_to_servos() to turn the values into safe values for the servos

    #use sleep_ms to slow do the program so that python can keep up and also be proportional to servo frequency
    sleep_ms(20)

    return servo_shoulder, servo_elbow


#This function will be called in xy_to_servos to transfer the raw value into "legal" values for servos
def float_to_servos(servo_shoulder, servo_elbow):#Estelle
    #(float, float)->int, int

    #use the same concept as lab7 to turn into safe values but instead of having wrong values,
    #values that are too high or low are tuned up or down to always be either the max or min


#This function will individually send their value to each servo (called twice)
def send_to_servo(duty_cycle, PWM_id):#Nathan
    #(int,int)->None

    #set PWM Pin, frequency and duty cycle



#This function will verify if the button has changed states or not (pressed or not pressed) includes debouncing
def is_button_pressed(button):#Algo
    #()->bool

    #set input pin for button  (documentation https://docs.micropython.org/en/latest/library/machine.Pin.html)

    #identify if the button is being pressed and return True if it is, else return False


#This function will change the state of the pen (on/off the paper) can use the send_to_servo() function
def change_pen_state(PWM_id, button_state):#Algo
    #(bool)->bool

    #define duty cycle for the two states of the pen (on/off the paper)

    #conditional that will determine what happens to the state of the pen depending on its current state
    if button_state:

        #send the duty cycle to the servo to change its position

        #return opposite button state

    elif not button_state:

        #send the duty cycle to the servo to change its position

        #return opposite button state

    else:
        print("This should not happen, problem in change_pen_state() function")



'''main'''

#initialize hardware IDs (documentation: https://randomnerdtutorials.com/raspberry-pi-pico-w-pinout-gpios/)
#all at the same place so easier for user to control
x_potentiometer_id = 'A1'   #x value potentiometer
y_potentiometer_id = 'A0'   #y value potentiometer
servo_elbow_id = 'GPIO0'    #elbow servo (middle one)
servo_shoulder_id = 'GPIO1' #shoulder servo (furthest from the pen)
servo_wrist_id = 'GPIO2'    #wrist servo (closest to the pen)
button_id = 'GPIO22'        #pen state button

#this variable will contain the state of the button at all times (bool)
button_state = False

try:
    while True:
        x_value = read_potentiometers(x_potentiometer_id)
        y_value = read_potentiometers(y_potentiometer_id)
        servo_shoulder, servo_elbow = xy_to_servos(x_value, y_value)
        send_to_servo(servo_shoulder, servo_elbow_id)
        send_to_servo(servo_shoulder, servo_shoulder_id)
        if is_button_pressed(button_id):
            button_state = change_pen_state(servo_wrist_id, button_state)

except KeyboardInterrupt:
    print("The program was interrupted by the user")