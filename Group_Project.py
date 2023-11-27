#This program is used to read data from potentiometers linked to a micropico and to communicate some x and y values
#obtained through this process to two servos that are part of an "arm" that has the ultimate goal of drawing something on paper
#Submitted int the context of the SED1115 class, fall 2023 semester

import os
import sys
from servo_translator import translate
from picozero import Pot    #pip install picozero   (in terminal, will take some time)
from time import sleep_ms
from machine import Pin, PWM
import requests
from io import BytesIO
from PIL import Image
import numpy as np

#set the file directory to avoid issues
script_directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_directory)

#This function will get input from each potentiometer (called once for each)
def read_potentiometers(potentiometer_id):#Dane
    #()->float
    
    #initialize the variable that will contain the value from the potentiometer
    potentiometer_value = 0
    #set potentiometer pin -> documentation for all that -> https://projects.raspberrypi.org/en/projects/introduction-to-the-pico/11
    #picozero is already imported as Pot

    #read potentiometer (.value) and assign to variable

    #return potentiometr value variable
    return potentiometer_value

    #sorry bro omg i did this but didnt put it in github FCK GITHUB


#This function will use both value obtained from the potentiometers (x and y values) and turn them into
#proportionally correct versions for the servos
def xy_to_servos(x_value, y_value):#Estelle
    #(float, float)->int, int

    #initialize the variables containing both servo values
    shoulder_angle = 0
    elbow_angle = 0
    #use inverse kinematics to turn the x and y values into proportional angles between 0 and 180 degrees

    #call translate from lab6 to turn the values into safe values for the servos
    servo_shoulder = translate(shoulder_angle)
    servo_elbow = translate(elbow_angle)
    #use sleep_ms to slow do the program so that python can keep up and also be proportional to servo frequency
    sleep_ms(20)

    return servo_shoulder, servo_elbow


#This function will individually send their value to each servo (called twice)
def send_to_servo(duty_cycle, PWM_id):#Nathan
    #(int,int)->None

    #set PWM Pin, frequency and duty cycle
    pin = Pin(PWM_id, Pin.IN)
    pwm_object = PWM(pin)
    pwm_object.freq(50)
    pwm_object.duty_u16(duty_cycle)
    pwm_object.deinit()


#This function will verify if the button has changed states or not (pressed or not pressed) includes debouncing
def is_button_pressed(button_id):#Nathan
    #()->bool

    #set input pin for button  (documentation https://docs.micropython.org/en/latest/library/machine.Pin.html)
    button = Pin(button_id, Pin.IN)
    #identify if the button is being pressed and return True if it is, else return False
    if button.value():
        return True
    elif not button.value():
        return False
    else:
        print("There seems to be an issue with the button")

#This function will change the state of the pen (on/off the paper) can use the send_to_servo() function
def change_pen_state(PWM_id, button_state):#Algo
    #(bool)->bool

    #define duty cycle for the two states of the pen (on/off the paper)

    #conditional that will determine what happens to the state of the pen depending on its current state
    if button_state:

        #send the duty cycle to the servo to change its position

        #return opposite button state
        return False

    elif not button_state:

        #send the duty cycle to the servo to change its position

        #return opposite button state
        return True

    else:
        print("This should not happen, problem in change_pen_state() function")
    

#This function will get the image in function of where it comes from
def get_image():#Nathan
    #()->Image
    #initialize the variable containing the Image object that will be returned
    image = None
    #initialize  the variable that will contain the user's choice of provenance of the image
    choice = -1
    #this is for the user to choose where their image comes from (url or file)
    while choice != 0 and choice != 1:
        choice = int(input("Where is your image coming from? URL (0), file (1)"))
        if choice != 0 and choice != 1:
            print("wrong input. make sure you choose between 0 and 1")
    #if the image is from an url
    if choice == 0:
        url = input("Please paste the URL or the image you desire to print here:\n")
        response = requests.get(url)
        #staus code 200 means that the image was successfully fetched
        if response.status_code == 200:
            image = Image.open(BytesIO(response.content)).convert('L')
        else:
            #try images from https://unsplash.com/
            print("failed to get the image. Make sure the image is from a good source")
    #if the image is from a file
    elif choice == 1:
        #flag fo the whil loop that makes sure that the image file exists
        img_not_found = True
        while img_not_found:
            file_name = input("please enter the full name of the image file including the file extension (ex. .png .jpg ...):\n")
            try:
                image = Image.open(file_name).convert('L')
                img_not_found = False
            #This happens if the image is not in the right directory or does not exists or if the name is spelled wrong
            except OSError:
                print("The image cannot be fond, make sure it is present in the same directory as this program and that you wrote the right name")
                ex = input("press a and enter if you wish to leave the program\n")
                if ex == 'a':
                    #this is to allow the user to eventually quit the program and go check in their files where the issue may come from
                    sys.exit()
            except:
                print("unkown error happened?")
    else:
        print("This should not happen, issue with the choice input")
    
    return image
    

#this function will
def


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

#disable servo Nathan