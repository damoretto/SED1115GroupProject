#This program is used to read data from potentiometers linked to a micropico and to communicate some x and y values
#obtained through this process to two servos that are part of an "arm" that has the ultimate goal of drawing something on paper
#Submitted int the context of the SED1115 class, fall 2023 semester

import os
#import sys
from servo_translator import translate
from time import sleep_ms
from machine import Pin, PWM, ADC
#import requests
#from io import BytesIO
#import numpy as np
import math

#set the file directory to avoid issues
#script_directory = os.path.dirname(os.path.abspath(__file__))
#os.chdir(script_directory)

#This function will get input from each potentiometer (called once for each)
def read_potentiometers(potentiometer_id):#Dane
    #()->float
    #initialize the variable that will contain the value from the potentiometer
    potentiometer_value = 0
    #set potentiometer pin -> documentation for all that -> https://projects.raspberrypi.org/en/projects/introduction-to-the-pico/11
    #picozero is already imported as Pot
    adc = ADC(Pin(potentiometer_id))
    #read potentiometer (.value) and assign to variable
    potentiometer_value = adc.read_u16()
    #turn duty cycle into degrees
    #return potentiometer value variable
    return potentiometer_value

#This function will use both value obtained from the potentiometers (x and y values) and turn them into
#proportionally correct versions for the servos
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

#This function will individually send their value to each servo (called twice)
def init_servo(PWM_id):#Nathan
    #(int,int)->None

    #set PWM Pin, frequency and duty cycle
    pin = Pin(PWM_id)
    pwm_object = PWM(pin)
    pwm_object.freq(50)
    return pwm_object

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
def change_pen_state(wrist_servo, pen_state):#Algo
    #(bool)->bool

    #define duty cycle for the two states of the pen (on/off the paper)

    #conditional that will determine what happens to the state of the pen depending on its current state
    if pen_state:    #if the pen is currently touching the paper

        #send the duty cycle to the servo to change its position

        #return opposite button state
        return False

    elif not pen_state: #if the pen is currently not touching the paper

        #send the duty cycle to the servo to change its position

        #return opposite button state
        return True

    else:
        print("This should not happen, problem in change_pen_state() function")
    
#This function will get the choice of the user on the provenance of the image
def get_choice():
    #()->int
    #initialize  the variable that will contain the user's choice of provenance of the image
    choice = -1
    #this is for the user to choose where their image comes from (url or file)
    while choice != 0 and choice != 1 and choice != 2:
        choice = int(input("Where is your image coming from? URL (0), file (1), potentiometers (2)"))
        if choice != 0 and choice != 1 and choice != 2:
            print("wrong input. make sure you choose between 0, 1 and 2")
    return choice
'''
#This function will get the image in function of where it comes from
def get_image_url():#Nathan
    #()->Image
    #initialize the variable containing the Image object that will be returned
    image = None
    #flag that will make sure that a good link is used
    image_found = False
    while not image_found:
        #get the url
        url = input("Please paste the URL or the image you desire to print here:\n")
        response = requests.get(url)
        #staus code 200 means that the image was successfully fetched
        if response.status_code == 200:
            #open image in grayscale
            image = Image.open(BytesIO(response.content)).convert('L')
            image_found = True
        else:
            #try images from https://unsplash.com/
            #try 'https://images.unsplash.com/photo-1700212966732-4e0ebd08ad42?q=80&w=3792&auto=format&fit=crop&ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D'
            print("failed to get the image. Make sure the image is from a good source")
    return image

#This function will get the image object from  file 
def get_image_file():
    #()->Image
    #initialize the variable containing the Image object that will be returned
    image = None
    #flag fo the whil loop that makes sure that the image file exists
    img_not_found = True
    while img_not_found:
        file_name = input("please enter the full name of the image file including the file extension (ex. .png .jpg ...):\n")
        try:
            #open image in grayscale
            image = Image.open(file_name).convert('L')
            img_not_found = False
            #This happens if the image is not in the right directory or does not exists or if the name is spelled wrong
        except OSError:
            print("The image cannot be fond, make sure it is present in the same directory as this program and that you wrote the right name")
            ex = input("press a then enter if you wish to leave the program\n")
            if ex == 'a':
                #this is to allow the user to eventually quit the program and go check in their files where the issue may come from
                sys.exit()
        except:
            print("unkown error happened?")
    else:
        print("This should not happen, issue with the choice input")

    #you can return an opened file object
    return image   

#this function will transfer the greyscale image object into an array'''



'''main'''

#initialize hardware IDs (documentation: https://randomnerdtutorials.com/raspberry-pi-pico-w-pinout-gpios/)
x_potentiometer_id = 'GP27'   #x value potentiometer
y_potentiometer_id = 'GP26'   #y value potentiometer
button_id = 'GPIO22'        #pen state button

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
                x_value = read_potentiometers(x_potentiometer_id)
                y_value = read_potentiometers(y_potentiometer_id)
                servo_shoulder_duty, servo_elbow_duty = xy_to_servos(x_value, y_value)
                elbow_servo.duty_u16(servo_elbow_duty)
                shoulder_servo.duty_u16(servo_shoulder_duty)
                #if is_button_pressed(button_id):
                   # pen_state = change_pen_state(wrist_servo, pen_state)
        except KeyboardInterrupt:
            wrist_servo.duty_u16(translate(30)) #disable servo by raising wrist
            print("The program was interrupted by the user")
    '''elif get_choice == 1:
        image = get_image_url()

    elif get_choice == 0:
        image = get_image_file()'''
finally:
    #deinit all servos
    shoulder_servo.deinit()
    elbow_servo.deinit()
    wrist_servo.deinit()