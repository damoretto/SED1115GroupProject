from servo_translator import translate
import math
from time import sleep_ms
from machine import ADC, PWM, Pin

def xy_to_servos(x_value, y_value):#Estelle
    #(float, float)->int, int

    #initialize the variables containing both servo values
    shoulder_angle = 0
    elbow_angle = 0
    
    #use inverse kinematics to turn the x and y values into proportional angles between 0 and 180 degrees
    elbow_angle = math.degrees(math.acos((x_value**2 + y_value**2 - seg1len**2 - seg2len**2)/(2*seg1len*seg2len))) #from https://www.youtube.com/watch?v=kAdbxsJZGto, law of cosines
    shoulder_angle = math.degrees((math.atan2(y_value, x_value)) - (math.atan2((seg2len*math.sin(elbow_angle)), (seg1len + seg2len*math.cos(elbow_angle))))) #from https://www.youtube.com/watch?v=kAdbxsJZGto, law of sines
    #maybe separate the conversion to degrees from the inverse kinematics to make it easier to read
    #call translate from lab6 to turn the values into safe values for the servos
    servo_shoulder = translate(shoulder_angle)
    servo_elbow = translate(elbow_angle)
    #use sleep_ms to slow do the program so that python can keep up and also be proportional to servo frequency
    sleep_ms(20)

    return servo_shoulder, servo_elbow

seg1len = 15.5
seg2len = 15.5

servo_shoulder_u16, servo_elbow_u16 = xy_to_servos(50, 58)