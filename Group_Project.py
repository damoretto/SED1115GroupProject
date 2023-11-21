#This program is used to read data from potentiometers linked to a micropico and to communicate some x and y values
#obtained through this process to two servos that are part of an "arm" that has the ultimate goal of drawing something on paper
#Submitted int the context of the SED1115 class, fall 2023 semester

#This function will get input from each potentiometer (called once for each)
def read_potentiometers():#Dane
    #()->float


#This function will use both value obtained from the potentiometers (x and y values) and turn them into
#proportionally correct versions for the servos
def xy_to_servos(x_value, y_value):#Estelle
    #(float, float)->int, int


#This function will be called in xy_to_servos to transfer the raw value into "legal" values for servos
def float_to_servos(servo_shoulder, servo_elbow):#Estelle
    #(float, float)->int, int


#This function will individually send their value to each servo (called twice)
def send_to_servo(servo_value, channel_id):#Nathan
    #(int,int)->None

#This function will verify if the button has changed states or not (pressed or not pressed) includes debouncing
def button_change_state():#Algo
    #()->bool


#This function will change the state of the pen (on/off the paper)
def pen_state():#Algo
    #(bool)->None



'''main'''
try:
    while True:
        x_value = read_potentiometers()
        y_value = read_potentiometers()
        servo_shoulder, servo_elbow = xy_to_servos(x_value, y_value)
        send_to_servo(servo_shoulder, servo_elbow)
        if button_change_state():
            pen_state()

except KeyboardInterrupt:
    print("The program was interrupted by the user")