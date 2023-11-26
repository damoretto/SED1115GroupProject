#This code contains a function that will translate a pulse width input for a servo
#(50Hz)

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