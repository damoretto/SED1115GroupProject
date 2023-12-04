from machine import Pin
from time import sleep

# Constants
DEBOUNCE_TIME = 50  # milliseconds for button debouncing

# Initialize the button Pin for the pen control
button_pin = Pin('GPIO22', Pin.IN, Pin.PULL_DOWN)  # Adjust the pin number as per your setup

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

# Main Loop
try:
    pen_state = False  # Initial state of the pen (up)
    wrist_servo = init_servo('GPIO2')  # Initialize the servo for pen control

    while True:
        button_state = debounce(button_pin)
        if button_state is not None:
            pen_state = toggle_pen_state(pen_state, button_pin)
            if pen_state:
                wrist_servo.duty_u16(translate(90))  # Adjust angle for pen down
            else:
                wrist_servo.duty_u16(translate(0))   # Adjust angle for pen up


except KeyboardInterrupt:
    print("Program interrupted by the user")
    wrist_servo.deinit()  # Deinitialize the servo on program termination


