from machine import Pin
from time import sleep
from Group_Project import init_servo, translate

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


