#import RPi.GPIO as GPIO

class Pin:
    def __init__(pin_num, io_mode, is_dig):
        self.pin_num             #int representing the pin. Intrinsic definition of the pin (essentially marks its location)
        self.io_mode             #boolean for whether the pin is an input or output pin. True if input, false if output
        self.is_dig              #boolean for whether the pin is analog or digital
        
        #setup the pin
        if(io_mode):
            GPIO.setup(pin_num, GPIO.IN)
        else:
            GPIO.setup(pin_num, GPIO.OUT)

    def __str__(self):
        return f"Pin number: {pin_num}"

    #getters and setters
    def set_pin_num(self, new_num):
        self.pin_num = new_num

    def get_pin_num(self):
        return pin_num 
    
    def set_input(self, is_input):
        self.io_mode = is_input
        #reset the GPIO pin
        GPIO.setup(self.pin_num, is_input)

    def get_input(self):
        return is_input

    def set_is_dig(self, set_to_dig):
        self.is_dig = set_to_dig

    def get_is_dig(self):
        return is_dig

