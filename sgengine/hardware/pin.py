class Pin:
    def __init__(ID, PinMode, analog_or_dig):
        self.ID             #int representing the pin. Intrinsic definition of the pin
        self.pin_mode       #string
        self.analog_or_dig  #string (either analog or dig) #TODO find better system

    def __str__(self):
        return f"Pin ID: {ID}"

    #getters and setters
    def set_ID(self, new_ID):
        self.ID = new_ID

    def get_ID(self):
        return ID 
    
    def set_mode(self, new_Pin_Mode):
        self.pin_mode = new_Pin_Mode

    def get_mode(self):
        return pin_mode

    def set_analog_or_dig(self, is_analog_or_dig):
        self.analog_or_dig = is_analog_or_dig

    def get_analog_or_dig(self):
        return analog_or_dig

