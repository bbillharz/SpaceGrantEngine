'''The pin handler class is what it sounds like: it handles and organizes the pins in the robot.
   All pin accessing should be done through the pin_handler class.
   Each pin has a simple string name that can be defined by the user, so it's easier to keep track of each pin'''

class Pin_handler:
    def __init__(pin_dictionary):
        self.pin_dict = pin_dictionary  #maps the name of a pin to the pin data itself {string:Pin}
        self.pin_groups = {}            #maps the name of a pin group to a list of pin names in that group {string:string[]}

    #Add a pin to the pin dictionary, takes the pin_name (string), and the pin (Pin)
    def add_dict_entry(self, pin_name, pin):
        pin_dict[pin_name] = pin

    #Removes an entry of the pin dictionary, takes the pin_name (string)
    def remove_dict_entry(self, pin_name):
        pin_dict.pop(pin_name)

    #Switches the names of two pins already in the dictionary (so each pin still has the same data, but which is accessed is different)
    #takes pin_name1 (string) and pin_name2 (string)
    def switch_pins(self, pin_name1, pin_name2):
        temp_pin = pin_dict[pin_name1]              #store pin at pin_name1 so it is not lost
        pin_dict[pin_name1] = pin_dict[pin_name2]   #replace pin at pin_name1 with the pin at pin_name2
        pin_dict[pin_name2] = temp_pin              #replace pin at pin_name2 with the pin that was at pin_name2

    #Renames a pin already in the dictionary
    def rename_pin(self, old_pin_name, new_pin_name):
        pin_dict[new_pin_name] = pin_dict[old_pin_name]     #create new entry with new name and old data
        pin_dict.pop(old_pin_name)                          #remove old pin

    #Get pin info
    def get_pin(self, pin_name):
        return pin_dict[pin_name]

    def get_pin_ID(self, pin_name):
        return pin_dict[pin_name].get_ID()

    def get_pin_Mode(self, pin_name):
        return pin_dict[pin_name].get_mode()

    def get_analog_or_dig(self, pin_name):
        return pin_dict[pin_name].get_analog_or_dig

    