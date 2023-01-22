class Pin_handler:
    def __init__():
        self.pin_dict = {}
        self.pin_groups = {}

    def add_dict_entry(self, pin_name, pin):
        pin_dict[pin_name] = pin

    def remove_dict_entry(self, pin_name):
        pin_dict.pop(pin_name)

    def switch_pins(self, pin_name1, pin_name2):
        temp_pin = pin_dict[pin_name1] #store pin at pin_name1 so it is not lost
        pin_dict[pin_name1] = pin_dict[pin_name2] #replace pin at pin_name1 with the pin at pin_name2
        pin_dict[pin_name2] = temp_pin #replace pin at pin_name2 with the pin that was at pin_name2

    def rename_pin(self, old_pin_name, new_pin_name):
        #create new entry with new name and old data
        pin_dict[new_pin_name] = pin_dict[old_pin_name]
        #remove old pin
        pin_dict.pop(old_pin_name)