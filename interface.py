from enum import Enum

### Enum variable for user input ###
class UserInput(Enum):
    NEW = 0
    BEST = 1
    RE = 2
    QUIT = 3

def get_user_input_init():
    print("\n[ Select the initial racing line ]")
    while True:
        print("new  : use previous optimization result")
        print("best : use best optimization result so far")
        print("re   : re-optimize the initial racing line")
        print("quit : quit the program")
        user_input = input("\nEnter your choice: ").strip().lower()
        if user_input == "" or user_input == "new":
            return UserInput.NEW
        elif user_input == "best":
            return UserInput.BEST
        elif user_input == "re":
            return UserInput.RE
        elif user_input == "quit":
            return UserInput.QUIT
        else:
            print("\n\nWarning: Invalid input. Note the following input format.")