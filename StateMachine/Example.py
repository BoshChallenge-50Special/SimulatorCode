from safetySM import StateMachineSM

#Main script
if __name__ == "__main__":
    try:
        m = StateMachineSM()

        while(True):
            values={"a":1,"b":42,"c":24,"d":2}
            state=m.runOneStep(values)

            if(state=="StateA"):

                print("StateA")
            elif(state=="StateB"):
                print("StateB")
            elif(state=="StateC"):
                print("StateC")

    # In case of a sudden interrupt, print the cause
    except KeyboardInterrupt as k:
        print("Keyboard: " + str(k))
    except Exception as e:
        print("Exception: " + str(e))
    except SystemExit as s:
        print("Exit: " + str(s))
