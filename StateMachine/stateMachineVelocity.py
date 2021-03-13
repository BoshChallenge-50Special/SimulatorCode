from statemachine import StateMachine

class StateMachineVelocity:
    '''
    Implementation of the specific StateMachine following the need of our project
    '''
    memory=[]
    m={}

    def __init__(self):
        self.m = StateMachine()
        #Declaration of all the states
        self.m.add_state("OnSteady", self.OnSteady_transitions)
        self.m.add_state("Slow", self.Slow_transitions)
        self.m.add_state("Fast", self.Fast_transitions)
        self.m.set_start("OnSteady")

    #method for evaluating the OR of a list of variables
    @staticmethod
    def conditionOr(op,data, names, values):
        result=False
        for i,key in enumerate(names):
            if(op==">"):
                result=result or data[key]>values[i]
            elif(op == "=="):
                result=result or data[key]==values[i]
            else:
                result=result or data[key]<values[i]
        return result

    #method for evaluating the AND of a list of variables
    @staticmethod
    def conditionAnd(op,data, names, values):
        result=True
        for i,key in enumerate(names):
            if(op==">"):
                result=result and data[key]>values[i]
            elif(op == "=="):
                result=result and data[key]==values[i]
            else:
                result=result and data[key]<values[i]
        return result

    #Declaration of the transition from the StateA to the other possible states
    def OnSteady_transitions(self, data):
        if self.conditionAnd("==", data, ["moving"], [True]):
            newState = "Fast"
        else:
            newState = "OnSteady"
        return newState

    #Declaration of the transition from the StateB to the other possible states
    def Slow_transitions(self, data):
        if self.conditionOr("==", data, ["stop_signal"], [True]):
            newState = "OnSteady"
        elif self.conditionAnd("==", data, ["horizontal_line", "state_steer"], ["Safe", "OnLane"]):
            newState = "Fast"
        else:
            newState = "Slow"
        return newState

    #Declaration of the transition from the StateC to the other possible states
    def Fast_transitions(self, data):
        if self.conditionOr("==", data, ["horizontal_line", "horizontal_line"], ["Stop", "Pedestrians"]):
            newState = "Slow"
        else:
            newState = "Fast"
        return newState

    def runOneStep(self,data):
        return self.m.runOneStep(data)

#Code for testing the state machine
#
#if __name__== "__main__":
#    m = StateMachineSM()
#
#    m.runOneStep({"a" :0,"b" :0,"c":0, "d":0 })
#    m.runOneStep({"a" :1,"v" :1,"c":1, "d":1 })
