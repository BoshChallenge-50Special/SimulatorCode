from statemachine import StateMachine

class StateMachineSteer:
    '''
    Implementation of the specific StateMachine following the need of our project
    '''
    memory=[]
    m={}

    def __init__(self):
        self.m = StateMachine()
        #Declaration of all the states
        self.m.add_state("Steady", self.Steady_transitions)
        self.m.add_state("OnLane", self.OnLane_transitions)
        self.m.add_state("NearCrossroad", self.NearCrossroad_transitions)
        self.m.add_state("OnCrossroad", self.OnCrossroad_transitions)
        self.m.set_start("Steady")

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

    #Declaration of the transition from the Steady_transitions to the other possible states
    def Steady_transitions(self, data):
        if self.conditionAnd("==", data, ["moving"], [True]):
            newState = "OnLane"
        else:
            newState = "Steady"
        return newState

    #Declaration of the transition from the StateB to the other possible states
    def OnLane_transitions(self, data):
        if self.conditionAnd("==", data, ["horizontal_line"], ["Stop"]):
            newState = "NearCrossroad"
        elif self.conditionAnd("==", data, ["moving"], [False]):
            newState = "Steady"
        else:
            newState = "OnLane"
        return newState

    #Declaration of the transition from the StateC to the other possible states
    def NearCrossroad_transitions(self, data):
        if self.conditionAnd("==", data, ["horizontal_line"], ["Safe"]):
            newState = "OnCrossroad"
        elif self.conditionAnd("==", data, ["horizontal_line"], ["Pedestrians"]):
            newState = "OnLane"
        else:
            newState = "NearCrossroad"
        return newState

    #Declaration of the transition from the StateC to the other possible states
    def OnCrossroad_transitions(self, data):
        if self.conditionAnd("==", data, ["turning"], [False]):
            newState = "OnLane"
        else:
            newState = "OnCrossroad"
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
