import numpy as np

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        s_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)

        succ = self.planning_env.GetSuccessors(s_id)[0]
        print('succ: {}'.format(succ))

        plan.append(goal_config)
   
        return plan
