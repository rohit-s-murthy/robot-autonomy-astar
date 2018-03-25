import numpy as np
from collections import deque

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        parent = dict()
        to_visit = deque()

        s_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        g_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        parent[s_id] = None
        succ = self.planning_env.GetSuccessors(s_id)
        

        while to_visit:
            s_id = to_visit.popleft()
            succ = self.planning_env.GetSuccessors(s_id)
            for s in succ:
                if s not in parent:
                    to_visit.append(s)
                    parent[s] = s_id
            if g_id in parent:
                break

        plan = [goal_config]
        while parent[plan[-1]] is not None:
            config_to_add = self.planning_env.NodeIdToConfiguration(parent[plan[-1]])
            plan.append(config_to_add)
        plan.reverse()   
        return plan
