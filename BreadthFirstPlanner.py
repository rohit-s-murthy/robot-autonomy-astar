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
        to_visit.append(s_id)
        
        print "starting planning"
        print g_id
        while to_visit:
            s_id = to_visit.popleft()
            print "visiting", s_id
            succ = self.planning_env.GetSuccessors(s_id)
            print "successors", succ
            for s in succ:
                if s not in parent:
                    to_visit.append(s)
                    parent[s] = s_id
                    print s_id, s
            if g_id in parent:
                break

        plan_ids = [g_id]
        plan = [goal_config]
        while parent[plan_ids[-1]] is not None:
            config_to_add = self.planning_env.NodeIdToConfiguration(parent[plan_ids[-1]])
            plan.append(config_to_add)
            plan_ids.append(parent[plan_ids[-1]])
        plan.reverse()   

        return plan
