import sys
sys.setrecursionlimit(10001)

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        # self.nodes = dict()
        self.parent = dict() #easier to relate
            

    def DFS(self,current_nodeId,goal_nodeId):
        succ = self.planning_env.GetSuccessors(current_nodeId)
        for idx in succ:
            #exit from function if we reach goal
            if idx == goal_nodeId:
                self.parent[goal_nodeId]=current_nodeId
                break
            #else perform recursion over successors
            elif idx not in self.parent:
                self.parent[idx]=current_nodeId
                self.DFS(idx,goal_nodeId)


    def Plan(self, start_config, goal_config):
        
        plan = []        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        #convert to node id to pass to helper function
        start_nodeId = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_nodeId = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        #helper function to perform recursion
        # print(start_nodeId)
        self.DFS(start_nodeId,goal_nodeId)

        idx = goal_nodeId
        #generating plan reverse
        plan.append(goal_config)
        #to stop backtracking later        
        self.parent[start_nodeId] = None

        while self.parent[idx] is not None:
            #convert to config space to append to plan
            current_config = self.planning_env.discrete_env.NodeIdToConfiguration(self.parent[idx])
            plan.append(current_config)
            idx = self.parent[idx]

        plan.append(start_config)
        plan.reverse()
        print(plan)
        return plan
