import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        #constraints.append({'agent': 0, 'loc' : [(1,5)], 'timestep' : 4, 'positive' : False})
        #constraints.append({'agent': 1, 'loc' : [(1,2),(1,3)], 'timestep' : 1, 'positive' : False})
        #constraints.append({'agent': 0, 'loc' : [(1,5)], 'timestep' : 10, 'positive' : False})
        
        #constraints.append({'agent': 1, 'loc' : [(1,3)], 'timestep' : 2, 'positive' : False})
        #constraints.append({'agent': 1, 'loc' : [(1,2)], 'timestep' : 2, 'positive' : False})
        #constraints.append({'agent': 1, 'loc' : [(1,4)], 'timestep' : 2, 'positive' : False})
        
        map_size = 0
        
        for i in range(len(self.my_map)):
            map_size = map_size+len(self.my_map[i])
        upper_bound = map_size
        
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            #print(path)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################
            if len(path) > upper_bound+1:
                raise BaseException('No solutions')
            
            upper_bound = upper_bound + len(path)
            
            
            if i < self.num_of_agents-1:
                for Nagent in range(i,self.num_of_agents-1):
                    for timestep in range(len(path)):
                        constraints.append({'agent': Nagent+1, 'loc' : [path[timestep]], 'timestep' : timestep, 'positive' : False})
                        if timestep > 0:
                            constraints.append({'agent': Nagent+1, 'loc' : [path[timestep],path[timestep-1]], 'timestep' : timestep, 'positive' : False})
                    for bound in range (len(path),upper_bound):
                        for goal in range (0,i+1):
                            constraints.append({'agent': Nagent+1, 'loc' : [self.goals[goal]], 'timestep' : bound, 'positive' : False})

            #print(constraints)
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
