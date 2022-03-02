import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    if len(path1)>len(path2):
        length = len(path1)
    else:
        length = len(path2)

    for i in range(length):
        if get_location(path1,i) == get_location(path2,i):
            return [i, [get_location(path1,i)]]
        elif get_location(path1,i) == get_location(path2,i+1) and get_location(path1,i+1) == get_location(path2,i):
            return [i+1, [get_location(path2,i), get_location(path1,i)]]
        
    return None
            
        


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []
    for i in range(len(paths)):
        for j in range(len(paths)):
            if j > i:
                timestep_loc = detect_collision(paths[i],paths[j])
                if timestep_loc != None:
                    collisions.append({'a1': i, 'a2': j, 'loc': timestep_loc[1], 'timestep': timestep_loc[0]})
    return collisions
            


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    constraints = []
    if len(collision['loc']) == 2:
        constraints.append({'agent': collision['a1'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive':False})
        constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive':False})
    else:
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive':False})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive':False})
        
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    constraints = []
    rand = random.randint(0,1)
    if len(collision['loc']) == 2:
        if rand == 0:
            constraints.append({'agent': collision['a1'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive':True})
            constraints.append({'agent': collision['a1'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive':False})
        else:
            constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive':True})
            constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive':False})    
    else:
        if rand == 0:
            constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive':True})
            constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive':False})
        else:
            constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive':True})
            constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive':False})
        
    return constraints

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        
        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        upper_bound = 0
        
        
        map_size = 0
        #print(self.my_map[0])
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[i])):
                if self.my_map[i][j] == False:
                    map_size = map_size+1
        upper_bound = map_size
        upper_bound = upper_bound * self.num_of_agents
        
        '''
        for i in range(self.num_of_agents):
            upper_bound = upper_bound + self.heuristics[i][self.starts[i]]
        print(upper_bound)
        '''
        #use upper bound or heuristic(shortest path from goal(0) to start loc(min-cost))

        ###################
        disjoint = False           #Flase for standard_splitting
        ####################
        
        while len(self.open_list) != 0:
            curr = self.pop_node()
            if len(curr['collisions']) == 0:
                #print(curr['paths'])
                #print(curr['constraints'])
                self.print_results(curr)
                return curr['paths']
            collision = curr['collisions'][0]
            
            if disjoint == True:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)
            
            for constraint in constraints:
                child = {'cost': 0,
                    'constraints': curr['constraints'] + [constraint],
                    'paths': curr['paths'] + [],
                    'collisions': []}
                if disjoint == True:
                    new_cons = []
                    swit = 0
                    if constraint['positive'] == True:
                        rst = paths_violate_constraint(constraint, curr['paths'])
                        for i in rst:
                            if len(constraint['loc']) == 2:
                                new_cons = {'agent': i, 'loc': [constraint['loc'][1],constraint['loc'][0]]+[], 'timestep': constraint['timestep'], 'positive':False}
                                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                          i, child['constraints'] + [new_cons])
                                if path != None:
                                    child['paths'][i] = path + []
                                    child['constraints'] = child['constraints'] + [new_cons]
                                else:
                                    swit = 1
                                    continue
                            else:
                                new_cons = {'agent': i, 'loc': constraint['loc']+[], 'timestep': constraint['timestep'], 'positive':False}
                                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                          i, child['constraints'] + [new_cons])
                                if path != None:
                                    child['paths'][i] = path + []
                                    child['constraints'] = child['constraints'] + [new_cons]
                                else:
                                    swit = 1
                                    continue
                    
                    if swit == 1:
                        continue

                
                
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                          agent, child['constraints'])
                if path != None:
                    child['paths'][agent] = path + []
                    #print(child['paths'])
                    child['collisions'] = detect_collisions(child['paths'])
                    #print(child['collisions'])
                    #print(child['constraints'])
                    child['cost'] = get_sum_of_cost(child['paths'])
                    if upper_bound >= child['cost']:#len(child['paths'][agent]):
                        self.push_node(child)
                        #print(child['paths'])
        raise BaseException('No solutions')

        self.print_results(root) 
        return root['paths']


    def print_results(self, node):
        #print("Expanded Nodes:")
        #print(self.open_list)
        #print(self.heuristics)
        #print(node['paths'])
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
