#!/usr/bin/env python
# coding: utf-8

# In[1]:


from search import *



class MissionaryCanibal(Problem):
    '''
    initail=[(3,3),(0,0),1,0], 
    goal=[(0,0),(3,3),0,1]
    defining state as list of size 4
    state[0] is tuple of count (missionariries,canibals) at source
    state[1] is tuple of count (missionariries, canibals) at destination
    state[2] is count of boat at source
    state[3] is count of boat at destination
    
    valid states (3,3),(0,0),(3,2),(0,1),(2,2),(1,1)

    ''' 
    '''
    possible_actions = ["CanibalCanibalToDestination","CanibalToDestination",
                            "MissionaryMissionaryToDestination","MissionaryToDestination",
                            "MissionaryCanibalToDestination",
                            "CanibalCanibalToSource","CanibalToSource",
                            "MissionaryMissionaryToSource","MissionaryToSource",
                            "MissionaryCanibalToSource",
                            ]
    '''
    def __init__(self,initial=((3,3),(0,0),1,0), goal=((0,0),(3,3),0,1)):
        self.goal = goal
        Problem.__init__(self, initial, goal)
    
    def valid_state(self,state):
        Source,Destination,boatAtSource,boatAtDestination = state
        M_S,C_S = Source
        M_D,C_D = Destination 
        if M_S < 0 or M_D < 0 or C_S < 0 or C_D < 0:
            return False
        if M_S<C_S and M_S>0:
            return False
        if M_D<C_D and M_D>0:
            return False        
        return True
        

    def actions(self, state):
        """ Return the actions that can be executed in the given state.
        The result would be a list 

         total_possible_actions = ["CanibalCanibalToDestination","CanibalToDestination",
                            "MissionaryMissionaryToDestination","MissionaryToDestination",
                            "MissionaryCanibalToDestination",
                            "CanibalCanibalToSource","CanibalToSource",
                            "MissionaryMissionaryToSource","MissionaryToSource",
                            "MissionaryCanibalToSource",
                            ]
        """
        possible_actions = []
        Source,Destination,boatAtSource,boatAtDestination = state
        M_S,C_S = Source
        M_D,C_D = Destination 
        
        if boatAtSource:
            """Do something"""
            for MissionaryCount,CanibalCount in [(2,0),(1,0),(1,1),(0,1),(0,2)]:
                new_M_S = M_S - MissionaryCount
                new_M_D = M_D + MissionaryCount    
                new_C_S = C_S - CanibalCount
                new_C_D = C_D + CanibalCount
                    
                new_state = ((new_M_S,new_C_S),(new_M_D,new_C_D),1,0)
                if self.valid_state(new_state):
                    action = "Missionary"*MissionaryCount + "Canibal"*CanibalCount
                    action+="ToDestination"
                    possible_actions.append(action)
            
        elif boatAtDestination:
            """Do something"""
            for MissionaryCount,CanibalCount in [(2,0),(1,0),(1,1),(0,1),(0,2)]:
                new_M_S = M_S + MissionaryCount
                new_M_D = M_D - MissionaryCount    
                new_C_S = C_S + CanibalCount
                new_C_D = C_D - CanibalCount
                    
                new_state = ((new_M_S,new_C_S),(new_M_D,new_C_D),1,0)
                if self.valid_state(new_state):
                    action = "Missionary"*MissionaryCount + "Canibal"*CanibalCount
                    action+="ToSource"
                    possible_actions.append(action)
        
        return possible_actions
    
    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action in the state """
        
        # blank is the index of the blank square
        Source,Destination,boatAtSource,boatAtDestination = state
        M_S,C_S = Source
        M_D,C_D = Destination 
        
        if boatAtSource:
            """Do something"""
            if action == "CanibalCanibalToDestination":
                C_S-=2
                C_D+=2
            elif action =="CanibalToDestination":
                C_S-=1
                C_D+=1
            elif action =="MissionaryMissionaryToDestination":
                M_S-=2
                M_D+=2
            elif action =="MissionaryToDestination":
                M_S-=1
                M_D+=1
            elif action =="MissionaryCanibalToDestination":
                C_S-=1
                C_D+=1
                M_S-=1
                M_D+=1
            new_state = ((M_S,C_S),(M_D,C_D),0,1)
            
        elif boatAtDestination:
            """Do something"""
            if action == "CanibalCanibalToSource":
                C_S+=2
                C_D-=2
            elif action =="CanibalToSource":
                C_S+=1
                C_D-=1
            elif action =="MissionaryMissionaryToSource":
                M_S+=2
                M_D-=2
            elif action =="MissionaryToSource":
                M_S+=1
                M_D-=1
            elif action =="MissionaryCanibalToSource":
                C_S+=1
                C_D-=1
                M_S+=1
                M_D-=1
            new_state = ((M_S,C_S),(M_D,C_D),1,0)
        return tuple(new_state)

    def goal_test(self, state):
        """ Given a state, return True if state is a goal state or False, otherwise """
        return state == self.goal
    
    def h(self, node):
        """ Returns count of total people shipped to destination """
        return (sum(node.state[0]))


# In[3]:


def best_first_graph_search_show_frontier(problem, f,showFrontier = True):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    while frontier:
        if showFrontier:
            print("Explored ==>",explored) 
            print("Frontier ==> ",frontier.heap)
            print("\n")
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node

        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
        
    return None


# In[4]:


def astar_search_show_frontier(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search_show_frontier(problem, lambda n: n.path_cost*2 + h(n))


# In[5]:


def recursive_best_first_search_show_frontier(problem, h=None):
    """[Figure 3.26]"""
    h = memoize(h or problem.h, 'h')
    """Rbf sometimes goes too deep so we will take a step to store all the steps into files 
    we are calling it rbfs_missionay_canibals.txt """
    def RBFS(problem, node, flimit):
        if problem.goal_test(node.state):
            return node, 0  # (The second value is immaterial)
        successors = node.expand(problem)
        if len(successors) == 0:
            return None, infinity
        for s in successors:
            """changed path cost to 2 """
            s.f = max(s.path_cost*2 + h(s), node.f)
        while True:
            # Order by lowest f value
            successors.sort(key=lambda x: x.f)
            best = successors[0]
            if len(successors) > 1:
                alternative = successors[1].f
            else:
                alternative = infinity
            print("flimit "+str(flimit))
            print("best "+str(best.f))
            print("alternative "+str(alternative))
            print("current "+str(node.state))
            if best.f > flimit:
                print("next fail")    
                print("\n")
                return None, best.f
            else:
                print("next "+str(successors[0].state))
                print("\n")
            
            result, best.f = RBFS(problem, best, min(flimit, alternative))
            if result is not None:
                return result, best.f
            
    node = Node(problem.initial)
    node.f = h(node)
    result, bestf = RBFS(problem, node, infinity)
    return result


# In[6]:


def uniform_cost_search_with_frontier(problem):
    """[Figure 3.14]"""
    return best_first_graph_search_show_frontier(problem, lambda node: node.path_cost)


# In[ ]:





# In[7]:


def iterative_deepening_search_with_frontier(problem):
    """[Figure 3.18]"""
    for depth in range(sys.maxsize):
        result = depth_limited_search_with_frontier(problem, depth)
        if result != 'cutoff':
            return result


# In[8]:


def depth_limited_search_with_frontier(problem, limit=50):
    """[Figure 3.17]"""
    print("Depth limit = ",limit)
    def recursive_dls(node, problem, limit, frontier, explored=set({})):
        print("depth",limit)
        print("Current",node.state)
        explored.add(node.state)
        if node.state in frontier:
            frontier.remove(node.state)
        if problem.goal_test(node.state):
            print("Goal state !!!\n")
            return node
        elif limit == 0:
            print('cutoff occured\n')
            return 'cutoff'
        else:
            cutoff_occurred = False
            for child in node.expand(problem):
                frontier.add(child.state)
            print("frontier==> ",frontier)
            print("explored==> ",explored)
            print("\n")
            for child in node.expand(problem):
                result = recursive_dls(child, problem, limit - 1, frontier, explored)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
                else:
                    explored.add(child.state)
                    if child.state in frontier:
                        frontier.remove(child.state)
            return 'cutoff' if cutoff_occurred else None
    
    # Body of depth_limited_search:
    return recursive_dls(Node(problem.initial), problem, limit,frontier=set({Node(problem.initial).state}))

# # Uniform Cost Search

# In[9]:


puzzle = MissionaryCanibal()
print("\n\nUniform Cost Search Problem\n")
path = uniform_cost_search_with_frontier(puzzle).solution()
print("Uniform Cost Search Problem path solution ==> ",path)

input("Press Any Key to continue....")
# # Itterative DFS

# In[10]:


print("\n\nThrougth Itterative DFS\n")
path = iterative_deepening_search_with_frontier(puzzle).solution()
print("Itterative DFS path solution ==> ",path)

input("Press Any Key to continue....")
# #  Best first Search

# In[11]:


print("\n\nThrougth Best first Search\n")
path = best_first_graph_search_show_frontier(puzzle,lambda node: sum(node.state[0])).solution()
print("Best first Search path solution ==> ",path)

input("Press Any Key to continue....")
# #  Astar 

# In[12]:


print("\n\nThrougth Astar solution\n")
path = astar_search_show_frontier(puzzle).solution()
print("Astar path solution ==> ",path)

input("Press Any Key to continue....")
# # Recursive Best First Search

# In[13]:


print("\n\nThrougth RBFS\n")
# just to clear all the content of the file
path = recursive_best_first_search_show_frontier(puzzle).solution()
print("RBFS path solution ==> ",path)

input("Press Any Key to continue....")