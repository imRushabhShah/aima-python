
from search import *


us_map = UndirectedGraph(dict(
    Austin=dict(LosAngeles=1377, Charlotte=1200, Dallas=195,Boston=1963),
    Charlotte=dict(NewYork=634),
    SanFrancisco=dict(Seattle=807,Boston=3095,LosAngeles=383,Bakersville=283),
    LosAngeles=dict(Bakersville=153),
    NewYork=dict(Dallas=1548,Boston=225),
    Chicago=dict(Boston=983,Seattle=2064,SantaFe=1272),
    Seattle=dict(SantaFe=1463),
    SantaFe=dict(Bakersville=864,Dallas=640)
))

us_map.locations = dict(
    Austin = 182,
    Charlotte = 929,
    SanFrancisco = 1230,
    LosAngeles = 1100,
    NewYork = 1368,
    Chicago = 800,
    Seattle = 1670,
    SantaFe = 560,
    Bakersville = 1282,
    Boston = 1551,
    Dallas = 0
)


# ## Created new heuristic for straight line distance graph

# In[3]:


class USGraph(GraphProblem):
    def h(self,node):
        '''
            we have straight line distances as heuristic which are already computed
        '''
        locs = getattr(self.graph,'locations',None)
        
        if locs:
            
            if type(node) is str:
                return locs[node]
            return int(locs[node.state])
        else:
            
            return infinity


# In[4]:


# psource(GraphProblem)


# In[5]:


SeattleToDallasProblem = USGraph('Seattle', 'Dallas', us_map)


# # A * Solution for the US map Problem

# In[6]:


def astar_search_show_frontier(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search_show_frontier(problem, lambda n: n.path_cost + h(n))


# In[17]:


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

        print("Explored ==>",explored) 
        print("Frontier ==> ",frontier.heap)
        print()
        node = frontier.pop()
        print("Current ==> ",node.state)
        print("Eval Function ==> ",f(node))
               
            
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


# In[19]:


print("\n\nAstar path solution\n")
route = astar_search_show_frontier(SeattleToDallasProblem).solution()
route = ['Seattle'] + route

print("min cost route for A star")
print(route)
input("press any key to continue.....")

# # RBFS for the US map problem

# In[9]:


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
            s.f = max(s.path_cost + h(s), node.f)
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


# In[10]:


print("\n\n\nRBFS path solution")
route = recursive_best_first_search_show_frontier(SeattleToDallasProblem).solution()
route = ['Seattle']+route
print("Best possible route by RBFS ",route)

input("press any key to continue.....")

# In[ ]:





# # Check consistancy of given Heuristic

# In[11]:


def checkHeuristicOfGraph(graph):
    for parent in graph.graph_dict:
        heuristic_parent = graph.locations[parent]
        for child in graph.graph_dict[parent]:
            heuristic_child = graph.locations[child]
            cost = graph.graph_dict[parent][child]
            if heuristic_child+cost<heuristic_parent:
                print("inconsistant heuristic for parent "+ parent + " child "+child)
            else:
                print("valid heuristic for parent "+ parent + " child "+child)


# In[12]:


checkHeuristicOfGraph(us_map)

