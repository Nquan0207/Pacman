# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    from util import Stack  # Assuming util.Stack is available for stack operations
    stack = Stack()
    stack.push((problem.getStartState(), []))
    visited = set()

    while not stack.isEmpty():
        # Pop the current node and the path taken to reach it
        state, actions = stack.pop()
        # If the state is the goal, return the path of actions
        if problem.isGoalState(state):
            return actions

        # If the state has not been visited, explore its successors
        if state not in visited:
            visited.add(state)  # Mark as visited

            # Get all successors: (successor, action, stepCost)
            for successor, action, _ in problem.getSuccessors(state):
                if successor not in visited:
                    # Push the successor with the updated path of actions
                    stack.push((successor, actions + [action]))

    # If no solution is found, raise an error
    return []

    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue  # Assuming you have a Queue utility available

    # Initialize the queue with the start state and an empty list of actions
    queue = Queue()
    queue.push((problem.getStartState(), []))

    # Initialize the visited set to track explored states
    visited = set()

    while not queue.isEmpty():
        # Dequeue the current state and the path of actions to reach it
        state, actions = queue.pop()

        # If the current state is the goal, return the path of actions
        if problem.isGoalState(state):
            return actions

        # If the state has not been visited, explore its successors
        if state not in visited:
            visited.add(state)  # Mark the state as visited

            # Get all successors: (successor, action, stepCost)
            for successor, action, _ in problem.getSuccessors(state):
                if successor not in visited:
                    # Enqueue the successor with the updated path of actions
                    queue.push((successor, actions + [action]))

    # If no solution is found, return an empty list (or raise an error)
    return []
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    frontier = PriorityQueue() 
    s = problem.getStartState()
    frontier.push((0,s),0)
    visited = {s:(0, None, None)}
    
    while not frontier.isEmpty():
        du, u = frontier.pop()
        if problem.isGoalState(u):
            path = []
            while u is not None:
                path.append(visited[u][2])
                u = visited[u][1]  
            path.reverse()
            return path[1:]
        
        neighbours = problem.getSuccessors(u) #state, action, cost
        for v,au,dv in neighbours:
            total_cost = du + dv
            if v not in visited or total_cost < visited[v][0]:
                visited[v] = (total_cost, u, au)
                frontier.push((total_cost,v),total_cost)

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
