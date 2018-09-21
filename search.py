"""Here you will implement generic search algorithms.

Champlain College CSI-480, Fall 2018
The following code was adapted by Joshua Auerbach (jauerbach@champlain.edu)
from the UC Berkeley Pacman Projects (see license and attribution below).

----------------------
Licensing Information:  You are free to use or extend these projects for
educational purposes provided that (1) you do not distribute or publish
solutions, (2) you retain this notice, and (3) you provide clear
attribution to UC Berkeley, including a link to http://ai.berkeley.edu.

Attribution Information: The Pacman AI projects were developed at UC Berkeley.
The core projects and autograders were primarily created by John DeNero
(denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
Student side autograding was added by Brad Miller, Nick Hay, and
Pieter Abbeel (pabbeel@cs.berkeley.edu).
"""

import util


class SearchProblem:
    """This class outlines the structure of a search problem.

    Note this class does not implement any of the methods
    (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """
    def get_start_state(self):
        """Return the start state for the search problem."""
        util.raise_not_defined()

    def is_goal_state(self, state):
        """Return True if and only if the state is a valid goal state."""
        util.raise_not_defined()

    def get_successors(self, state):
        """Return list of successors from given state.

        Return:
            List of triples: (successor, action, step_cost),
            where 'successor' is a successor to the current state,
            'action' is the action required to get there, and 'step_cost' is
            the incremental cost of expanding to that successor.
        """
        util.raise_not_defined()

    def get_cost_of_actions(self, actions):
        """Return the total cost of a particular sequence of actions.

        Args:
            actions: A list of actions to take

        The sequence must be composed of legal moves.
        """
        util.raise_not_defined()


def tiny_maze_search(problem):
    """Return a sequence of moves that solves tiny_maze.

    For any other maze, the sequence of moves will be incorrect,
    so only use this for tiny_maze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def search(problem, fringe):
    """Runs a search problem based on the fringe type you give it

    will work for DFS with a stack and BFS with a queue

    returns a path to the gold as a list of actions
    """
    # Create closed node list with the start state as its first item
    closed_list = [problem.get_start_state()]

    # Expand start state and add its children to the fringe
    # Note: Fringe structure is a list of 2 items, [node name (ie A), list of paths that got you to that node]
    for first_node in problem.get_successors(problem.get_start_state()):
        if type(fringe) is util.PriorityQueue:
            fringe.push([first_node[0], [first_node[1]], first_node[2]], first_node[2])
        else:
            fringe.push([first_node[0], [first_node[1]]])

    # Loop while there is still nodes to expand in the fringe
    while not fringe.is_empty():
        # Pop node off fringe
        node = fringe.pop()
        # Check if the current node is the goal state
        if problem.is_goal_state(node[0]):
            # Return path to goal state
            return node[1]
        # Check if node is in closed list
        if node[0] not in closed_list:
            # Add node to closed list
            closed_list.append(node[0])
            # Expand node adding its children to the fringe
            for nextNode in problem.get_successors(node[0]):
                # Copy path from past node
                path = node[1][:]
                # Add next nodes path to list
                path.append(nextNode[1])
                # Add node to fringe
                if type(fringe) is util.PriorityQueue:
                    fringe.push([nextNode[0], path, node[2] + nextNode[2]], node[2] + nextNode[2])
                else:
                    fringe.push([nextNode[0], path])


def depth_first_search(problem):
    """Run DFS on the given problem."""
    return search(problem, util.Stack())


def breadth_first_search(problem):
    """Run BFS on the given problem."""
    return search(problem, util.Queue())


def uniform_cost_search(problem):
    """Run UCS on the given problem."""
    return search(problem, util.PriorityQueue())


def null_heuristic(state, problem=None):
    """Return a trivial heuristic.

    In general a heuristic function estimates the cost from the current state
    to the nearest goal in the provided SearchProblem.
    """
    return 0


def a_star_search(problem, heuristic=null_heuristic):
    """Run A* on the given problem.

    A* searches the node that has the lowest combined cost and heuristic first.
    """
    # *** YOUR CODE HERE ***
    util.raise_not_defined()


# Abbreviations
bfs = breadth_first_search
dfs = depth_first_search
a_star = a_star_search
ucs = uniform_cost_search
