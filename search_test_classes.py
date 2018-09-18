"""PA 2 specific autograding test classes.

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

import re
import textwrap

# import project specific code
import test_classes
import layout
import pacman
from search import SearchProblem
from game import Actions


def wrap_solution(solution):
    """Use for printing solutions in solution files."""
    if isinstance(solution, list):
        return '\n'.join(textwrap.wrap(' '.join(solution)))
    else:
        return str(solution)


def follow_action(state, action, problem):
    """Return successor state of following action."""
    for successor1, action1, cost1 in problem.get_successors(state):
        if action == action1:
            return successor1
    return None


def follow_path(path, problem):
    """Return sequence of states encountered by following path."""
    state = problem.get_start_state()
    states = [state]
    for action in path:
        state = follow_action(state, action, problem)
        states.append(state)
    return states


def check_solution(problem, path):
    """Check if path results in a goal state."""
    state = problem.get_start_state()
    for action in path:
        state = follow_action(state, action, problem)
    return problem.is_goal_state(state)


class GraphSearch(SearchProblem):
    """Search problem on a plain graph."""

    def __init__(self, graph_text):
        """Read in the state graph.

        Define start/end states, edges and costs.
        """
        self.expanded_states = []
        lines = graph_text.split('\n')
        r = re.match('start_state:(.*)', lines[0])
        if r is None:
            print("Broken graph:")
            print('"""%s"""' % graph_text)
            raise Exception("GraphSearch graph specification start_state not" +
                            " found or incorrect on line:" + lines[0])
        self.start_state = r.group(1).strip()
        r = re.match('goal_states:(.*)', lines[1])
        if r is None:
            print("Broken graph:")
            print('"""%s"""' % graph_text)
            raise Exception("GraphSearch graph specification goal_states not" +
                            " found or incorrect on line:" + lines[1])
        goals = r.group(1).split()
        self.goals = list(map(str.strip, goals))
        self.successors = {}
        all_states = set()
        self.ordered_successor_tuples = []
        for l in lines[2:]:
            if len(l.split()) == 3:
                start, action, next_state = l.split()
                cost = 1
            elif len(l.split()) == 4:
                start, action, next_state, cost = l.split()
            else:
                print("Broken graph:")
                print('"""%s"""' % graph_text)
                raise Exception("Invalid line in GraphSearch graph " +
                                "specification on line:" + l)
            cost = float(cost)
            self.ordered_successor_tuples.append((start, action,
                                                  next_state, cost))
            all_states.add(start)
            all_states.add(next_state)
            if start not in self.successors:
                self.successors[start] = []
            self.successors[start].append((next_state, action, cost))
        for s in all_states:
            if s not in self.successors:
                self.successors[s] = []

    def get_start_state(self):
        """Get start state."""
        return self.start_state

    def is_goal_state(self, state):
        """Check if a state is a goal state."""
        return state in self.goals

    def get_successors(self, state):
        """Get all successors of a state."""
        self.expanded_states.append(state)
        return list(self.successors[state])

    def get_cost_of_actions(self, actions):
        """Calculate total cost of a sequence of actions."""
        total_cost = 0
        state = self.start_state
        for a in actions:
            successors = self.successors[state]
            match = False
            for (next_state, action, cost) in successors:
                if a == action:
                    state = next_state
                    total_cost += cost
                    match = True
            if not match:
                raise Exception('invalid action sequence ' + str(actions))
        return total_cost

    def get_expanded_states(self):
        """Return a list of all states on which 'get_successors' was called."""
        return self.expanded_states

    def __str__(self):
        """Return string representation."""
        # print(self.successors)
        edges = ["%s %s %s %s" % t for t in self.ordered_successor_tuples]
        return """start_state: %s
goal_states: %s
%s""" % (self.start_state, " ".join(self.goals), "\n".join(edges))


def parse_heuristic(heuristic_text):
    """Parse the given heuristic and return as a function."""
    heuristic = {}
    for line in heuristic_text.split('\n'):
        tokens = line.split()
        if len(tokens) != 2:
            print("Broken heuristic:")
            print('"""%s"""' % heuristic_text)
            raise Exception("GraphSearch heuristic specification broken:"
                            + line)
        state, h = tokens
        heuristic[state] = float(h)

    def graph_heuristic(state, problem=None):
        if state in heuristic:
            return heuristic[state]
        else:
            print("Heuristic:")
            print(heuristic)
            raise Exception("Graph heuristic called with invalid state: "
                            + str(state))

    return graph_heuristic


class GraphSearchTest(test_classes.TestCase):
    """Test case for graph search problems."""

    def __init__(self, question, test_dict):
        """Extend test_classes.TestCase.__init__."""
        super().__init__(question, test_dict)
        self.graph_text = test_dict['graph']
        self.alg = test_dict['algorithm']
        self.diagram = test_dict['diagram']
        self.exact_expansion_order = test_dict.get('exact_expansion_order',
                                                   'True').lower() == "true"
        if 'heuristic' in test_dict:
            self.heuristic = parse_heuristic(test_dict['heuristic'])
        else:
            self.heuristic = None

    def get_solution_info(self, search):
        """Return solution info: (solution, expanded states, error message)."""
        alg = getattr(search, self.alg)
        problem = GraphSearch(self.graph_text)
        if self.heuristic is not None:
            solution = alg(problem, self.heuristic)
        else:
            solution = alg(problem)

        if not isinstance(solution, list):
            return (None, None,
                    'The result of %s must be a list. (Instead, it is %s)' %
                    (self.alg, type(solution)))

        return solution, problem.get_expanded_states(), None

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides test_classes.TestCase.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        search = module_dict['search']
        # search_agents = module_dict['search_agents']
        gold_solution = [str.split(solution_dict['solution']),
                         str.split(solution_dict['rev_solution'])]
        gold_expanded_states = [str.split(solution_dict['expanded_states']),
                                str.split(solution_dict['rev_expanded_states'])
                                ]

        solution, expanded_states, error = self.get_solution_info(search)
        if error is not None:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('\t%s' % error)
            return False

        if (solution in gold_solution and
            (not self.exact_expansion_order or
             expanded_states in gold_expanded_states)):
            grades.add_message('PASS: %s' % self.path)
            grades.add_message('\tsolution:\t\t%s' % solution)
            grades.add_message('\texpanded_states:\t%s' % expanded_states)
            return True
        else:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('\tgraph:')
            for line in self.diagram.split('\n'):
                grades.add_message('\t    %s' % (line, ))
            grades.add_message('\tstudent solution:\t\t%s' % solution)
            grades.add_message('\tstudent expanded_states:\t%s' %
                               expanded_states)
            grades.add_message('')
            grades.add_message('\tcorrect solution:\t\t%s' %
                               gold_solution[0])
            grades.add_message('\tcorrect expanded_states:\t%s' %
                               gold_expanded_states[0])
            grades.add_message('\tcorrect rev_solution:\t\t%s' %
                               gold_solution[1])
            grades.add_message('\tcorrect rev_expanded_states:\t%s' %
                               gold_expanded_states[1])
            return False

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides test_classes.TestCase.write_solution
        """
        search = module_dict['search']
        # search_agents = module_dict['search_agents']
        # open file and write comments
        handle = open(file_path, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# This solution is designed to support both ' +
                     'right-to-left\n')
        handle.write('# and left-to-right implementations.\n')

        # write forward solution
        solution, expanded_states, error = self.get_solution_info(search)
        if error is not None:
            raise Exception("Error in solution code: %s" % error)
        handle.write('solution: "%s"\n' % ' '.join(solution))
        handle.write('expanded_states: "%s"\n' % ' '.join(expanded_states))

        # reverse and write backwards solution
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        solution, expanded_states, error = self.get_solution_info(search)
        if error is not None:
            raise Exception("Error in solution code: %s" % error)
        handle.write('rev_solution: "%s"\n' % ' '.join(solution))
        handle.write('rev_expanded_states: "%s"\n' % ' '.join(expanded_states))

        # clean up
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        handle.close()
        return True


class PacmanSearchTest(test_classes.TestCase):
    """Test case for Pacman search problems."""

    def __init__(self, question, test_dict):
        """Extend test_classes.TestCase.__init__."""
        super().__init__(question, test_dict)
        self.layout_text = test_dict['layout']
        self.alg = test_dict['algorithm']
        self.layout_name = test_dict['layout_name']

        # TODO: sensible to have defaults like this?
        self.leeway_factor = float(test_dict.get('leeway_factor', '1'))
        self.cost_fn = eval(test_dict.get('cost_fn', 'None'))
        self.search_problem_class_name = test_dict.get('search_problem_class',
                                                       'PositionSearchProblem')
        self.heuristic_name = test_dict.get('heuristic', None)

    def get_solution_info(self, search, search_agents):
        """Return solution info: (solution, expanded states, error message)."""
        alg = getattr(search, self.alg)
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        start_state = pacman.GameState()
        start_state.initialize(lay, 0)

        problem_class = getattr(search_agents, self.search_problem_class_name)
        problem_options = {}
        if self.cost_fn is not None:
            problem_options['cost_fn'] = self.cost_fn
        problem = problem_class(start_state, **problem_options)
        if self.heuristic_name is not None:
            heuristic = getattr(search_agents, self.heuristic_name)
        else:
            heuristic = None

        if heuristic is not None:
            solution = alg(problem, heuristic)
        else:
            solution = alg(problem)

        if not isinstance(solution, list):
            return (None, None,
                    'The result of %s must be a list. (Instead, it is %s)' %
                    (self.alg, type(solution)))

        from game import Directions
        dirs = list(Directions.LEFT.keys())
        if [el in dirs for el in solution].count(False) != 0:
            return (None, None,
                    ('Output of %s must be a list of actions ' % self.alg) +
                    'from game.Directions')

        expanded = problem._expanded
        return solution, expanded, None

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides test_classes.TestCase.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        gold_solution = [str.split(solution_dict['solution']),
                         str.split(solution_dict['rev_solution'])]
        gold_expanded = max(int(solution_dict['expanded_nodes']),
                            int(solution_dict['rev_expanded_nodes']))

        solution, expanded, error = self.get_solution_info(search,
                                                           search_agents)
        if error is not None:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('%s' % error)
            return False

        # FIXME: do we want to standardize test output format?

        if solution not in gold_solution:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('Solution not correct.')
            grades.add_message('\tstudent solution length: %s' % len(solution))
            grades.add_message('\tstudent solution:\n%s' %
                               wrap_solution(solution))
            grades.add_message('')
            grades.add_message('\tcorrect solution length: %s' %
                               len(gold_solution[0]))
            grades.add_message('\tcorrect (reversed) solution length: %s' %
                               len(gold_solution[1]))
            grades.add_message('\tcorrect solution:\n%s' %
                               wrap_solution(gold_solution[0]))
            grades.add_message('\tcorrect (reversed) solution:\n%s' %
                               wrap_solution(gold_solution[1]))
            return False

        if (expanded > self.leeway_factor * gold_expanded and
                expanded > gold_expanded + 1):
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('Too many node expanded; are you expanding' +
                               ' nodes twice?')
            grades.add_message('\tstudent nodes expanded: %s' % expanded)
            grades.add_message('')
            grades.add_message(('\tcorrect nodes expanded: ' +
                                '%s (leeway_factor %s)') %
                               (gold_expanded, self.leeway_factor))
            return False

        grades.add_message('PASS: %s' % self.path)
        grades.add_message('\tpacman layout:\t\t%s' % self.layout_name)
        grades.add_message('\tsolution length: %s' % len(solution))
        grades.add_message('\tnodes expanded:\t\t%s' % expanded)
        return True

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides test_classes.TestCase.write_solution
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        # open file and write comments
        handle = open(file_path, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# This solution is designed to support both ' +
                     'right-to-left\n')
        handle.write('# and left-to-right implementations.\n')
        handle.write('# Number of nodes expanded must be with a factor of ' +
                     '%s of the numbers below.\n' % self.leeway_factor)

        # write forward solution
        solution, expanded, error = self.get_solution_info(search,
                                                           search_agents)
        if error is not None:
            raise Exception("Error in solution code: %s" % error)
        handle.write('solution: """\n%s\n"""\n' % wrap_solution(solution))
        handle.write('expanded_nodes: "%s"\n' % expanded)

        # write backward solution
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        solution, expanded, error = self.get_solution_info(search,
                                                           search_agents)
        if error is not None:
            raise Exception("Error in solution code: %s" % error)
        handle.write('rev_solution: """\n%s\n"""\n' % wrap_solution(solution))
        handle.write('rev_expanded_nodes: "%s"\n' % expanded)

        # clean up
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        handle.close()
        return True


def get_states_from_path(start, path):
    """Return the list of states visited along the path."""
    vis = [start]
    curr = start
    for a in path:
        x, y = curr
        dx, dy = Actions.direction_to_vector(a)
        curr = (int(x + dx), int(y + dy))
        vis.append(curr)
    return vis


class CornerProblemTest(test_classes.TestCase):
    """Test for navigating to all corners."""

    def __init__(self, question, test_dict):
        """Extend test_classes.TestCase.__init__."""
        super().__init__(question, test_dict)
        self.layout_text = test_dict['layout']
        self.layout_name = test_dict['layout_name']

    def solution(self, search, search_agents):
        """Return solution info: (solution, missed_corners)."""
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        game_state = pacman.GameState()
        game_state.initialize(lay, 0)
        problem = search_agents.CornersProblem(game_state)
        path = search.bfs(problem)

        game_state = pacman.GameState()
        game_state.initialize(lay, 0)
        visited = get_states_from_path(game_state.get_pacman_position(), path)
        top = game_state.get_walls().height - 2
        right = game_state.get_walls().width - 2
        missed_corners = [p for p in ((1, 1), (1, top),
                                      (right, 1), (right, top))
                          if p not in visited]

        return path, missed_corners

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides test_classes.TestCase.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        gold_length = int(solution_dict['solution_length'])
        solution, missed_corners = self.solution(search, search_agents)

        if not isinstance(solution, list):
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('The result must be a list. (Instead, it is %s)'
                               % type(solution))
            return False

        if len(missed_corners) != 0:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('Corners missed: %s' % missed_corners)
            return False

        if len(solution) != gold_length:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('Optimal solution not found.')
            grades.add_message('\tstudent solution length:\n%s'
                               % len(solution))
            grades.add_message('')
            grades.add_message('\tcorrect solution length:\n%s' % gold_length)
            return False

        grades.add_message('PASS: %s' % self.path)
        grades.add_message('\tpacman layout:\t\t%s' % self.layout_name)
        grades.add_message('\tsolution length:\t\t%s' % len(solution))
        return True

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides test_classes.TestCase.write_solution
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        # open file and write comments
        handle = open(file_path, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)

        print("Solving problem", self.layout_name)
        print(self.layout_text)

        path, _ = self.solution(search, search_agents)
        length = len(path)
        print("Problem solved")

        handle.write('solution_length: "%s"\n' % length)
        handle.close()


# template = """class: "HeuristicTest"
#
# heuristic: "food_heuristic"
# search_problem_class: "FoodSearchProblem"
# layout_name: "Test %s"
# layout: \"\"\"
# %s
# \"\"\"
# """
#
# for i, (_, _, l) in enumerate(done_tests + food_tests):
#     f = open("food_heuristic_%s.test" % (i+1), "w")
#     f.write(template % (i+1, "\n".join(l)))
#     f.close()

class HeuristicTest(test_classes.TestCase):
    """Test for heuristics: check consistency and admissibility."""

    def __init__(self, question, test_dict):
        """Extend test_classes.TestCase.__init__."""
        super().__init__(question, test_dict)
        self.layout_text = test_dict['layout']
        self.layout_name = test_dict['layout_name']
        self.search_problem_class_name = test_dict['search_problem_class']
        self.heuristic_name = test_dict['heuristic']

    def setup_problem(self, search_agents):
        """Set up the problem."""
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        game_state = pacman.GameState()
        game_state.initialize(lay, 0)
        problem_class = getattr(search_agents, self.search_problem_class_name)
        problem = problem_class(game_state)
        state = problem.get_start_state()
        heuristic = getattr(search_agents, self.heuristic_name)

        return problem, state, heuristic

    def check_heuristic(self, heuristic, problem, state, solution_cost):
        """Run tests on the heuristic."""
        h0 = heuristic(state, problem)

        if solution_cost == 0:
            if h0 == 0:
                return True, ''
            else:
                return False, 'Heuristic failed H(goal) == 0 test'

        if h0 < 0:
            return False, 'Heuristic failed H >= 0 test'
        if not h0 > 0:
            return False, 'Heuristic failed non-triviality test'
        if not h0 <= solution_cost:
            return False, 'Heuristic failed admissibility test'

        for succ, action, step_cost in problem.get_successors(state):
            h1 = heuristic(succ, problem)
            if h1 < 0:
                return False, 'Heuristic failed H >= 0 test'
            if h0 - h1 > step_cost:
                return False, 'Heuristic failed consistency test'

        return True, ''

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides test_classes.TestCase.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        # search = module_dict['search']
        search_agents = module_dict['search_agents']
        solution_cost = int(solution_dict['solution_cost'])
        problem, state, heuristic = self.setup_problem(search_agents)

        passed, message = self.check_heuristic(heuristic, problem, state,
                                               solution_cost)

        if not passed:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('%s' % message)
            return False
        else:
            grades.add_message('PASS: %s' % self.path)
            return True

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides test_classes.TestCase.write_solution
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        # open file and write comments
        handle = open(file_path, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)

        print("Solving problem", self.layout_name, self.heuristic_name)
        print(self.layout_text)
        problem, _, heuristic = self.setup_problem(search_agents)
        path = search.a_star(problem, heuristic)
        cost = problem.get_cost_of_actions(path)
        print("Problem solved")

        handle.write('solution_cost: "%s"\n' % cost)
        handle.close()
        return True


class HeuristicGrade(HeuristicTest):
    """Test for goodness of Heuristic.

    The fewer nodes expanded the higher the grade will be.
    """

    def __init__(self, question, test_dict):
        """Extend HeuristicTest.__init__."""
        super().__init__(question, test_dict)
        self.base_points = int(test_dict['base_points'])
        self.thresholds = [int(t) for t in
                           test_dict['grading_thresholds'].split()]

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides HeuristicTest.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        problem, _, heuristic = self.setup_problem(search_agents)

        path = search.a_star(problem, heuristic)

        expanded = problem._expanded

        if not check_solution(problem, path):
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('\t_returned path is not a solution.')
            grades.add_message('\tpath returned by a_star: %s' % expanded)
            return False

        grades.add_points(self.base_points)
        points = 0
        for threshold in self.thresholds:
            if expanded <= threshold:
                points += self.base_points
        grades.add_points(points)
        if points >= len(self.thresholds):
            grades.add_message('PASS: %s' % self.path)
        else:
            grades.add_message('FAIL: %s' % self.path)
        grades.add_message('\texpanded nodes: %s' % expanded)
        grades.add_message('\tthresholds: %s' % self.thresholds)

        return True

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides HeuristicTest.write_solution
        """
        handle = open(file_path, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# File intentionally blank.\n')
        handle.close()
        return True


# template = """class: "ClosestDotTest"
#
# layout_name: "Test %s"
# layout: \"\"\"
# %s
# \"\"\"
# """
#
# for i, (_, _, l) in enumerate(food_tests):
#     f = open("closest_dot_%s.test" % (i+1), "w")
#     f.write(template % (i+1, "\n".join(l)))
#     f.close()

class ClosestDotTest(test_classes.TestCase):
    """Test for finding closest dot."""

    def __init__(self, question, test_dict):
        """Extend test_classes.TestCase.__init__."""
        super().__init__(question, test_dict)
        self.layout_text = test_dict['layout']
        self.layout_name = test_dict['layout_name']

    def solution(self, search_agents):
        """Return path."""
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        game_state = pacman.GameState()
        game_state.initialize(lay, 0)
        path = search_agents.ClosestDotSearchAgent().find_path_to_closest_dot(
            game_state)
        return path

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides test_classes.TestCase.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        # search = module_dict['search']
        search_agents = module_dict['search_agents']
        gold_length = int(solution_dict['solution_length'])
        solution = self.solution(search_agents)

        if not isinstance(solution, list):
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('\t_the result must be a list. ' +
                               '(Instead, it is %s)' % type(solution))
            return False

        if len(solution) != gold_length:
            grades.add_message('FAIL: %s' % self.path)
            grades.add_message('Closest dot not found.')
            grades.add_message('\tstudent solution length:\n%s' %
                               len(solution))
            grades.add_message('')
            grades.add_message('\tcorrect solution length:\n%s' % gold_length)
            return False

        grades.add_message('PASS: %s' % self.path)
        grades.add_message('\tpacman layout:\t\t%s' % self.layout_name)
        grades.add_message('\tsolution length:\t\t%s' % len(solution))
        return True

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides test_classes.TestCase.write_solution
        """
        # search = module_dict['search']
        search_agents = module_dict['search_agents']
        # open file and write comments
        handle = open(file_path, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)

        print("Solving problem", self.layout_name)
        print(self.layout_text)

        length = len(self.solution(search_agents))
        print("Problem solved")

        handle.write('solution_length: "%s"\n' % length)
        handle.close()
        return True


class CornerHeuristicSanity(test_classes.TestCase):
    """Test for heuristic sanity in corners problem."""

    def __init__(self, question, test_dict):
        """Extend test_classes.TestCase.__init__."""
        super().__init__(question, test_dict)
        self.layout_text = test_dict['layout']

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides test_classes.TestCase.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        # search = module_dict['search']
        search_agents = module_dict['search_agents']
        game_state = pacman.GameState()
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        game_state.initialize(lay, 0)
        problem = search_agents.CornersProblem(game_state)
        start_state = problem.get_start_state()
        h0 = search_agents.corners_heuristic(start_state, problem)
        succs = problem.get_successors(start_state)
        # corner_consistency_a
        for succ in succs:
            h1 = search_agents.corners_heuristic(succ[0], problem)
            if h0 - h1 > 1:
                grades.add_message('FAIL: inconsistent heuristic')
                return False
        heuristic_cost = search_agents.corners_heuristic(start_state, problem)
        true_cost = float(solution_dict['cost'])
        # corner_nontrivial
        if heuristic_cost == 0:
            grades.add_message('FAIL: must use non-trivial heuristic')
            return False
        # corner_admissible
        if heuristic_cost > true_cost:
            grades.add_message('FAIL: Inadmissible heuristic')
            return False
        path = solution_dict['path'].split()
        states = follow_path(path, problem)
        heuristics = []
        for state in states:
            heuristics.append(search_agents.corners_heuristic(state, problem))
        for i in range(0, len(heuristics) - 1):
            h0 = heuristics[i]
            h1 = heuristics[i + 1]
            # corner_consistency_b
            if h0 - h1 > 1:
                grades.add_message('FAIL: inconsistent heuristic')
                return False
            # corner_pos_h
            if h0 < 0 or h1 < 0:
                grades.add_message('FAIL: non-positive heuristic')
                return False
        # corner_goal_h
        if heuristics[len(heuristics) - 1] != 0:
            grades.add_message('FAIL: heuristic non-zero at goal')
            return False
        grades.add_message('PASS: heuristic value less than true cost ' +
                           'at start state')
        return True

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides test_classes.TestCase.write_solution
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        # write comment
        handle = open(file_path, 'w')
        handle.write('# In order for a heuristic to be admissible,' +
                     ' the value\n')
        handle.write('# of the heuristic must be less at each state ' +
                     'than the\n')
        handle.write('# true cost of the optimal path from that state ' +
                     'to a goal.\n')

        # solve problem and write solution
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        start_state = pacman.GameState()
        start_state.initialize(lay, 0)
        problem = search_agents.CornersProblem(start_state)
        solution = search.a_star(problem, search_agents.corners_heuristic)
        handle.write('cost: "%d"\n' % len(solution))
        handle.write('path: """\n%s\n"""\n' % wrap_solution(solution))
        handle.close()
        return True


class CornerHeuristicPacman(test_classes.TestCase):
    """Test for heuristic in corners problem."""

    def __init__(self, question, test_dict):
        """Extend test_classes.TestCase.__init__."""
        super().__init__(question, test_dict)
        self.layout_text = test_dict['layout']

    def execute(self, grades, module_dict, solution_dict):
        """Run student code.

        Overrides test_classes.TestCase.execute

        If an error message is returned, print error and return false.
        If a good solution is returned, print the solution and return true.
        Otherwise, print both the correct and student's solution and return
        false.
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        # total = 0
        true_cost = float(solution_dict['cost'])
        thresholds = list(map(int, solution_dict['thresholds'].split()))
        points_per_threshold = 1
        if 'points_per_threshold' in solution_dict:
            points_per_threshold = int(solution_dict['points_per_threshold'])
        game_state = pacman.GameState()
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        game_state.initialize(lay, 0)
        problem = search_agents.CornersProblem(game_state)
        start_state = problem.get_start_state()
        if search_agents.corners_heuristic(start_state, problem) > true_cost:
            grades.add_message('FAIL: Inadmissible heuristic')
            return False
        path = search.a_star(problem, search_agents.corners_heuristic)
        print("path:", path)
        print("path length:", len(path))
        cost = problem.get_cost_of_actions(path)
        if cost > true_cost:
            grades.add_message('FAIL: Inconsistent heuristic')
            return False
        expanded = problem._expanded
        points = 0
        for threshold in thresholds:
            if expanded <= threshold:
                points += points_per_threshold
        grades.add_points(points)
        if points >= len(thresholds):
            grades.add_message('PASS: Heuristic resulted in expansion of ' +
                               '%d nodes' % expanded)
        else:
            grades.add_message('FAIL: Heuristic resulted in expansion of ' +
                               '%d nodes' % expanded)
        return True

    def write_solution(self, module_dict, file_path):
        """Write solution for the test.

        Overrides test_classes.TestCase.write_solution
        """
        search = module_dict['search']
        search_agents = module_dict['search_agents']
        # write comment
        handle = open(file_path, 'w')
        handle.write('# This solution file specifies the length of the ' +
                     'optimal path\n')
        handle.write('# as well as the thresholds on number of nodes ' +
                     'expanded to be\n')
        handle.write('# used in scoring.\n')

        # solve problem and write solution
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        start_state = pacman.GameState()
        start_state.initialize(lay, 0)
        problem = search_agents.CornersProblem(start_state)
        solution = search.a_star(problem, search_agents.corners_heuristic)
        handle.write('cost: "%d"\n' % len(solution))
        handle.write('path: """\n%s\n"""\n' % wrap_solution(solution))
        handle.write('thresholds: "2000 1600 1200"\n')
        handle.close()
        return True
