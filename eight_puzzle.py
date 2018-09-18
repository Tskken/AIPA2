"""Eight Puzzle classes and utilities.

See: Chapter 3 of Russel and Norving.

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

import search
import random


class EightPuzzleState:
    """This class defines the mechanics of the eight puzzle.

    Note: The task of recasting this puzzle as a search problem is left to
    the EightPuzzleSearchProblem class.
    """

    def __init__(self, numbers):
        """Construct a new eight puzzle from an ordering of numbers.

        numbers: a list of integers from 0 to 8 representing an
          instance of the eight puzzle.  0 represents the blank
          space.  Thus, the list

            [1, 0, 2, 3, 4, 5, 6, 7, 8]

          represents the eight puzzle:
            -------------
            | 1 |   | 2 |
            -------------
            | 3 | 4 | 5 |
            -------------
            | 6 | 7 | 8 |
            ------------

        The configuration of the puzzle is stored in a 2-dimensional
        list (a list of lists) 'cells'.
        """
        self.cells = []
        numbers = numbers[:]  # Make a copy so as not to cause side-effects.
        numbers.reverse()
        for row in range(3):
            self.cells.append([])
            for col in range(3):
                self.cells[row].append(numbers.pop())
                if self.cells[row][col] == 0:
                    self.blank_location = row, col

    def is_goal(self):
        """Check to see if the puzzle is in its goal state.

            -------------
            |   | 1 | 2 |
            -------------
            | 3 | 4 | 5 |
            -------------
            | 6 | 7 | 8 |
            -------------

        Examples:
        >>> EightPuzzleState([0, 1, 2, 3, 4, 5, 6, 7, 8]).is_goal()
        True

        >>> EightPuzzleState([1, 0, 2, 3, 4, 5, 6, 7, 8]).is_goal()
        False

        """
        current = 0
        for row in range(3):
            for col in range(3):
                if current != self.cells[row][col]:
                    return False
                current += 1
        return True

    def legal_moves(self):
        """Return a list of legal moves from the current state.

        Moves consist of moving the blank space up, down, left or right.
        These are encoded as 'up', 'down', 'left' and 'right' respectively.

        Example:
        >>>EightPuzzleState([0, 1, 2, 3, 4, 5, 6, 7, 8]).legal_moves()
        ['down', 'right']

        """
        moves = []
        row, col = self.blank_location
        if(row != 0):
            moves.append('up')
        if(row != 2):
            moves.append('down')
        if(col != 0):
            moves.append('left')
        if(col != 2):
            moves.append('right')
        return moves

    def result(self, move):
        """Return a new EightPuzzle after executing the given move.

        with the current state and blank_location
        updated based on the provided move.

        The move should be a string drawn from a list returned by legal_moves.
        Illegal moves will raise an exception, which may be an array bounds
        exception.

        NOTE: This function *does not* change the current object.  Instead,
        it returns a new object.
        """
        row, col = self.blank_location
        if(move == 'up'):
            newrow = row - 1
            newcol = col
        elif(move == 'down'):
            newrow = row + 1
            newcol = col
        elif(move == 'left'):
            newrow = row
            newcol = col - 1
        elif(move == 'right'):
            newrow = row
            newcol = col + 1
        else:
            raise Exception("Illegal Move")

        # Create a copy of the current EightPuzzle
        new_puzzle = EightPuzzleState([0, 0, 0, 0, 0, 0, 0, 0, 0])
        new_puzzle.cells = [values[:] for values in self.cells]
        # And update it to reflect the move
        new_puzzle.cells[row][col] = self.cells[newrow][newcol]
        new_puzzle.cells[newrow][newcol] = self.cells[row][col]
        new_puzzle.blank_location = newrow, newcol

        return new_puzzle

    # Utilities for comparison and display
    def __eq__(self, other):
        r"""Overloads '==' for eight puzzle configurations.

        Two EightPuzzles with the same configuration are equal.

        Example:
        >>> EightPuzzleState([0, 1, 2, 3, 4, 5, 6, 7, 8]) == \
              EightPuzzleState([1, 0, 2, 3, 4, 5, 6, 7, 8]).result('left')
        True

        """
        for row in range(3):
            if self.cells[row] != other.cells[row]:
                return False
        return True

    def __hash__(self):
        """Overload hash function to be based on puzzle configuration."""
        return hash(str(self.cells))

    def __str__(self):
        """Return a display string for the maze."""
        lines = []
        horizontal_line = ('-' * (13))
        lines.append(horizontal_line)
        for row in self.cells:
            row_line = '|'
            for col in row:
                if col == 0:
                    col = ' '
                row_line = row_line + ' ' + col.__str__() + ' |'
            lines.append(row_line)
            lines.append(horizontal_line)
        return '\n'.join(lines)


class EightPuzzleSearchProblem(search.SearchProblem):
    """Implementation of a SearchProblem for the  Eight Puzzle domain.

    Each state is represented by an instance of an EightPuzzle
    """

    def __init__(self, puzzle):
        """Create a new EightPuzzleSearchProblem.

        Args:
            puzzle (EightPuzzleState): starting state of puzzle
        """
        self.start_state = puzzle

    def get_start_state(self):
        """Get start state of puzzle.

        Overrides search.SearchProblem.get_start_state
        """
        return self.start_state

    def is_goal_state(self, state):
        """Return True if and only if the state is a valid goal state.

        Overrides search.SearchProblem.is_goal_state

        Args:
            state (EightPuzzleState)
        """
        return state.is_goal()

    def get_successors(self, state):
        """Return list of successors from given state.

        Overrides search.SearchProblem.get_successors

        Return:
            List of triples: (successor, action, step_cost),
            where 'successor' is a successor to the current state, 'action'
            is the action required to get there: one of
            {'left', 'right', 'up', down'},
            and the cost is 1.0 for each.
        """
        successors = []
        for a in state.legal_moves():
            successors.append((state.result(a), a, 1))
        return successors

    def get_cost_of_actions(self, actions):
        """Return the total cost of a particular sequence of actions.

        Overrides search.SearchProblem.get_cost_of_actions

        Args:
            actions: A list of actions to take

        Assumes that the sequence is composed of legal moves.
        """
        return len(actions)


EIGHT_PUZZLE_DATA = [[1, 0, 2, 3, 4, 5, 6, 7, 8],
                     [1, 7, 8, 2, 3, 4, 5, 6, 0],
                     [4, 3, 2, 7, 0, 5, 1, 6, 8],
                     [5, 1, 3, 4, 0, 2, 6, 7, 8],
                     [1, 2, 5, 7, 6, 8, 0, 4, 3],
                     [0, 3, 1, 6, 8, 2, 7, 5, 4]]


def load_eight_puzzle(puzzle_number):
    """Return an EightPuzzleState object generated from EIGHT_PUZZLE_DATA.

    Args:
        puzzle_number (int): The number of the eight puzzle to load.
            Can be in range [0,5]


    Example:
      >>> print load_eight_puzzle(0)
      -------------
      | 1 |   | 2 |
      -------------
      | 3 | 4 | 5 |
      -------------
      | 6 | 7 | 8 |
      -------------
    """
    return EightPuzzleState(EIGHT_PUZZLE_DATA[puzzle_number])


def create_random_eight_puzzle(moves=100):
    """Create a random eight puzzle by applying a sequence of random moves.

    Starting with solved puzzle, applies 'moves' moves randomly chosen from
    legal moves.

    Args:
        moves (int): number of random moves to apply
    """
    puzzle = EightPuzzleState([0, 1, 2, 3, 4, 5, 6, 7, 8])
    for i in range(moves):
        # Execute a random legal move
        puzzle = puzzle.result(random.choice(puzzle.legal_moves()))
    return puzzle


def main():
    """Run simple test using BFS to solve random puzzle."""
    puzzle = create_random_eight_puzzle(25)
    print('A random puzzle:')
    print(puzzle)

    problem = EightPuzzleSearchProblem(puzzle)
    path = search.breadth_first_search(problem)
    print(('BFS found a path of %d moves: %s' % (len(path), str(path))))
    curr = puzzle
    i = 1
    for a in path:
        curr = curr.result(a)
        print(('After %d move%s: %s' % (i, ("", "s")[i > 1], a)))
        print(curr)

        input("Press return for the next state...")   # wait for key stroke
        i += 1


if __name__ == "__main__":
    main()
