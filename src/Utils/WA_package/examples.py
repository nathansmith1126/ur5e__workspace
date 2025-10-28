import sympy as sp

from weighted_automaton import WeightedAutomaton


def frac(a, b):
    return sp.Rational(a, b)


def example_fibonacci() -> WeightedAutomaton:
    initial = sp.Matrix([[55, 3, 0]])
    transitions = {'a': sp.Matrix(
        [[frac(13, 8), 0, frac(1, 8)],
         [-frac(1, 8), -frac(5, 8), 0],
         [-frac(1, 8), -frac(5, 8), 0]])}
    final = sp.Matrix([[frac(2, 8)], [-frac(34, 8)], [-frac(34, 8)]])

    return WeightedAutomaton(n=3,
                             alphabet=['a'],
                             initial=initial,
                             transitions=transitions,
                             final=final)


def example_non_integer() -> WeightedAutomaton:
    initial = sp.Matrix([[8, 8, 4, 6]])
    transitions = {'a': sp.Matrix(
        [[0, -1, 0, 0],
         [1, 2, 0, 0],
         [0, 0, 0, -1],
         [0, 0, 1, 2]]), 'b': sp.Matrix(
        [[0, 0, -frac(1, 2), 0],
         [0, 0, 0, -frac(1, 2)],
         [1, 0, frac(3, 2), 0],
         [0, 1, 0, frac(3, 2)]])}
    final = sp.Matrix([[1], [0], [0], [0]])

    return WeightedAutomaton(n=4,
                             alphabet=['a', 'b'],
                             initial=initial,
                             transitions=transitions,
                             final=final)
