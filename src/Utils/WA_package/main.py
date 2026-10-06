from examples import *


def main():
    automaton = example_fibonacci()
    # automaton = example_non_integer()

    print("Initial automaton:")
    automaton.print(newline=True)

    print("Starting minimisation over Z.\n")
    new_automaton = automaton.minimal_z(verbose=True)
    if isinstance(new_automaton, WeightedAutomaton):
        print("Minimized automaton:")
        new_automaton.print(newline=True)
        print("Weight of the first 10 words:")
        print([automaton.weight('a' * i) for i in range(10)])
        print("")
        print("Weight of the first 10 words of the new automaton:")
        print([automaton.weight('a' * i) for i in range(10)])
    else:
        print("Could not minimise automaton over Z.")


if __name__ == "__main__":
    main()
