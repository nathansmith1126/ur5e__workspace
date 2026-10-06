from __future__ import annotations

import sympy as sp
from abelian.linalg.factorizations import smith_normal_form, hermite_normal_form

sp.init_printing()


class WeightedAutomaton:
    def __init__(self,
                 n: int,
                 alphabet: list[str],
                 initial: sp.Matrix,
                 transitions: dict[str, sp.Matrix],
                 final: sp.Matrix):

        # Check is the size of the given matrices are valid
        if initial.shape != (1, n):
            raise ValueError(f"Initial vector must have shape (1, n)\n"
                             f"Expected: (1, {n})\n"
                             f"Received: {initial.shape}")
        for letter in alphabet:
            if transitions[letter].shape != (n, n):
                raise ValueError(f"Transition vector must have shape (n, n)\n"
                                 f"Expected: ({n}, {n})\n"
                                 f"Received: {transitions[letter].shape}")
        if final.shape != (n, 1):
            raise ValueError(f"Final vector must have shape (n, 1)\n"
                             f"Expected: ({n}, 1)\n"
                             f"Received: {final.shape()}")

        self.n = n
        self.alphabet = alphabet
        self.initial = initial
        self.transitions = transitions
        self.final = final

    def __str__(self):
        return (f"n: {self.n}\n"
                f"alphabet: {self.alphabet}\n"
                f"initial: {self.initial}\n"
                f"transitions: {self.transitions}\n"
                f"final: {self.final}")

    def print(self, newline: bool = False) -> None:
        """
        Print the automaton.
        """
        print(f"n: {self.n}")
        print(f"alphabet: {self.alphabet}")
        print("Initial:")
        sp.pretty_print(self.initial)
        for letter in self.alphabet:
            print(f"{letter}:")
            sp.pretty_print(self.transitions[letter])
        print("Final:")
        sp.pretty_print(self.final)
        if newline:
            print("")

    def weight(self, word: str) -> sp.Integer | sp.Rational:
        """
        Compute the weight of a word.
        """
        weight = self.initial
        for letter in word:
            weight *= self.transitions[letter]
        return (weight * self.final)[0]

    def forward_vector(self, word: str) -> sp.Matrix:
        """
        Compute the forward vector of a word.
        """
        forward_vector = self.initial
        for letter in word:
            forward_vector *= self.transitions[letter]
        return forward_vector

    def backward_vector(self, word: str) -> sp.Matrix:
        """
        Compute the backward vector of a word.
        """
        backward_vector = sp.eye(self.n)
        for letter in word:
            backward_vector *= self.transitions[letter]
        return backward_vector * self.final

    def forward_conjugate(self) -> ('WeightedAutomaton', list[str]):
        """
        Compute a forward conjugate of the automaton.
        Also returns the words that represent the
        used forward basis of the forward space.
        """
        # Compute forward space
        words = [""]
        forward_vectors = self.forward_vector("")
        for word in words:
            for letter in self.alphabet:
                forward_vector = self.forward_vector(word + letter)
                new_forward_vectors = forward_vectors.col_join(forward_vector)
                if new_forward_vectors.rank() > forward_vectors.rank():
                    words.append(word + letter)
                    forward_vectors = new_forward_vectors

        # Build new automaton
        initial = forward_vectors.transpose().solve(self.initial.transpose()).transpose()
        transitions = {}
        for letter in self.alphabet:
            transitions[letter] = forward_vectors.transpose().solve(
                (forward_vectors * self.transitions[letter]).transpose()).transpose()
        final = forward_vectors * self.final

        return WeightedAutomaton(n=len(words),
                                 alphabet=self.alphabet,
                                 initial=initial,
                                 transitions=transitions,
                                 final=final), words

    def backward_conjugate(self) -> ('WeightedAutomaton', list[str]):
        """
        Compute a backward conjugate of the automaton.
        Also returns the words that represent the
        used backward basis of the backward space.
        """
        # Compute backward space
        words = [""]
        backward_vectors = self.backward_vector("")
        for word in words:
            for letter in self.alphabet:
                new_backward_vectors = backward_vectors.row_join(self.backward_vector(letter + word))
                if new_backward_vectors.rank() > backward_vectors.rank():
                    words.append(letter + word)
                    backward_vectors = new_backward_vectors

        # Build new automaton
        initial = self.initial * backward_vectors
        transitions = {}
        for letter in self.alphabet:
            transitions[letter] = backward_vectors.solve(self.transitions[letter] * backward_vectors)
        final = backward_vectors.solve(self.final)

        return WeightedAutomaton(n=len(words),
                                 alphabet=self.alphabet,
                                 initial=initial,
                                 transitions=transitions,
                                 final=final), words

    def conjugate_forward(self) -> None:
        """
        Conjugate the automaton forward.
        """
        new_automaton = self.forward_conjugate()[0]
        self.n = new_automaton.n
        self.initial = new_automaton.initial
        self.transitions = new_automaton.transitions
        self.final = new_automaton.final

    def conjugate_backward(self) -> None:
        """
        Conjugate the automaton backward.
        """
        new_automaton = self.backward_conjugate()[0]
        self.n = new_automaton.n
        self.initial = new_automaton.initial
        self.transitions = new_automaton.transitions
        self.final = new_automaton.final

    def minimal(self, backward_first: bool = False) -> 'WeightedAutomaton':
        """
        A compute a minimal equivalent automaton.
        """
        if backward_first:
            return self.backward_conjugate().forward_conjugate()
        return self.forward_conjugate().backward_conjugate()

    def minimise(self, backward_first: bool = False) -> None:
        """
        Minimise the automaton.
        """
        if backward_first:
            self.conjugate_backward()
            self.conjugate_forward()
        else:
            self.conjugate_forward()
            self.conjugate_backward()

    def z_generators(self, verbose: bool = False) -> sp.Matrix | (str, int):
        """
        Determine if the forward module is integer valued.
        If it is, return a set of vectors that generate the
        forward module in the form of a matrix.
        Otherwise, return a word such that the weight of
        the forward vector of that word is not integer valued,
        and the index of the entry that is not an integer.
        """
        # Compute forward space
        words = [""]
        forward_vectors = self.forward_vector("")
        for (index, number) in enumerate(forward_vectors.tolist()[0]):
            if not number.is_integer:
                return "", index
        for word in words:
            for letter in self.alphabet:
                forward_vector = self.forward_vector(word + letter)
                for (index, number) in enumerate(forward_vector.tolist()[0]):
                    if not number.is_integer:
                        return word + letter, index
                new_forward_vectors = forward_vectors.col_join(forward_vector)
                if new_forward_vectors.rank() > forward_vectors.rank():
                    words.append(word + letter)
                    forward_vectors = new_forward_vectors

        if verbose:
            print(f"Forward space words: {words}")
            print("Forward space basis:")
            sp.pretty_print(forward_vectors)
            print("")

        # Compute forward module
        forward_vectors = sp.Matrix(forward_vectors, domain=sp.ZZ)
        for word in words:
            for letter in self.alphabet:
                forward_vector = self.forward_vector(word + letter)
                for (index, number) in enumerate(forward_vector.tolist()[0]):
                    if not number.is_integer:
                        return word + letter, index
                new_forward_vectors = sp.Matrix(forward_vectors.col_join(forward_vector), domain=sp.ZZ)

                ########################################################
                # TODO: Improve this code
                ########################################################
                u, b, _ = smith_normal_form(forward_vectors.transpose())
                uc = u * forward_vector.transpose()
                solvable = True
                for i in range(min(b.shape[0], b.shape[1])):
                    if b[i, i] == 0:
                        if not uc[i] == 0:
                            solvable = False
                        continue
                    if uc[i] % b[i, i] != 0:
                        solvable = False
                if not solvable:
                    words.append(word + letter)
                    forward_vectors = new_forward_vectors
                ########################################################

        if verbose:
            print(f"Forward module words: {words}")
            print("Forward module vectors:")
            sp.pretty_print(forward_vectors)
            print("")

        return forward_vectors

    def minimal_z(self, verbose: bool = False) -> 'WeightedAutomaton' | str:
        """
        Determine if the semantics of the automaton are integer valued.
        If they are, return an equivalent minimal automaton consisting
        of only integers.
        Otherwise, return a word such that its weight is not an integer.
        """
        backward_conjugate, words = self.backward_conjugate()
        if verbose:
            print("Backward conjugate:")
            backward_conjugate.print(newline=True)

        z_generators_ret = backward_conjugate.z_generators(verbose=verbose)
        if isinstance(z_generators_ret, tuple):
            word = z_generators_ret[0] + words[z_generators_ret[1]]
            if verbose:
                print(f"Automaton is not z-valued.\n"
                      f"Word \"{word}\" has weight {backward_conjugate.weight(word)}.\n")
            return word
        # We use the HNF from Abelian instead of Sympy because the Sympy version gives unexpected results
        _, forward_basis = hermite_normal_form(z_generators_ret.transpose())
        forward_basis = forward_basis.transpose()[:forward_basis.rank(), :]
        if verbose:
            print("Forward integer basis:")
            sp.pretty_print(forward_basis)
            print("")

        initial = forward_basis.transpose().solve(backward_conjugate.initial.transpose()).transpose()
        transitions = {}
        for letter in backward_conjugate.alphabet:
            transitions[letter] = forward_basis.transpose().solve(
                (forward_basis * backward_conjugate.transitions[letter]).transpose()).transpose()
        final = forward_basis * backward_conjugate.final

        return WeightedAutomaton(n=forward_basis.rank(),
                                 alphabet=backward_conjugate.alphabet,
                                 initial=initial,
                                 transitions=transitions,
                                 final=final)

    def minimise_z(self, verbose: bool = False) -> None:
        """
        Determine if the semantics of the automaton are integer valued.
        If they are, minimise this automaton to an equivalent automaton
        consisting of only integers.
        Otherwise, minimise this automaton regularly.
        """
        minimal_z_ret = self.minimal_z(verbose=verbose)
        if isinstance(minimal_z_ret, str):
            self.minimise()
        else:
            self.n = minimal_z_ret.n
            self.initial = minimal_z_ret.initial
            self.transitions = minimal_z_ret.transitions
            self.final = minimal_z_ret.final
