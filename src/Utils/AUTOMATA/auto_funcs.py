import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from automata.fa.dfa import DFA
from WA_package.weighted_automaton import WeightedAutomaton
from enum import Enum
from typing import Optional, List, Dict, Tuple
import sympy as sp
import numpy as np 
# import builtins

# # Disable print
# builtins.print = lambda *args, **kwargs: None

class DFAMonitor:
    def __init__(self, dfa, 
                 potential_dict: Optional[ Dict ] = None, 
                 gamma: Optional[ float ] = 0.9):
        "monitors discrete finite automaton"
        self.dfa = dfa
        self.current_state = dfa.initial_state
        self.history = []
        
        # boolean to track if progress is made
        self.progress = False
        
        # dictionary mapping between automaton states and potential function values
        self.potential_dict = potential_dict
        
        # discount factor for potential difference calculation
        self.gamma = gamma

        # most recent potential difference initialized to zero
        self.delta_potential = 0.0
        
        "list of states"
        state_list = list(dfa.states)
        
        self.num_states = len(state_list)
        
        "dictionary mapping between states and natural numbers"
        self.states_dict = {state: index for index, state in enumerate(sorted(state_list))}
  
    def step(self, event: Optional[ str ] = None ):
        """
        step function for dfa
        input [str] - event 
        """
        "assume no progress is made by default"
        self.progress = False
        if not event:
            pass
        else:
            """
            check for invalid events
            """
            if event not in self.dfa.input_symbols:
                print(f"Undefined symbol: {event}")
                raise ValueError(f"Invalid input event: {event}")
            elif event not in self.dfa.transitions[self.current_state]:
                raise ValueError(f"No transition from {self.current_state} on {event}")
            
            next_state = self.dfa.transitions[self.current_state][event]
            
            # append history
            self.history.append((self.current_state, event, next_state))
            
            # update potential difference if potential dict provided
            if self.potential_dict:
                self.update_potential_difference(current_state=self.current_state,
                                                 next_state=next_state)
            
            # check for progress 
            if not next_state == self.current_state:
                "transition has been made"
                self.progress = True
                # print(f"progress is {self.progress}")
            # print(f"{self.current_state} --[{event}]--> {next_state}")
            # print(f"state before transition is {self.current_state}")
            
            # update current state
            self.current_state = next_state
            # print(f"state post transition is {self.current_state}")
            

    def is_accepting(self):
        return self.current_state in self.dfa.final_states

    def state_label2array(self, state = None):
        """Maps state label to array representation eg [0, 1.0, 0]
        state - string label of automaton state
        obtains array rep of ucrrent stte if state = None
        """

        if not state:
            # current state is state evaluated post action
            state = self.current_state

        state_index = self.states_dict[state]

        # initialize auto state array to allow for input into neural network
        state_array = np.zeros(self.num_states, dtype=np.float32)

        # fill auto state array with right index
        state_array[state_index] = 1.0
        return state_array
    
    def update_potential_difference(self, 
                             current_state: str,
                             next_state: str) -> float:
        '''
        Computes potential difference between two automaton states
        '''
        
        if self.potential_dict:
            current_potential = self.potential_dict[current_state]
            next_potential    = self.potential_dict[next_state]
            self.delta_potential = self.gamma * next_potential - current_potential
        else:
            raise ValueError("No potential dictionary provided")
    
    
    def reset(self):
        """Reset auto to initial state and clear history"""
        self.current_state = self.dfa.initial_state
        self.history = []
        self.delta_potential = 0.0

class WFA_monitor:
    def __init__(self, WFA: WeightedAutomaton, delta_thresh: Optional[float] = 0.05, 
                 min_f_thresh: Optional[float] = 1e-30, 
                 word: Optional[ List[str] ] = None):
        """
        ADD PARAMETERS TO CHECK FOR WFA COMPLETION
        """
        # WFA parameters
        "monitors weighted finite automaton"
        self.WFA = WFA

        "convert WFA arrays to numpy arrays from sympy arrays"
        self.initial    = np.array( WFA.initial ).astype(np.float64)
        self.final      = np.array( WFA.final ).astype(np.float64)
        self.num_states = WFA.n
        self.current_state = self.initial
        # accessed by environment
        self.word = word 

        # Monitoring parameters
        self.history = []
        self.progress = False
        self.trans_quality = True 
        self.delta_thresh = delta_thresh
        self.min_f_thresh     = min_f_thresh
        self.f_decrease = False
        self.f_increase = False
        self.f = 1
        self.unstable_ind = False
        self.transitions = {}
        self.complete_ind = False 

        # convert transition matrices from sympy arrays to numpy arrays
        for key in WFA.transitions:
            self.transitions[key] = np.array( WFA.transitions[key] ).astype(np.float64)
        
    
    def step(self, event: Optional[str] = None):
        """
        step function for wfa
        ARGS
        event [str] - event observed in environment
        """
        self.f_increase = False
        self.f_decrease = False 
        if event:
            """
            check for invalid events
            """
            if event not in self.WFA.alphabet:
                print(f"Undefined event: {event}")
                raise ValueError(f"Invalid input event: {event}")

            next_state = self.current_state@self.transitions[event]
            
            self.measure_progress(next_state=next_state)
            self.current_state = next_state
            self.check_complete()
            self.instability_check()
            # print(f"state post transition is {self.current_state}")
            self.history.append((self.current_state, event, next_state))
    
    def instability_check(self):
        """
        checks if scoring function is "too close" to zero
        """
        self.f_value = self.current_state@self.final
        if self.f_value <= 0.1:
            self.trans_quality = False
            self.unstable_ind = np.isclose(self.f_value, 0, atol=self.min_f_thresh)
        else:
            self.trans_quality = True 

    def measure_progress(self, next_state):
        """
        Update indicators to determine if 
        the scoring function of the current path has changed significantly
        """
        # WFA scroing func of current state and next
        current_f = self.current_state@self.final
        next_f    = next_state@self.final

        # change in wfa scoring function 
        delta_f = current_f - next_f 

        if delta_f <= -0.01:
        # scoring function increases 
            self.f_increase = True 
        else: 
        # check if there was a big score decrease
            norm_delta_f = np.abs( delta_f/current_f )
            if  norm_delta_f > self.delta_thresh:
                "transition has been made"
                self.f_decrease = True
            else:
                self.f_decrease = False
                # print(f"progress is {self.progress}")
        # print(f"{self.current_state} --[{event}]--> {next_state}")
        # print(f"state before transition is {self.current_state}")

    def check_complete(self):
        if self.current_state[-1,-1] > 0.:
            self.complete_ind = True

    def reset(self):
        """Reset auto to initial state and clear history"""
        self.current_state = self.initial
        self.history = []
        self.complete_ind = False 

dfa_T1 = DFA(
    states={'q0', 'q1', 'q2', 'q3', 'q4', 'q5'},
    input_symbols={'pickup key', 'opened door', 
                   'dropped key', 'closed door', 
                   'pickup box', 'movement'},
    transitions={
        'q0':  {
                'pickup key': 'q1',        # Correct symbol → progress
                'opened door': 'q0',         # Wrong symbol → stay
                'dropped key': 'q0',
                'closed door': 'q0',
                'pickup box': 'q0', 
                'movement': 'q0'
                },
        'q1': {
                'pickup key': 'q1',        
                'opened door': 'q2',         # progress
                'dropped key': 'q1',
                'closed door': 'q1',
                'pickup box': 'q1', 
                'movement': 'q1'
                },
        'q2': {
                'pickup key': 'q2',        
                'opened door': 'q2',         
                'dropped key': 'q3',  #progress
                'closed door': 'q2',
                'pickup box': 'q2', 
                'movement': 'q2'
                },
        'q3': {
                'pickup key': 'q3',        # Correct symbol → progress
                'opened door': 'q3',         # Wrong symbol → stay
                'dropped key': 'q3',
                'closed door': 'q4',
                'pickup box': 'q3', 
                'movement': 'q3',
                },
        'q4': {
                'pickup key': 'q4',        # Correct symbol → progress
                'opened door': 'q4',         # Wrong symbol → stay
                'dropped key': 'q4',
                'closed door': 'q4',
                'pickup box': 'q5', 
                'movement': 'q4'
                },
        'q5': {
            'pickup key': 'q5',
            'opened door': 'q5',
            'dropped key': 'q5',
            'closed door': 'q5',
            'pickup box': 'q5', 
            'movement': 'q5'
        }
    },
    initial_state='q0',
    final_states={'q5'}
)

def create_UR5e_xyz_DFA(version: Optional[str] = 'strict', 
                        potential_delta: Optional[float] = 1.0) -> Tuple:
    '''
    Creates DFA for UR5e reaching task in xyz space where the arm must reach the desired z coordinate of the
    target state after reaching the desired x and y coordinates first.
    
    Inputs:
    potential_delta [float] - difference in potential function for automaton states
                                used for reward functions
    version [str] - 'strict' or 'lenient' strict requires reaching x,y,z in order, lenient allows reaching z after x or y
    
    Outputs:
    UR5e_DFA - DFA object from automata-lib representing the reaching task
    potential_dict [Dict] - dictionary mapping between automaton states and potential function values
    '''
    if version == 'strict':
        states = {'000', '100', '010', '110', '111', 'sink'}
        initial_state = '000'
        final_states = {'111'}
        input_symbols = {'delta_x', 'delta_y', 'delta_z', 'no_delta', 'invalid'}
        transitions = {
            '000': {'delta_x': '100', 'delta_y': '010', 'delta_z': 'sink', 'no_delta': '000', 'invalid': 'sink'},
            '100': {'delta_x': '000', 'delta_y': '110', 'delta_z': 'sink', 'no_delta': '100', 'invalid': 'sink'},
            '010': {'delta_x': '110', 'delta_y': '000', 'delta_z': 'sink', 'no_delta': '010', 'invalid': 'sink'},
            '110': {'delta_x': '010', 'delta_y': '100', 'delta_z': '111', 'no_delta': '110', 'invalid': 'sink'},
            '111': {'delta_x': '111', 'delta_y': '111', 'delta_z': '111',  'no_delta': '111', 'invalid': 'sink'},
            'sink': {'delta_x': 'sink', 'delta_y': 'sink', 'delta_z': 'sink', 'no_delta': 'sink', 'invalid': 'sink'}
        }
        
        potential_dict = {
            '000': 0,
            '100': potential_delta,
            '010': potential_delta,
            '110': potential_delta*2,
            '111': potential_delta*3,
            'sink': 0
                     }

    
    UR5e_DFA = DFA(
        states=states,
        initial_state=initial_state,
        final_states=final_states,
        input_symbols=input_symbols,
        transitions=transitions
    )

    return UR5e_DFA, potential_dict

def create_UR5e_traj_DFA(trajectory: List[ List[ int ] ], 
                         potential_delta: Optional[float] = 1.0) -> Tuple:
    '''
    Creates DFA for UR5e for following a given trajectory in grid space
    Args:
    trajectory: List[ List[ int ] ] - the trajectory to follow
    potential_delta: float - describes the difference in values 
                            for the potential function at each state
    Returns:
    UR5e_DFA - DFA object from automata-lib representing the trajectory following task
    potential_dict - dictionary mapping between automaton states and potential function values
    alphabet_dict - dictionary mapping between input symbols and grid states
    '''
    num_configs = len( trajectory )
    
    # number of automaton states is number of configurations + 1 (final state)
    states = { str(i) for i in range(num_configs+1) }
    initial_state = '0'
    final_states = { str( num_configs ) }
    
    # reach_x means reaching the x index of trajectory
    input_symbols = { f"reach_{i}" for i in range(num_configs) }
    
    # dictionary to mapy input symbols to grid states
    alphabet_dict = { f"reach_{i}": trajectory[i] for i in range(num_configs) }
    
    # empty transition dictionary and potential function dictionary that is filled below
    transitions = {}
    potential_dict = {}
    
    for state in states: 
        # get the index of the current state
        state_index = int(state)
        
        # initialize empty dictionary for each state
        transitions[state] = {}
        
        # assign potential value for each state
        potential_dict[state] = potential_delta * state_index
        
        # unique symbol describing forward progress to the next state
        forward_progress_symbol = f"reach_{state_index}"
        
        # indicating that the ur5e has reached the next config
        forward_progress_symbol = f"reach_{state_index}"
        
        # fill dictionary with all input symbols
        for symbol in input_symbols:
            # check if symbol indicates forward progress
            if symbol == forward_progress_symbol:
                # add transition for progress to next state
                next_state = str( state_index + 1 )
                transitions[state][symbol] = next_state
            else:
                # no progress, stay in current state
                transitions[state][symbol] = state
    
    # create DFA object
    UR5e_DFA = DFA(
        states=states,
        initial_state=initial_state,
        final_states=final_states,
        input_symbols=input_symbols,
        transitions=transitions
    )

    return UR5e_DFA, potential_dict, alphabet_dict

def frac(a, b):
    return sp.Rational(a, b)

def simple_wfa() -> WeightedAutomaton:
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

def create_wfa_T1(f, s, u=0.8):
    """
    Function to create wfa representation of task 1
    INPUTS 
    f - weight assigned for forward progress
    s - weight assigned for stationary progres
    u - weight assigned for useless actions e.g. pickup an item when nothing is in front
    OUTPUTS
    wfa_T1 - WFA object representing task 1
    """
    num_letters = 7
    num_states  = 6

    initial_array = sp.Matrix([[1, 0, 0, 0, 0, 0]])
    final_array   = sp.Matrix.ones(6,1)
    transition_dictionary = {}
    
    keys = {}
    symbols = ['pickup key', 'opened door', 
                'dropped key', 'closed door', 
                'pickup box', 'movement', 'useless']
    for index in np.arange(num_letters-2):
        progress_matrix   = sp.Matrix.zeros(num_states, num_states)
        progress_matrix[index, index+1] = f

        adjustment_matrix = sp.Matrix.zeros(num_states, num_states)
        adjustment_matrix[index,index] = s

        transition_matrix = s*sp.eye(num_states) + progress_matrix - adjustment_matrix
        symbol = symbols[index]
        transition_dictionary[symbol] = transition_matrix
    transition_dictionary["movement"] = sp.Matrix.eye(num_states)
    transition_dictionary['useless']  = u*sp.Matrix.eye(num_states)

    wfa_T1 = WeightedAutomaton(n=num_states,alphabet=symbols, 
                                initial=initial_array, 
                                transitions=transition_dictionary, 
                                final=final_array)
    return wfa_T1

def create_wfa_T1a(f, s, u=0.8):
    """
    Function to create wfa representation of task 1 
    with movement removed from alphabet
    INPUTS 
    f - weight assigned for forward progress
    s - weight assigned for stationary progres
    u - weight assigned for useless actions e.g. pickup an item when nothing is in front
    OUTPUTS
    wfa_T1a - WFA object representing task 1
    """
    num_letters = 6
    num_states  = 6

    initial_array = sp.Matrix([[1, 0, 0, 0, 0, 0]])
    final_array   = sp.Matrix.ones(6,1)
    transition_dictionary = {}
    
    symbols = ['pickup key', 'opened door', 
                'dropped key', 'closed door', 
                'pickup box', 'useless']
    for index in np.arange(num_letters-1):
        progress_matrix   = sp.Matrix.zeros(num_states, num_states)
        progress_matrix[index, index+1] = f

        adjustment_matrix = sp.Matrix.zeros(num_states, num_states)
        adjustment_matrix[index,index] = s

        transition_matrix = s*sp.eye(num_states) + progress_matrix - adjustment_matrix
        symbol = symbols[index]
        transition_dictionary[symbol] = transition_matrix
    transition_dictionary['useless']  = u*sp.Matrix.eye(num_states)

    wfa_T1a = WeightedAutomaton(n=num_states,alphabet=symbols, 
                                initial=initial_array, 
                                transitions=transition_dictionary, 
                                final=final_array)
    return wfa_T1a

def spwfa2WFA(wfa, alphabet=None) -> WeightedAutomaton:
    """
    Maps splearn wfa to WFA object defined in WFA_package that is much easier to work with
    IMPORTANT !!!
    wfa and alphabet must be in accordance
    ASSUME the ith transition array in the list provided by wfa.transitions is 
    the transition array for the ith letter in alphabet list provided as input
    INPUTS
    wfa - automaton object from splearn
    alphabet - ordered list of letters for wfa
    OUTPUTS
    WFA - wfa object from WFA_package
    """

    trans_list  = wfa.transitions
    num_letters = len( trans_list )

    if alphabet is None:
        alphabet = np.arange(num_letters)

    # call for arrays from splearn wfa
    num_states  = wfa.initial.shape[0]
    initial     = sp.Matrix( wfa.initial.reshape( (1,num_states) ) )
    final       = sp.Matrix( wfa.final.reshape( (num_states, 1) ) )
    trans_list  = wfa.transitions
    transitions = {}

    for letter_index in np.arange(num_letters):
        transitions[ alphabet[letter_index] ] = sp.Matrix( trans_list[letter_index] )

    WFA = WeightedAutomaton(n=num_states, alphabet=alphabet,
                            initial=initial, final=final, 
                            transitions=transitions)

    return WFA

if __name__ == "__main__":

    ur5e_DFA = create_UR5e_xyz_DFA(version='strict')

    print(ur5e_DFA)