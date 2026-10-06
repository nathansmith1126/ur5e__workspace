from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor, create_UR5e_traj_DFA

trajectory = [
              [0, 2, 4],
              [-1, -1, 4]
                      ] 
# ur5e_DFA, potentials = create_UR5e_xyz_DFA()

ur5e_DFA, potentials, alphabet_dict = create_UR5e_traj_DFA(trajectory)

print(f"DFA transition dictionary: {ur5e_DFA.transitions}")
print(f"Alphabet dictionary: {alphabet_dict}")

dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

# word = ["delta_y", "delta_y", "delta_z"]

word = ['reach_1',  'reach_1', 'reach_0', 'reach_1']

for letter in word:
    current_state = dfa_monitor.current_state
    dfa_monitor.step(letter)
    next_state = dfa_monitor.current_state
    print(f"Current State: {current_state}, Next State: {next_state}, Potential Difference: {dfa_monitor.delta_potential}")