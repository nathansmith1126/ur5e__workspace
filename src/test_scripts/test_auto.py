from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor


ur5e_DFA, potentials = create_UR5e_xyz_DFA()

dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

word = ["delta_y", "delta_y", "delta_z"]

for letter in word:
    current_state = dfa_monitor.current_state
    dfa_monitor.step(letter)
    next_state = dfa_monitor.current_state
    print(f"Current State: {current_state}, Next State: {next_state}, Potential Difference: {dfa_monitor.delta_potential}")