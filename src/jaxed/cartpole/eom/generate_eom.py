from eom import save_equations_of_motions, calculate_equations_of_motions

def generate_eom() -> None:
    max_n_poles = 2
    print("Generating equations of motion files...")
    for n_poles in range(1, max_n_poles+1):
        sols, pure_sols = calculate_equations_of_motions(n_poles)

        print("Saving equations of motion...")
        save_equations_of_motions(n_poles, pure_sols)

if __name__ == "__main__":
    generate_eom()