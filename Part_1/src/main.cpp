/*
* Filename: Drone_Cargo_Simulation_CSV
* Authors: Vladislavs Svarnovics (s2366010), Amina Mokhtar (s2492695)
* Version: 0.1
* License: N/A
* Description: The program ask the user to choose what dynamical system has to be simulated.
*              The user can choose between "Drone" or "Drone with Cargo" [Y/N]. Next to that,
*              the user can choose the integration method, either "Euler" or "Runge-Kutta" [Euler/RungeKutta].
*              Note that integration step "h" is chosen automatically depending on the integration method.
*              The program reads a "csv" file (via the provided "readPath") which indicates the input vector
               to the dynamical system. This input vector consists of the thrust "T [N]" and the rate of change 
               of the orientation "w [rad/s]". Intermiated simulations results are written to another "csv" file
               (to the provided "writePath").
*/

#include "inc/Simulator.hpp"

int main() {
    std::string readPath = "/home/vlad/Documents/Projects/inputs.csv"; // Reading inputs from a csv file
    std::string writePath = "/home/vlad/Documents/Projects/outputs.csv"; // Wriing outputs to another csv file

    try { // Exceptions possible
        Simulator sim(readPath, writePath); // Crearing a simulator
        std::string inp1, inp2; // Memory allocation for input arguments
        do { // Drone with or without cargo
            std::cout << "Drone with cargo: Y/N" << "\n";
            std::cin >> inp1;
        } while (!(inp1 == "Y" || inp1 == "N")); // Only "Y" <yes> or "N" <no> inputs are possible

        do { // Choosing a simulator
            std::cout << "Simulator: Euler/RungeKutta" << "\n";
            std::cin >> inp2;
        } while (!(inp2 == "Euler" || inp2 == "RungeKutta")); // Only "Euler" or "RungeKutta" inputs are possible

        std::unique_ptr<State> state; // Defining a pointer to a state
        float h; // Time step

        const float tStart = 0; // Begin the simulation at 0 time units
        const float tEnd = 5; // End the simulation after 5 time units (value chosen randomly)

        if (inp1 == "N") { 
            state = std::make_unique<DroneState>(); // Create a simple drone state
        }
        else {
            state = std::make_unique<DroneCargoState>(); // Create a drone state with cargo
        }

        if (inp2 == "Euler") { // Choosing a time step depending on the chosen simulator and the state
            if (inp1 == "N") {
                h = 0.01; 
            }
            else {
                h = 0.0005;
            }
            sim.euler(state, tStart, tEnd, h);
        }
        else {
            h = 0.01;
            sim.rungeKutta(state, tStart, tEnd, h);
        }
    }
    catch (const char* e) { // Printing an exception
        std::cout << "Wrong read path name: " << e << "\n";
    }
    
    return 0;
}
