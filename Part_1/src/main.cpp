/*
* Filename: Drone_Cargo_Simulation_CSV
* Authors: Vladislavs Svarnovics (s2366010), Amina Mokhtar (s2492695)
* Version: 0.1
* License: N/A
* Description: The program ask the user to choose what dynamical system has to be simulated.
*              The user can choose between "Drone" or "Drone with Cargo". Next to that,
*              the user can choose the integration method, either "Euler" or "Runge-Kutta".
*              Note that integration step "h" is chosen automatically depending on the integration method.
*              The program reads a "csv" file (via the provided "readPath") which indicates the input vector
               to the dynamical system. This input vector consists of the thrust "T [N]" and the rate of change 
               of the orientation "w [rad/s]". Intermiated simulations results are written to another "csv" file
               (to the provided "writePath").
*/

#include "inc/Simulator.hpp"

int main() {
    std::string readPath = "/home/vlad/Documents/Projects/inputs.csv";
    std::string writePath = "/home/vlad/Documents/Projects/outputs.csv";

    try {
        Simulator sim(readPath, writePath);
        std::string inp1, inp2;
        do {
            std::cout << "Drone with cargo: Y/N" << "\n";
            std::cin >> inp1;
        } while (!(inp1 == "Y" || inp1 == "N"));

        do {
            std::cout << "Simulator: Euler/RungeKutta" << "\n";
            std::cin >> inp2;
        } while (!(inp2 == "Euler" || inp2 == "RungeKutta"));

        std::unique_ptr<State> state;
        float h;

        const float tStart = 0;
        const float tEnd = 5;

        if (inp1 == "N") {
            state = std::make_unique<DroneState>();
        }
        else {
            state = std::make_unique<DroneCargoState>();
        }

        if (inp2 == "Euler") {
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
    catch (const char* e) {
        std::cout << "Wrong read path name: " << e << "\n";
    }
    
    return 0;
}
