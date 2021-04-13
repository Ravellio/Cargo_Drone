/*
* Filename: Drone_Cargo_Simulation_Graphics_CSV
* Authors: Vladislavs Svarnovics (s2366010), Amina Mokhtar (s2492695)
* Version: 0.1
* License: N/A
* Description: The program ask the user to choose what dynamical system has to be simulated.
*              The user can choose between "Drone" or "Drone with Cargo" [Y/N]. Next to that,
*              the user can choose the integration method, either "Euler" or "Runge-Kutta" [Euler/RungeKutta].
*              Note that integration step "h" is chosen automatically depending on the integration method.
*              The program reads a "csv" file (via the provided "readPath") which indicates the input vector
               to the dynamical system. This input vector consists of the thrust "T [N]" and the rate of change 
               of the orientation "w [rad/s]". Next to the previous challenge, in this program, the drone 
               (with or without cargo) shown on the screen which means we are no longer writing to a "csv" file.
*/

#include "inc/Simulator.hpp"
#include "inc/Graphics.hpp"

int main() {
    const std::string readPath = "/home/vlad/Documents/Projects/inputs.csv"; // Reading the inputs from the csv file

    Simulator sim; // Creating a simulator object
    try { // Exceptions possible 
        sim = Simulator(readPath);
    }
    catch (const char* e) {
        std::cout << "Wrong read path name: " << e << "\n";
        return -1;
    }

    std::string inp1, inp2; // Allocating mememory for input arguments
    do { // Drone with or without cargo
        std::cout << "Drone with cargo: Y/N" << "\n";
        std::cin >> inp1;
    } while (!(inp1 == "Y" || inp1 == "N")); // Only "Y" <yes> or "N" <no> inputs are possible

    do { // Choosing a simulator
        std::cout << "Simulator: Euler/RungeKutta" << "\n";
        std::cin >> inp2;
    } while (!(inp2 == "Euler" || inp2 == "RungeKutta")); // Only "Euler" or "RungeKutta" inputs are possible

    const int windowWidth = 800; 
    const int windowHeight = 600;
    const char* droneBitmapPath = "/home/vlad/Documents/Projects/Drone.bmp"; // Path for drone image
    const char* cargoBitmapPath = "/home/vlad/Documents/Projects/Cargo.bmp"; // Path for cargo image

    std::unique_ptr<State> state; // Declaring a state object
    std::unique_ptr<Graphics> g; // Declaring a graphics object
    float h; // Time step

    try { // Exceptions possible 
        if (inp1 == "N") { // Only drone, no cargo
            state = std::make_unique<DroneState>();
            g = std::make_unique<DroneGraphics>(windowWidth, windowHeight, droneBitmapPath);
        }
        else { // Drone with cargo
            state = std::make_unique<DroneCargoState>();
            g = std::make_unique<DroneCargoGraphics>(windowWidth, windowHeight, droneBitmapPath, cargoBitmapPath);
        }
    }
    catch (const char* e) {
        std::cout << "Could create graphics object: " << e << "\n";
        return -1;
    }
	
	void (Simulator:: * nextStateFunc) (const std::unique_ptr<State>&, float, float); // Function pointer for computing the next state
   
    if (inp2 == "Euler") { // Choosing the appropriate time step
		nextStateFunc = &Simulator::euler;
        if (inp1 == "N") {
            h = 0.01f;
        }
        else {
            h = 0.0005f;
        }
    }
    else {
		nextStateFunc = &Simulator::rungeKutta;
        h = 0.01f;
    }
    
    const int fps = 20; // Frames per second
    int timeout = SDL_GetTicks() + 1000 / fps;
    float t = 0.0f;

    SDL_Event e; // Memory for storing the event
    bool quit = false;
    while (!quit) {
        g->draw(state); // Draw the state
        (sim.*nextStateFunc)(state, t, h); // Compute the next state
        
        t += h; // Increment the time

        while (!SDL_TICKS_PASSED(SDL_GetTicks(), timeout)) { // Wait for the time elapse
            while (SDL_PollEvent(&e)) { // Check for any events
                if (e.type == SDL_QUIT) { // If quit is pressed
                    quit = true;
                }
            }
        }
        timeout += 1000 / fps;
    }
   
    return 0;
}
