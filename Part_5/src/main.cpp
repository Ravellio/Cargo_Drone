/*
* Filename: Drone_Cargo_Simulation_Graphics_Keyboard
* Authors: Vladislavs Svarnovics (s2366010), Amina Mokhtar (s2492695)
* Version: 0.1
* License: N/A
* Description: The program asks the user to choose what dynamical system has to be simulated.
*              The user can choose between "Drone" or "Drone with Cargo" [Y/N]. Next to that,
*              the user can choose the integration method, either "Euler" or "Runge-Kutta" [Euler/RungeKutta].
*              Note that integration step "h" is chosen automatically depending on the integration method.
*              The program is processing user key for drone movement. Key "w" correspons to up (enable trust),
*              "s" - going down (disable trust), "a" - left and "d" right. No writing or reading to "csv" file is performed.
* 			   In the program multithreading is used.
*/

#include <thread>
#include <mutex>
#include <chrono>
#include <functional>

#include "inc/Simulator.hpp"
#include "inc/Graphics.hpp"

// Mutex object not in scope -> fix this

std::mutex mtx; // Mutex object
bool quit = false; // Flag for quiting the game

std::unique_ptr<State> state; // Dynamic state
std::vector<float> u = { 0.0f, 0.0f }; // Input vector

// Next state computation function : thread function
void nextStateSimulation(Simulator sim, void (Simulator::* f) (const std::unique_ptr<State>&, const std::vector<float>&, float h), int sleepTimeMs, float h) {
    while (!quit) { 
		{ // We need to scope the guard object. It is required to create an extra scope
			std::lock_guard<std::mutex> guard(mtx); // Guard is a convenient way to engage and dis-engage the lock
			(sim.*f)(state, u, h); // Compute the next state using a function pointer
		}
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMs)); // Sleep
    }
}

int main() {
    Simulator sim; // Simulator object

    std::string inp1, inp2; // Allocating mememory for input arguments
    do { // Drone with or without cargo
        std::cout << "Drone with cargo: Y/N" << "\n";
        std::cin >> inp1;
    } while (!(inp1 == "Y" || inp1 == "N")); // Only "Y" <yes> or "N" <no> inputs are possible

    do { // Choosing a simulator
        std::cout << "Simulator: Euler/RungeKutta" << "\n";
        std::cin >> inp2;
    } while (!(inp2 == "Euler" || inp2 == "RungeKutta")); // Only "Euler" or "RungeKutta" inputs are possible

    const int windowWidth = 1300;
    const int windowHeight = 700;
    const char* droneBitmapPath = "/home/vlad/Documents/Projects/Drone.bmp"; // Path for drone image
    const char* cargoBitmapPath = "/home/vlad/Documents/Projects/Cargo.bmp"; // Path for cargo image

    std::unique_ptr<Graphics> g; // Declaring a graphics object
    const float w = 0.7f; // Max rotation angle 
    const float downT = 0.0f; // Disabled thrust value
    float upT; // Enabled trust
    float h; // Time step
   
    try { // Exceptions possible 
        if (inp1 == "N") { // Only drone, no cargo
            state = std::make_unique<DroneState>();
            g = std::make_unique<DroneGraphics>(windowWidth, windowHeight, droneBitmapPath);
            upT = 2 * 3 * 9.81f; // Set the thrust value
        }
        else { // Drone with cargo
            state = std::make_unique<DroneCargoState>();
            g = std::make_unique<DroneCargoGraphics>(windowWidth, windowHeight, droneBitmapPath, cargoBitmapPath);
            upT = 2 * (3 + 2) * 9.81f; // Set the thrust value
        }
    }
    catch (const char* e) {
        std::cout << "Could create graphics object: " << e << "\n";
        return -1;
    }

    void (Simulator:: * nextStateFunc) (const std::unique_ptr<State>&, const std::vector<float>&, float); // Function pointer for computing the next state
   
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
        if (inp1 == "N") {
            h = 0.02f;
        }
        else {
            h = 0.01f;
        }
    }
    
    const int fps = 30; // Frames per second
    const int threadSleepTimeMs = 20; // Computational thread sleep time
    int timeout = SDL_GetTicks() + 1000 / fps;

    std::thread th(nextStateSimulation, sim, nextStateFunc, threadSleepTimeMs, h); // Initializing the thread
    
    SDL_Event e; // Memory for storing the event

    while (!quit) {
        g->draw(state); // Draw the state
  
        while (!SDL_TICKS_PASSED(SDL_GetTicks(), timeout)) { // Wait for the time elapse
            while (SDL_PollEvent(&e)) { // Check for any events
                switch (e.type) { // Check the event type
                case SDL_QUIT: // If quit
                    quit = true;
					th.join();
                    break;
                case SDL_KEYDOWN: // If keypress
                    switch (e.key.keysym.sym) { // Check what key was pressed
                        case SDLK_w: // 'w'
                            u[0] = upT; // Enable trust 
                            break;
                        case SDLK_a: // 'a'
                            u[1] = w; // Go left
                            break;
                        case SDLK_d: // 'd'
                            u[1] = -w; // Go right
                            break;
                        case SDLK_s: // 's'
                            u[0] = downT; // Disable thrust
                        default:
                            break;
                    }           
                default:
                    break;
                } 
            }
        }
        
        timeout += 1000 / fps;
    }
  
    return 0;
}