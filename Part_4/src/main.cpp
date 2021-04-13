/*
* Filename: Drone_Cargo_Simulation_Graphics_Keyboard
* Authors: Vladislavs Svarnovics (s2366010), Amina Mokhtar (s2492695)
* Version: 0.1
* License: N/A
* Description: The program asks the user to choose what dynamical system has to be simulated.
*              The user can choose between "Drone" or "Drone with Cargo". Next to that,
*              the user can choose the integration method, either "Euler" or "Runge-Kutta".
*              Note that integration step "h" is chosen automatically depending on the integration method.
*              The program is processing user key for drone movement. Key "w" correspons to up (enable trust),
*              "s" - going down (disable trust), "a" - left and "d" right. No writing or reading to "csv" file is performed.
*/

#include <thread>
#include <mutex>
#include <chrono>
#include <functional>

#include "inc/Simulator.hpp"
#include "inc/Graphics.hpp"

// Add a more elaborate description about input parameters
// Check if the description matches the program requirements
// Check if we need to rotate the cargo (change the angle accoding to the rope angle) - or it is done in the bonus question?
// Increare the rope size
// Mutex object not in scope -> fix this

std::mutex mtx;
bool quit = false;

std::unique_ptr<State> state;
std::vector<float> u = { 0.0f, 0.0f };

void nextStateSimulation(Simulator sim, void (Simulator::* f) (const std::unique_ptr<State>&, const std::vector<float>&, float h), int sleepTimeMs, float h) {
    while (!quit) {
        (sim.*f)(state, u, h);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMs));
    }
}

int main() {
    Simulator sim;

    std::string inp1, inp2;
    do {
        std::cout << "Drone with cargo: Y/N" << "\n";
        std::cin >> inp1;
    } while (!(inp1 == "Y" || inp1 == "N"));

    do {
        std::cout << "Simulator: Euler/RungeKutta" << "\n";
        std::cin >> inp2;
    } while (!(inp2 == "Euler" || inp2 == "RungeKutta"));

    const int windowWidth = 1300;
    const int windowHeight = 700;
    const char* droneBitmapPath = "/home/vlad/Documents/Projects/Drone.bmp";
    const char* cargoBitmapPath = "/home/vlad/Documents/Projects/Cargo.bmp";

    std::unique_ptr<Graphics> g;
    const float w = 0.7f;
    const float downT = 0.0f;
    float upT;
    float h;
   
    try {
        if (inp1 == "N") {
            state = std::make_unique<DroneState>();
            g = std::make_unique<DroneGraphics>(windowWidth, windowHeight, droneBitmapPath);
            upT = 2 * 3 * 9.81f;
        }
        else {
            state = std::make_unique<DroneCargoState>();
            g = std::make_unique<DroneCargoGraphics>(windowWidth, windowHeight, droneBitmapPath, cargoBitmapPath);
            upT = 2 * (3 + 2) * 9.81f;
        }
    }
    catch (const char* e) {
        std::cout << "Could create graphics object: " << e << "\n";
        return -1;
    }

    void (Simulator:: * nextStateFunc) (const std::unique_ptr<State>&, const std::vector<float>&, float);
   
    if (inp2 == "Euler") {
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
    
    const int fps = 30; 
    const int threadSleepTimeMs = 20;
    int timeout = SDL_GetTicks() + 1000 / fps;

    std::thread th(nextStateSimulation, sim, nextStateFunc, threadSleepTimeMs, h);
    
    SDL_Event e;

    while (!quit) {
        g->draw(state);
  
        while (!SDL_TICKS_PASSED(SDL_GetTicks(), timeout)) {
            while (SDL_PollEvent(&e)) {
                switch (e.type) {
                case SDL_QUIT:
                    quit = true;   
                    th.join();
                    break;
                case SDL_KEYDOWN:
                    switch (e.key.keysym.sym) {
                    case SDLK_w:
                        u[0] = upT;
                        break;
                    case SDLK_a:
                        u[1] = w;
                        break;
                    case SDLK_d:
                        u[1] = -w;
                        break;
                    case SDLK_s:
                        u[0] = downT;
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