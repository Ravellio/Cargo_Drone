#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <math.h>

#include "SDL2/SDL.h"
#undef main

#include "Dynamics.hpp"

class Graphics { // Base class
private:

protected:
	const int scale = 50; // Scaling the screen and real drone size. No real formula used.
	int windowWidth;
	int windowHeight;

	SDL_Window* window = nullptr;
	SDL_Renderer* renderer = nullptr;

public:
	Graphics(void); // Default contructor
	Graphics(int windowWidth, int windowHeight); // Constructor
	virtual ~Graphics(void); // Virtual destructor

	virtual void draw(const std::unique_ptr<State>& state) = 0; // Drawing a drone on the screen given the state vector
};

class DroneGraphics : public Graphics { // Derived class
private:

protected:
	int widthOffset; // x-Ofset to place to drone in the middle 
	int heightOffset; // y-Ofset to place to drone in the middle 

	SDL_Surface* droneImage = nullptr; 
	SDL_Texture* droneTexture = nullptr; 
	SDL_Rect dronePosition; // Drone position is held by a rectnagle object

public:
	DroneGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath); // Contructor
	virtual ~DroneGraphics(void); // Virtual desctructor

	virtual void draw(const std::unique_ptr<State>& state) override; // Drawing a drone on the screen given the state vector
};

class DroneCargoGraphics : public DroneGraphics { // Derived class
private:
	int widthOffset; // x-Ofset to place to drone in the middle 
	int heightOffset; // y-Ofset to place to drone in the middle 

	SDL_Surface* cargoImage = nullptr;
	SDL_Texture* cargoTexture = nullptr;
	SDL_Rect cargoPosition;

public:
	DroneCargoGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath, const char* cargoBitmapPath); // Contructor
	virtual ~DroneCargoGraphics(void); // Virtual desctructor

	virtual void draw(const std::unique_ptr<State>& state) override; // Drawing a drone on the screen given the state vector
};

#endif
