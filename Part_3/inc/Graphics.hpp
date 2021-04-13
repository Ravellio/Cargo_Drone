#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <math.h>

#include <SDL2/SDL.h>
#undef main

#include "inc/Dynamics.hpp"

class Graphics {
private:

protected:
	const int scale = 50;
	int windowWidth;
	int windowHeight;

	SDL_Window* window = nullptr;
	SDL_Renderer* renderer = nullptr;

public:
	Graphics(void);
	Graphics(int windowWidth, int windowHeight);
	virtual ~Graphics(void);

	virtual void draw(const std::unique_ptr<State>& state) = 0;
};

class DroneGraphics : public Graphics {
private:
	
protected:
	int widthOffset;
	int heightOffset;

	SDL_Surface* droneImage = nullptr;
	SDL_Texture* droneTexture = nullptr;
	SDL_Rect dronePosition;

public:
	DroneGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath);
	virtual ~DroneGraphics(void);

	virtual void draw(const std::unique_ptr<State>& state) override;
};

class DroneCargoGraphics : public DroneGraphics {
private:
	int widthOffset;
	int heightOffset;

	SDL_Surface* cargoImage = nullptr;
	SDL_Texture* cargoTexture = nullptr;
	SDL_Rect cargoPosition;

public:
	DroneCargoGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath, const char* cargoBitmapPath);
	virtual ~DroneCargoGraphics(void);

	virtual void draw(const std::unique_ptr<State>& state) override;
};

#endif
