#include "inc/Graphics.hpp"

Graphics::Graphics(void) {

}

Graphics::Graphics(int windowWidth, int windowHeight) {
	SDL_Init(SDL_INIT_EVERYTHING);

	window = SDL_CreateWindow("Drone Application", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, windowWidth, windowHeight, SDL_WINDOW_SHOWN);
	if (!window) {
		throw "Could not initialize window object";
	}

	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (!renderer) {
		throw "Could not initialize renderer object";
	}

	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	SDL_RenderClear(renderer);

	this->windowWidth = windowWidth;
	this->windowHeight = windowHeight;
}

Graphics::~Graphics(void) {
	SDL_DestroyWindow(window);
	SDL_DestroyRenderer(renderer);
	SDL_Quit();
}

DroneGraphics::DroneGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath) : Graphics(windowWidth, windowHeight) {
	droneImage = SDL_LoadBMP(droneBitmapPath);
	if (droneImage == NULL) {
		throw "Could not load drone bitmap image";
	}

	droneTexture = SDL_CreateTextureFromSurface(Graphics::renderer, droneImage);
	if (!droneTexture) {
		throw "Could not initialize drone texture object";
	}

	widthOffset = windowWidth / 2 - droneImage->w / 2;
	heightOffset = windowHeight / 2 - droneImage->h / 2;

	dronePosition.h = droneImage->h;
	dronePosition.w = droneImage->w;
}

DroneGraphics::~DroneGraphics(void) {
	SDL_FreeSurface(droneImage);
	SDL_DestroyTexture(droneTexture);
}

DroneCargoGraphics::DroneCargoGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath, const char* cargoBitmapPath) : DroneGraphics(windowWidth, windowHeight, droneBitmapPath) {
	cargoImage = SDL_LoadBMP(cargoBitmapPath);
	if (cargoImage == NULL) {
		throw "Could not load cargo bitmap image";
	}

	cargoTexture = SDL_CreateTextureFromSurface(Graphics::renderer, cargoImage);
	if (!cargoTexture) {
		throw "Could not initialize cargo texture object";
	}

	widthOffset = windowWidth / 2 - cargoImage->w / 2;
	heightOffset = windowHeight / 2 - cargoImage->h / 2;

	cargoPosition.h = cargoImage->h;
	cargoPosition.w = cargoImage->w;
}

DroneCargoGraphics::~DroneCargoGraphics(void) {
	SDL_FreeSurface(cargoImage);
	SDL_DestroyTexture(cargoTexture);
}

void DroneGraphics::draw(const std::unique_ptr<State>& state) {
	auto v = state->getState();
	const double theta = -(v["theta"] * 180.0) / M_PI;

	int tempX = v["x"] * Graphics::scale + DroneGraphics::widthOffset;
	int tempY = -v["y"] * Graphics::scale + DroneGraphics::heightOffset;

	if (tempX > windowWidth || tempY > windowHeight || tempX < 0 || tempY < 0) {
		state->reset();
		return;
	}

	dronePosition.x = tempX;
	dronePosition.y = tempY;

	SDL_RenderClear(Graphics::renderer);
	SDL_RenderCopyEx(Graphics::renderer, droneTexture, nullptr, &dronePosition, theta, NULL, SDL_FLIP_NONE);
	SDL_RenderPresent(Graphics::renderer);
}

void DroneCargoGraphics::draw(const std::unique_ptr<State>& state) {
	auto v = state->getState();
	const double thetaD = -(v["thetaD"] * 180.0) / M_PI;
	const float thetaC = 0.0f;

	int tempXD = v["xD"] * Graphics::scale + DroneGraphics::widthOffset;
	int tempYD = -v["yD"] * Graphics::scale + DroneGraphics::heightOffset;
	int tempXC = v["xC"] * Graphics::scale + DroneCargoGraphics::widthOffset;
	int tempYC = -v["yC"] * Graphics::scale + DroneCargoGraphics::heightOffset;

	if (tempXD > windowWidth || tempYD > windowHeight || tempXD < 0 || tempYD < 0) {
		state->reset();
		return;
	}

	dronePosition.x = tempXD;
	dronePosition.y = tempYD;
	cargoPosition.x = tempXC;
	cargoPosition.y = tempYC;

	SDL_RenderClear(Graphics::renderer);
	SDL_SetRenderDrawColor(Graphics::renderer, 0, 0, 0, 255);
	SDL_RenderDrawLine(Graphics::renderer, dronePosition.x + dronePosition.w / 2, dronePosition.y, cargoPosition.x + cargoPosition.w / 2, cargoPosition.y);
	SDL_SetRenderDrawColor(Graphics::renderer, 255, 255, 255, 255);
	SDL_RenderCopyEx(Graphics::renderer, droneTexture, nullptr, &dronePosition, thetaD, NULL, SDL_FLIP_NONE);
	SDL_RenderCopyEx(Graphics::renderer, cargoTexture, nullptr, &cargoPosition, thetaC, NULL, SDL_FLIP_NONE);
	SDL_RenderPresent(Graphics::renderer);
}
