#include "inc/Graphics.hpp"

Graphics::Graphics(void) { // Base class default contructor

}

Graphics::Graphics(int windowWidth, int windowHeight) { // Base class contructor
	SDL_Init(SDL_INIT_EVERYTHING); // SDL initialization

	window = SDL_CreateWindow("Drone Application", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, windowWidth, windowHeight, SDL_WINDOW_SHOWN); // Creating a window object
	if (!window) { // Check if the window object was corretly initialized
		throw "Could not initialize window object";
	}

	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC); // Creating a renderer object
	if (!renderer) { // Check if the renderer object was corretly initialized
		throw "Could not initialize renderer object";
	}

	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // Setting a draw color to white
	SDL_RenderClear(renderer); // Clear the renderer

	this->windowWidth = windowWidth;
	this->windowHeight = windowHeight;
}

Graphics::~Graphics(void) { // Base class desctructor
	SDL_DestroyWindow(window);
	SDL_DestroyRenderer(renderer);
	SDL_Quit();
}

DroneGraphics::DroneGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath) : Graphics(windowWidth, windowHeight) { // Derived class contructor
	droneImage = SDL_LoadBMP(droneBitmapPath); // Loading the drone image
	if (droneImage == NULL) { // Check if the drone image was loaded correctly
		throw "Could not load drone bitmap image";
	}

	droneTexture = SDL_CreateTextureFromSurface(Graphics::renderer, droneImage); // Creating a texture object
	if (!droneTexture) { // Check if the texture object was initialized correctly
		throw "Could not initialize drone texture object";
	}

	widthOffset = windowWidth / 2 - droneImage->w / 2; // Computing the x-offset value so the drone is shown in the middle
	heightOffset = windowHeight / 2 - droneImage->h / 2; // Computing the y-offset value so the drone is shown in the middle

	dronePosition.h = droneImage->h; // Drone rectangle height is set to the drone image height
	dronePosition.w = droneImage->w; // Drone rectangle width is set to the drone image width
}

DroneGraphics::~DroneGraphics(void) { // Derived class desctructor
	SDL_FreeSurface(droneImage);
	SDL_DestroyTexture(droneTexture);
}

DroneCargoGraphics::DroneCargoGraphics(int windowWidth, int windowHeight, const char* droneBitmapPath, const char* cargoBitmapPath) : DroneGraphics(windowWidth, windowHeight, droneBitmapPath) { // Derived class contructor
	cargoImage = SDL_LoadBMP(cargoBitmapPath); // Loading the cargo image
	if (cargoImage == NULL) { // Check if the cargo image was loaded correctly
		throw "Could not load cargo bitmap image";
	}

	cargoTexture = SDL_CreateTextureFromSurface(Graphics::renderer, cargoImage); // Creating a texture object
	if (!cargoTexture) { // Check if the texture object was initialized correctly
		throw "Could not initialize cargo texture object";
	}

	widthOffset = windowWidth / 2 - cargoImage->w / 2; // Computing the x-offset value so the drone is shown in the middle
	heightOffset = windowHeight / 2 - cargoImage->h / 2; // Computing the y-offset value so the drone is shown in the middle

	cargoPosition.h = cargoImage->h; // Cargo rectangle height is set to the drone image height
	cargoPosition.w = cargoImage->w; // Cargo rectangle width is set to the drone image width
}

DroneCargoGraphics::~DroneCargoGraphics(void) { // Derived class desctructor
	SDL_FreeSurface(cargoImage);
	SDL_DestroyTexture(cargoTexture);
}

void DroneGraphics::draw(const std::unique_ptr<State>& state) { // Drawing a drone on the screen
	auto v = state->getState(); // Get the state vector
	const double theta = -(v["theta"] * 180.0) / M_PI; // Converting the drone angle from rad to deg

	int tempX = v["x"] * Graphics::scale + DroneGraphics::widthOffset; // Finding the drone x-location on the screen
	int tempY = -v["y"] * Graphics::scale + DroneGraphics::heightOffset; // Finding the drone y-location on the screen

	if (tempX > windowWidth || tempY > windowHeight || tempX < 0 || tempY < 0) { // Checking against screen boundaries: if outside limits then
		state->reset(); // Reset the state
		return;
	}

	dronePosition.x = tempX; // Set the drone x-position
	dronePosition.y = tempY; // Set the drone y-position

	SDL_RenderClear(Graphics::renderer); // Clear the renderer
	SDL_RenderCopyEx(Graphics::renderer, droneTexture, nullptr, &dronePosition, theta, NULL, SDL_FLIP_NONE); // Show the drone
	SDL_RenderPresent(Graphics::renderer); // Render
}

void DroneCargoGraphics::draw(const std::unique_ptr<State>& state) { // Drawing a drone with cargo on the screen
	auto v = state->getState(); // Get the state vector
	const double thetaD = -(v["thetaD"] * 180.0) / M_PI; // Converting the drone angle from rad to deg
	const float thetaC = -(v["thetaRope"] * 180.0) / M_PI;; // Converting the cargo (rope) angle from rad to deg

	int tempXD = v["xD"] * Graphics::scale + DroneGraphics::widthOffset; // Finding the drone x-location on the screen
	int tempYD = -v["yD"] * Graphics::scale + DroneGraphics::heightOffset; // Finding the drone y-location on the screen
	int tempXC = v["xC"] * Graphics::scale + DroneCargoGraphics::widthOffset; // Finding the cargo x-location on the screen
	int tempYC = -v["yC"] * Graphics::scale + DroneCargoGraphics::heightOffset; // Finding the cargo y-location on the screen
 
	if (tempXD > windowWidth || tempYD > windowHeight || tempXD < 0 || tempYD < 0) { // Checking against screen boundaries: if outside limits then
		state->reset(); // Reset the state
		return;
	}

	dronePosition.x = tempXD; // Set the drone x-position
	dronePosition.y = tempYD; // Set the drone y-position
	cargoPosition.x = tempXC; // Set the cargo x-position
	cargoPosition.y = tempYC; // Set the cargo y-position

	SDL_RenderClear(Graphics::renderer); // Clear the renderer
	SDL_SetRenderDrawColor(Graphics::renderer, 0, 0, 0, 255); // Set the draw color to black
	SDL_RenderDrawLine(Graphics::renderer, dronePosition.x + dronePosition.w / 2, dronePosition.y, cargoPosition.x + cargoPosition.w / 2, cargoPosition.y); // Draw the rope
	SDL_SetRenderDrawColor(Graphics::renderer, 255, 255, 255, 255); // Set the draw color to white
	SDL_RenderCopyEx(Graphics::renderer, droneTexture, nullptr, &dronePosition, thetaD, NULL, SDL_FLIP_NONE); // Draw the drone
	SDL_RenderCopyEx(Graphics::renderer, cargoTexture, nullptr, &cargoPosition, thetaC, NULL, SDL_FLIP_NONE); // Draw the cargo
	SDL_RenderPresent(Graphics::renderer); // Render
}
