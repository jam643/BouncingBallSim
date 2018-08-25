/*
Simulates balls colliding elastically together and inelastically with walls
in uniform gravity field.

Left click to create inverse linear repulsion to mouse (2D).
Right click to create inverse linear attraction to mouse (2D).

author: Jesse Miller
date: 8/25/18
*/

#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <stdexcept>
#include <stdlib.h> 
#include <math.h>
#include <sstream>

using namespace std;

//size of square window in pixels
int WINDOW_SIZE = 600;

// abstract class representing a physical object with dynamics
class physical {
protected:
	const float G = 1; //accel of gravity [m/s^2]
	const int PIXEL_PER_METER = 6671; //pixel per meter of screen
	const int ZOOM = 30; // how far zoomed out
	const float SCALE = float(PIXEL_PER_METER / ZOOM); // scaling factor
public:
	// state [m] and [m/s]
	float x, y, xdot, ydot;
	// vector to store state
	sf::Vector2f z;
	sf::Vector2f zdot;

	virtual void draw(sf::RenderWindow& window) = 0;
};

//bouncing ball
class ball : public physical {
private:
	// coefficient of restitution of ball against walls
	const float coef_rest = 0.8;
public:
	float KE; // kinetic energy
	float radius;
	sf::CircleShape shape;
	bool collide; // true iff in colliding state with other ball
	static size_t count; // counts # of balls instantiated

	//constructor specifying state and size
	ball(float x, float y, float xdot, float ydot, float rad){
		z.x = x;
		z.y = y;
		zdot.x = xdot;
		zdot.y = ydot;
		radius = rad;
		shape.setRadius(radius);
		shape.setFillColor(sf::Color::White);
		shape.setOrigin(radius, radius);
		++count;
	}

	// constructor sets random pose/size
	ball(){	
		// random pose
		z.x = rand() % WINDOW_SIZE;
		z.y = rand() % WINDOW_SIZE;
		zdot.x = (float)rand() * 4.0 / RAND_MAX - 2.0;
		zdot.y = 0;

		// random radius
		radius = rand() % 20 + 5.0;
		shape.setRadius(radius);

		//shape.setFillColor(sf::Color(rand() % 255, rand() % 255, rand() % 255));
		shape.setFillColor(sf::Color::White);
		shape.setOrigin(radius, radius);

		++count;
	}

	// determine if two balls are colliding and updates their states
	static void collide_ball(ball& b1, ball& b2) {
		//distance between centers [m]
		float dist = sqrt(pow(b2.z.x - b1.z.x, 2) + pow(b2.z.y - b1.z.y, 2));
		//normalized vector from b1 to b2
		sf::Vector2f n_hat((b2.z.x - b1.z.x)/dist,(b2.z.y - b1.z.y)/dist);
		// overlap distance [m]
		float overlap = (b1.radius + b2.radius) - dist;

		if (overlap >= 0) {
			//move both overlapping balls away from each other half overlap distance
			b2.z.x += n_hat.x * overlap / 2;
			b2.z.y += n_hat.y * overlap / 2;
			b1.z.x -= n_hat.x * overlap / 2;
			b1.z.y -= n_hat.y * overlap / 2;

			// update velocity based on conservation of energy/momentum
			// https://en.wikipedia.org/wiki/Elastic_collision
			float x1dot = b1.zdot.x - (2 * b2.radius / (b1.radius + b2.radius))*
				((b1.zdot.x - b2.zdot.x)*(b1.z.x - b2.z.x) + (b1.zdot.y - b2.zdot.y)*(b1.z.y - b2.z.y))*
				(b1.z.x - b2.z.x) /
				(pow(b1.z.x - b2.z.x, 2) + pow(b1.z.y - b2.z.y, 2));
			float y1dot = b1.zdot.y - (2 * b2.radius / (b1.radius + b2.radius))*
				((b1.zdot.x - b2.zdot.x)*(b1.z.x - b2.z.x) + (b1.zdot.y - b2.zdot.y)*(b1.z.y - b2.z.y))*
				(b1.z.y - b2.z.y) /
				(pow(b1.z.x - b2.z.x, 2) + pow(b1.z.y - b2.z.y, 2));
			float x2dot = b2.zdot.x - (2 * b1.radius / (b2.radius + b1.radius))*
				((b2.zdot.x - b1.zdot.x)*(b2.z.x - b1.z.x) + (b2.zdot.y - b1.zdot.y)*(b2.z.y - b1.z.y))*
				(b2.z.x - b1.z.x) /
				(pow(b2.z.x - b1.z.x, 2) + pow(b2.z.y - b1.z.y, 2));
			float y2dot = b2.zdot.y - (2 * b1.radius / (b2.radius + b1.radius))*
				((b2.zdot.x - b1.zdot.x)*(b2.z.x - b1.z.x) + (b2.zdot.y - b1.zdot.y)*(b2.z.y - b1.z.y))*
				(b2.z.y - b1.z.y) /
				(pow(b2.z.x - b1.z.x, 2) + pow(b2.z.y - b1.z.y, 2));
			b1.zdot.x = x1dot;
			b1.zdot.y = y1dot;
			b2.zdot.x = x2dot;
			b2.zdot.y = y2dot;
		}
	}

	//inelastic collision with call
	void collide_wall() {
		if (z.x + radius > WINDOW_SIZE) {
			zdot.x = -coef_rest*zdot.x;
			z.x = WINDOW_SIZE - radius;
		}
		else if (z.x < radius) {
			zdot.x = -coef_rest*zdot.x;
			z.x = radius;
		}
		if (z.y + radius > WINDOW_SIZE) {
			zdot.y = -coef_rest*zdot.y;
			z.y = WINDOW_SIZE - radius;
		}
		else if (z.y < radius) {
			zdot.y = -coef_rest*zdot.y;
			z.y = radius;
		}
	}

	//update state using Euler integration
	void update(float& dt,sf::Vector2i m, sf::Mouse mouse) {
		
		// if left/right mouse click set inverse linear acceleration
		bool isLeft = mouse.isButtonPressed(sf::Mouse::Button::Left);
		bool isRight = mouse.isButtonPressed(sf::Mouse::Button::Right);
		float accel_mouse;
		if (isLeft || isRight){
			float d_to_mouse = sqrt(pow(z.x - m.x, 2) + pow(z.y - m.y, 2)) / SCALE;
			// inverse linear accel with small offset to avoid undefined near center
			accel_mouse = 1.0 / (pow(d_to_mouse + 0.1, 3));
		}
		else {
			accel_mouse = 0;
		}

		// attractive accel if right mouse
		if (isRight) {
			accel_mouse = -accel_mouse;
		}

		//euler integration of velocity
		zdot.y += (G + accel_mouse*(z.y - m.y)/SCALE)*dt;
		zdot.x += (accel_mouse*(z.x - m.x)/SCALE)*dt;

		//euler integration of pose
		z.x += zdot.x*dt*SCALE;
		z.y += zdot.y*dt*SCALE;

		// update kinetic energy
		KE = 0.5 * radius * (pow(zdot.x, 2) + pow(zdot.y, 2));

		shape.setPosition(z);
	}

	void draw(sf::RenderWindow& window) {
		window.draw(shape);
	}
};

size_t ball::count = 0; //init static ball object counter

int main()
{
	sf::RenderWindow window(sf::VideoMode(WINDOW_SIZE, WINDOW_SIZE), "Ball simulator 2018");

	float dt;
	int FPS = 200;
	sf::Time period = sf::seconds((float) 1 / FPS);
	size_t num_balls = 70;

	sf::Clock clock;

	vector <ball> ball_vect(num_balls);

	//create balls by specifying initial pose
	//vector <ball> ball_vect;
	//ball_vect.push_back(ball(30, 100, 1, 0, 30));
	//ball_vect.push_back(ball(200, 100, 0, 0, 30));
	//ball_vect.push_back(ball(230, 100, 0, 0, 30));
	//ball_vect.push_back(ball(260, 100, 0, 0, 30));

	sf::Mouse mouse;

	// Declare and load a font
	sf::Font font;
	font.loadFromFile("Courier Prime Code.ttf");

	// Create text object to display kinetic energy
	sf::Text ke_text("", font);
	ke_text.setCharacterSize(30);
	ke_text.setStyle(sf::Text::Bold);
	ke_text.setFillColor(sf::Color::White);
	ke_text.setOutlineColor(sf::Color::Black);
	ke_text.setOutlineThickness(1.0);

	// Create text object to display fps
	sf::Text fps_text("", font);
	fps_text.setCharacterSize(30);
	fps_text.setStyle(sf::Text::Bold);
	fps_text.setFillColor(sf::Color::White);
	fps_text.setOutlineColor(sf::Color::Black);
	fps_text.setOutlineThickness(1.0);
	fps_text.setPosition(WINDOW_SIZE-200,0);

	// used to reduce rate of text updating
	double text_update_timer = 0;

	while (window.isOpen())
	{
		sf::Event event;

		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}

		//throttle update speed to not exceed fps
		sf::Time delta = clock.getElapsedTime();
		sf::Time sleep = period - delta;
		sf::sleep(sleep);
		dt = clock.restart().asSeconds(); //dt since last update

		window.clear();
		float totalKE = 0; // init total kinetic energy

		// iter through all balls in vector and update state
		for (auto it = begin(ball_vect); it != end(ball_vect); ++it) {

			//order matters!
			// check collision with all other balls
			for (auto obj = it + 1; obj != end(ball_vect); ++obj) {
				ball::collide_ball(*it, *obj);
			}

			sf::Vector2i mouse_pose = mouse.getPosition(window);
 
			it->collide_wall();

			//update state
			it->update(dt,mouse_pose,mouse);

			totalKE += it->KE;

			it->draw(window);
		}

		text_update_timer += dt;
		
		// update text occasionally
		if (text_update_timer > 0.1){
			text_update_timer = 0;

			// update kinetic energy
			ostringstream KEos;
			KEos.precision(4);
			KEos << "KE = " << totalKE << endl;
			string KEtxt = KEos.str();
			ke_text.setString(KEtxt);

			// update fps
			ostringstream fps_os;
			fps_os.precision(3);
			fps_os << "FPS =  " << 1. / dt << endl;
			string fps_txt = fps_os.str();
			fps_text.setString(fps_txt);
		}

		window.draw(ke_text);
		window.draw(fps_text);
		window.display();
	}

	return 0;
}