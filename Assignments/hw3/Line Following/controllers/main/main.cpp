#include <iostream> 
#include <algorithm>
#include <limits>
#include <string>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
// #define DEBUG

#define MAX_VELOCITY -1

using namespace std;
using namespace webots;

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

int main() 
{
  Robot *robot = new Robot();

	Motor *motors[4];
	char wheels_names[4][8] = { "motor1", "motor2", "motor3", "motor4" };

	double speed1[4];
	double speed2[4];
	double velocity = 10;

	for (int i = 0; i < 4; i++)
	{
		motors[i] = robot->getMotor(wheels_names[i]);
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);

		speed1[i] = 0;
		speed2[i] = 0;
	}

	double speed_forward[4] = { velocity, velocity, velocity, velocity };
	double speed_backward[4] = { -velocity, -velocity, -velocity, -velocity };
	double speed_leftward[4] = { velocity, -velocity, velocity, -velocity };
	double speed_rightward[4] = { -velocity, velocity, -velocity, velocity };

	double speed_leftCircle[4] = { velocity, -velocity, -velocity, velocity };
	double speed_rightCircle[4] = { -velocity, velocity, velocity, -velocity };

	int timeStep = (int)robot->getBasicTimeStep();
  #ifdef DEBUG
	  cout << timeStep << endl;
  #endif

  Camera camera("camera");
  camera.enable(timeStep);
  int image_width = camera.getWidth();
  int image_height = camera.getHeight();
  
  #ifdef DEBUG
    cout << "Image Width: " << image_width << endl;
    cout << "Image Height: " << image_height << endl;
  #endif

  GPS *gps = robot->getGPS("car_gps");
  gps->enable(timeStep);

  double center[3] = {0.0, -0.5, 0.0};
  double radius = 1.79;
  double trace_error = 0.0;
  int counter = 0;

  double motor0_max_velocity = MAX_VELOCITY;
  double motor1_max_velocity = MAX_VELOCITY;
  double motor2_max_velocity = MAX_VELOCITY;
  double motor3_max_velocity = MAX_VELOCITY;

  Keyboard keyboard;
  keyboard.enable(1);
  
	while (robot->step(timeStep) != -1)
	{
    int key = keyboard.getKey();
    if (key == 'S') // stop the car
      break;
    
    const double *pos = gps->getValues();
    cout << "Car Position: "
         << pos[0] << ' '
         << pos[1] << ' '
         << pos[2] << ' ' << endl;

    trace_error += fabs(distance(pos[0], pos[1], center[0], center[1]) - radius);
    counter += 1;
    cout << "Average Error: " << trace_error / counter << endl;

    const unsigned char *image = camera.getImage();
    int pixel_black_left = 0;
    int pixel_black_right = 0;

    for (int x = 0; x < image_width; x++)
    {
      for (int y = 0; y < image_height; y++)
      {
        int grey = camera.imageGetGrey(image, image_width, x, y);

        if (grey < 128 && x < image_width / 2)  // grey pixel on the left part
        {
          pixel_black_left++;
        }
        else if (grey < 128 && x >= image_width / 2)  // grey pixel on the right part
        {
          pixel_black_right++;
        }
      }
    }

    #ifdef DEBUG
      cout << "Left black pixel: " << pixel_black_left << endl;
      cout << "Right black pixel: " << pixel_black_right << endl;
    #endif

    if (pixel_black_left - pixel_black_right > 400)  // turn left
    {
      for (int i = 0; i < 4; i++)
      {
        speed1[i] = speed_forward[i];
        speed2[i] = speed_leftCircle[i] / 3;
      }
    }
    else if (pixel_black_right - pixel_black_left > 400) // turn right
    {
      for (int i = 0; i < 4; i++)
      {
        speed1[i] = speed_forward[i];
        speed2[i] = speed_rightCircle[i] / 3;
      }
    }
    else  // go straight 
    {
      for (int i = 0; i < 4; i++)
      {
        speed1[i] = speed_forward[i];
        speed2[i] = 0;
      }
    }

		for (int i = 0; i < 4; i++)
		{
			motors[i]->setVelocity(speed1[i] + speed2[i]);
		}

    cout << "Average Velocities: "
         << motors[0]->getVelocity() / counter << ' '
         << motors[1]->getVelocity() / counter << ' '
         << motors[2]->getVelocity() / counter << ' '
         << motors[3]->getVelocity() / counter << endl;

    motor0_max_velocity = motors[0]->getVelocity() > motor0_max_velocity ? motors[0]->getVelocity() : motor0_max_velocity;
    motor1_max_velocity = motors[1]->getVelocity() > motor1_max_velocity ? motors[1]->getVelocity() : motor1_max_velocity;
    motor2_max_velocity = motors[2]->getVelocity() > motor2_max_velocity ? motors[2]->getVelocity() : motor2_max_velocity;
    motor3_max_velocity = motors[3]->getVelocity() > motor3_max_velocity ? motors[3]->getVelocity() : motor3_max_velocity;

    cout << "Maximum Velocities: "
         << motor0_max_velocity << ' '
         << motor1_max_velocity << ' '
         << motor2_max_velocity << ' '
         << motor3_max_velocity << endl << endl;
  }

  delete robot;
	return 0;
}
