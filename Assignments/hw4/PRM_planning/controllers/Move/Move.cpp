#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <limits>

#define MAX_SPEED 20

using namespace std;
using namespace webots;

void set_speed(double *speed, int key_value, double velocity)
{
  if (key_value == 'W') // forward
  {
    speed[0] = velocity;
    speed[1] = velocity;
    speed[2] = velocity;
    speed[3] = velocity;
  }
  else if (key_value == 'S')  // backward
  {
    speed[0] = -velocity;
    speed[1] = -velocity;
    speed[2] = -velocity;
    speed[3] = -velocity;
  }
  else if (key_value == 'A')  // leftward
  {
    speed[0] = velocity;
    speed[1] = -velocity;
    speed[2] = velocity;
    speed[3] = -velocity;
  }
  else if (key_value == 'D')  // rightward
  {
    speed[0] = -velocity;
    speed[1] = velocity;
    speed[2] = -velocity;
    speed[3] = velocity;
  }
  else if (key_value == 'Q')  // leftcircle
  {
    speed[0] = velocity;
    speed[1] = -velocity;
    speed[2] = -velocity;
    speed[3] = velocity;
  }
  else if (key_value == 'E')  // rightcircle
  {
    speed[0] = -velocity;
    speed[1] = velocity;
    speed[2] = velocity;
    speed[3] = -velocity;
  }
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  Motor *motors[4];
  char wheels_names[4][8] = {"motor1", "motor2", "motor3", "motor4"};

  for (int i = 0; i < 4; i++) // get motors and initialize
  {
    motors[i] = robot->getMotor(wheels_names[i]);
    motors[i]->setPosition(std::numeric_limits<double>::infinity());
    motors[i]->setVelocity(0.0);
  }

  Keyboard keyboard;
  keyboard.enable(1); // set keyboard read frequency
  double time_step = robot->getBasicTimeStep(); // set virtual time step in simulation
  double velocity = 0.3 * MAX_SPEED;  // set velocity
  while (robot->step(time_step) != -1) 
  {
    int key_value1 = keyboard.getKey();
		int key_value2 = keyboard.getKey();

    double speed1[4] = {0};
    double speed2[4] = {0};
   
    set_speed(speed1, key_value1, velocity);
    set_speed(speed2, key_value2, velocity);

    for (int i = 0; i < 4; i++)
		{
			motors[i]->setVelocity(speed1[i] + speed2[i]);
		}
  };

  delete robot;
  return 0;
}
