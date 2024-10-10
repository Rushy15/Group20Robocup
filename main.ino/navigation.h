#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"
#include <Servo.h>

class Navigation {
  protected:
    
  public:
    Servo Rservo;
    Servo Lservo;
    bool isRemovingWeight = false;
    bool walldetected_bool = false;
    bool robotstuck_bool = false;
    bool weight_detcted_bool = false;

    int check_stuck_count = 0;

    bool weight_entered_channel = false;
    int start_weight_detection;

    int start_timer;
    bool firstTurn = false;

    void navigation_setup();
};


void go_straight();
void go_straight_full();
void stop();
void reverse();
void reverse_full();
void turn_left();
void roll_left();
void turn_left_slow();
void turn_right();
void turn_right_slow();
void roll_right();
void shake();

void walldetected(int state);
void wallFollowingRight();
void wallFollowingLeft();

bool reachedDesiredHeadingAngle(int desiredAngle);
int angleToTurn(int currentHeadingAngle, int angleToTurn, int directionToTurn);

int angleToTurn_WF(int currentHeadingAngle, int angleToTurn, int directionOfTurn);
bool perpendicularToWall(int currentHeadingAngle);

bool get_weight_detected_bool();
void set_weight_detected_bool(bool state);

void set_wall_detected_bool(bool state);

bool get_isRemovingWeight_bool();
void set_isRemovingWeight_bool(bool state);

void weight_entered_entry(int startFindingWeight);
void check_stuck(int directionOfTurn);

/*
Returns true if the angle of the robot about the y-axis is greater than 7 degrees
*/
bool checkPitch();

/*
Returns true if the angle of the robot about the a-axis is + or - 5 degrees
*/
bool checkRoll();

/*
If the robot is on the ramp, it'll get the fuck out of there
*/
void get_the_fuck_out();

void general_navigation();
void weightDetection(bool direction);
void nav_loop(bool weight_detected);

extern Navigation *navigation; 
#endif // NAVIGATION_H