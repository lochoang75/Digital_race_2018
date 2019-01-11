#include "carcontrol.h"

// float temp = 0;

// void get_speed(const std_msgs::Float32::ConstPtr& msg)
// {
//     temp = msg->data;
// }

CarControl::CarControl()
{
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("team211_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("team211_speed",10);
    
    //speed_subscriber = node_obj3.subscribe("get_speed", 10, &get_speed);
}

CarControl::~CarControl() {}

// float CarControl::get_Velocity()
// {
//     velocity = temp;
//     return velocity;
// }

float CarControl::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y;
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    int i = left.size() - 12;
    float error = preError;
    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0) return;
    }
    if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
    {
        error = errorAngle((left[i] + right[i]) / 2);
    } 
    else if (left[i] != DetectLane::null)
    {
        error = errorAngle(left[i] + Point(laneWidth / 2, 0));
    }
    else
    {
        error = errorAngle(right[i] - Point(laneWidth / 2, 0));
    }

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 

void CarControl::drive_Right(const vector<Point> &right, float velocity)
{
    int i = right.size() - 11;
    float error;
    while (right[i] == DetectLane::null) 
    {
        i--;
        if (i < 0) return;
    }
    error = errorAngle(right[i] - Point(laneWidth / 2, 0));
    error = fabsf(error);

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
}

void CarControl::drive_Left(const vector<Point> &left, float velocity)
{
    int i = left.size() - 11;
    float error;
    while (left[i] == DetectLane::null) 
    {
        i--;
        if (i < 0) return;
    }
    error = errorAngle(left[i] + Point(laneWidth / 2, 0));
    

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
}

void CarControl::drive_right()
{
	std_msgs::Float32 angle;
	std_msgs::Float32 speed;
	angle.data = 50;
	speed.data = 45;
	steer_publisher.publish(angle);
	speed_publisher.publish(speed);    
}

void CarControl::drive_left()
{
	std_msgs::Float32 angle;
	std_msgs::Float32 speed;
	angle.data = -50;
	speed.data = 45;
	steer_publisher.publish(angle);
	speed_publisher.publish(speed);    
}

