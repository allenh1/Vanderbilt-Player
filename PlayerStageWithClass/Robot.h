#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include <iostream>
#include <fstream>
#include <sstream>
#include <climits>
#include <iomanip>
#include <time.h>
#include <string>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;
using namespace std;

typedef bool Measurement;
const int numLasers = 361;
const double MIN_D = .5;
const double TURN_SPEED = 40;

class Robot
{
    public:
        Robot(int, double, double, double);
        Robot(int);
        ~Robot(void){};
        double getFSpeed();
        double getTSpeed();
        double getXPos();
        double getYPos();
        player_pose2d_t getPose();
        void read();
        void setVelHead(double, double, double);
        void updateSpeed();
        void setForward(double);
        void setTurn(double);
        void AvoidObstacles();
        void Avoidance2(player_pose2d_t);
        std::string toString(double, int);
        void setupMap();
        void planPath();
        void writeLine(std::string);
        void resetMark();
        void turnLeft();
        void turnRight();
        void goForward();
        void kill();
        void goBackward();
        void makeRTurn();
        void setGoal(double, double, double);
        void goTo(double, double, double);
        void moveToPose(player_pose2d_t *pose);
        int getHeading();
        std::string getOpposite(player_pose2d_t);
        std::string getMove(player_pose2d_t);
        void markPoint();
        void setCmd(string);
        void recordPose(player_pose2d_t);
        bool wasBad(player_pose2d_t pose);
        bool hasPastInfo(player_pose2d_t pose);
        void Wander();
        bool Ready();
        void writeOutFreaks(std::list<player_pose2d_t>);
        bool atPoint();
    private:
        bool IsBad();
        bool WasBad();
        bool close(double, double, double);
        bool ready;
        bool isStill();
        bool getState(int);
        int portNumber;
        int occurances(player_pose2d_t);
        double forwardSpeed;
        double turnSpeed;
        std::string fileName;
        std::string pose2String(player_pose2d_t);
        std::string getCmd(int);
        void rcdCmd(string);
        list<player_pose2d_t> poses;
        list<bool> bad;
        list<std::string> commands;
    protected:
        PlayerClient *robot;
        PlayerClient *planner;
        Position2dProxy *p2dProxy;
        LaserProxy *laserProxy;
        PlannerProxy *plannerProxy;
        SpeechProxy *speechProxy;
        MapProxy *mapProxy;
};//end class
#endif // ROBOT_H_INCLUDED
