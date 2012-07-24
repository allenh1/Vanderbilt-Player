#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
#include "Robot.h"

using namespace std;
using namespace PlayerCc;

void wait ( int seconds )
{
    clock_t endwait;
    endwait = clock () + seconds * CLOCKS_PER_SEC ;
    while (clock() < endwait) {}
}

void goAround(int port)
{
    Robot robot(port);
    char answer;

    cout<<"Welcome to the path planner test!\n";
    cout<<"The first waypoint is (11, 7).\n";

    robot.goTo(11, 7, dtor(-90));
    cout<<"Ok to continue to (11, -6)? (y/n) ";
    cin>>answer;
    if (answer == 'y')
        robot.goTo(11, -6, dtor(180));
    if (answer == 'k')
        robot.kill();
    cout<<"Ok to continue to (-9.5, -6)? (y/n) ";
    cin>>answer;
    if (answer == 'y')
        robot.goTo(-9.5, -6, dtor(90));
    if (answer == 'k')
        robot.kill();
    cout<<"Ok to continue to (-9.5, 7)? (y/n) ";
    cin>>answer;
    if (answer == 'y')
        robot.goTo(-9.5, 7, dtor(0));
    if (answer == 'k')
        robot.kill();
    cout<<"Ok to continue to (-5.4, 7)? (y/n) ";
    cin>>answer;
    if (answer == 'y')
        robot.goTo(-5.4, 7, dtor(-90));
    if (answer == 'k')
        robot.kill();
    cout<<"Ok to continue to (-5.4, 3)? (y/n) ";
    cin>>answer;
    if (answer == 'y')
        robot.goTo(-5.4, 3, dtor(-90));
    if (answer == 'k')
        robot.kill();
    cout<<"All done! ";
    cin>>answer;
}

void planPath(int port)
{
    Robot robot(port);

    cout<<"Welcome to the Wavefront test!";
    for (int x = 0; x < 25; x++)
    {
        robot.planPath();

        while (!robot.Ready()){}
    }
}

int main(int argc, char *argv[])
{

    int port;

    cout<<"On what port do you want to construct the robot? ";
    cin>>port;

    planPath(port);

    return 0;
}
