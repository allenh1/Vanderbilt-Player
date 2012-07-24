#include "Robot.h"

Robot::Robot(QString hostname, int portNum)
{
    robot = new PlayerClient(hostname.toStdString(), portNum);
    p2dProxy = new Position2dProxy(robot, 0);
    localized = new Position2dProxy(robot, 0);
    localizer = new LocalizeProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);
    plannerProxy = new PlannerProxy(robot, 0);
    map = new MapProxy(robot, 0); //Request the robot's onboard map

    robot->Read();
    laserProxy->RequestGeom();
    p2dProxy->RequestGeom();
    localized->RequestGeom();
}//Default constructor for the robot.

Robot::Robot(QString hostname, int portNum, bool separatePlanner)
{
    robot = new PlayerClient(hostname.toStdString(), portNum);
    if (separatePlanner)
        planner = new PlayerClient(hostname.toStdString(), portNum + 1);
    p2dProxy = new Position2dProxy(robot, 0);
    localized = new Position2dProxy(robot, 0);
    localizer = new LocalizeProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);

    if (separatePlanner)
        plannerProxy = new PlannerProxy(planner, 0);
    else
        plannerProxy = new PlannerProxy(robot, 0);
    map = new MapProxy(robot, 0); //Request the robot's onboard map

    robot->Read();
    if (separatePlanner)
        planner->Read();
    laserProxy->RequestGeom();
    p2dProxy->RequestGeom();
    localized->RequestGeom();

    hasSeparatePlanner = separatePlanner;
}//Default constructor for the robot.

Robot::Robot(QString hostname, QString startPos, int portNum)
{
    robot = new PlayerClient(hostname.toStdString(), portNum);
    p2dProxy = new Position2dProxy(robot, 0);
    localized = new Position2dProxy(robot, 0);
    localizer = new LocalizeProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);
    map = new MapProxy(robot, 0); //Request the robot's onboard map

    robot->Read();
    laserProxy->RequestGeom();
    p2dProxy->RequestGeom();
    localized->RequestGeom();

    hadStartData = true;
    initPose = startPos;
}//Constructor with input start position.

player_pose2d_t Robot::GetPose()
{
    player_pose2d_t toReturn;

    toReturn.px = p2dProxy->GetXPos();
    toReturn.py = p2dProxy->GetYPos();
    toReturn.pa = p2dProxy->GetYaw();

    return toReturn;
}//Accessor for the position of the robot.

player_pose2d_t toPlayerPose(double x, double y, double a)
{
    player_pose2d_t toReturn;

    toReturn.px = x;
    toReturn.py = y;
    toReturn.pa = a;

    return toReturn;
}

static double getDistance(player_pose2d_t posea, player_pose2d_t poseb)
{
    double x_dist = pow(poseb.px - posea.px, 2);
    double y_dist = pow(poseb.py - posea.py, 2);

    return sqrt(x_dist + y_dist);
}//get distance between two poses.

bool Robot::isStill()
{
    return p2dProxy->GetXSpeed() == 0 && p2dProxy->GetYSpeed() == 0;
}

QString Robot::Substring(QString String, char a, char b)
{
    return String;

    /**To be implemented later. Currently returns the input string. Will eventually return a substring.***/
}

void Robot::SetPose(QString pose)
{
    /***Position data should be in the format (x, y, θ) where x & y are in meters and θ is in radians***/
}

void Robot::updateHypothesies()
{
    hypothesies.clear();

    for (uint x = 0; x < localizer->GetHypothCount(); x++)
        hypothesies.push_back(localizer->GetHypoth(x));
}

void Robot::goTo(player_pose2d_t goal)
{
    Read();

    plannerProxy->SetGoalPose(goal.px, goal.py, goal.pa);

    plannerProxy->SetEnable(1);

    while (!plannerProxy->GetPathDone())
    {
        Read();
        writePosition();
        updateLocalizedPose();

        if (isStill())
        {
            int i = 0;

            while (i < 20)
            {
                robot->Read();
                ++i;

                if (!isStill())
                    break;
            }//end while

            plannerProxy->SetGoalPose(goal.px, goal.py, goal.pa);
        }//end if.
    }//end while
}

void Robot::updateLocalizedPose()
{
    Read();
    updateHypothesies();

    cout<<"Number of Hypothesies: "<<hypothesies.size()<<"; ";

    player_pose2d_t pose = GetPose();

    int index = 0;
    int minDist = 100;

    for (int x = 0; x < hypothesies.size(); x++)
    {
        if (getDistance(pose, hypothesies.at(x).mean) < minDist)
        {
            index = x;
            minDist = getDistance(pose, hypothesies.at(x).mean);
        }//end if.
    }//end for x.

    cout<<"Setting Position to ("<<hypothesies.at(index).mean.px<<", "<<hypothesies.at(index).mean.py<<", "<<hypothesies.at(index).mean.pa<<")\n";
    writePosition();
    p2dProxy->SetOdometry(hypothesies.at(index).mean.px, hypothesies.at(index).mean.py, hypothesies.at(index).mean.pa);
}//reset the position of the robot.

void Robot::Read()
{
    if (hasSeparatePlanner)
        planner->Read();
    robot->Read();
}//read from the proxies.

void Robot::buildGoalList()
{
    goals.clear();

    goals.push_back(toPlayerPose(9.88, -29.5, 180));
    goals.push_back(toPlayerPose(-14, -30.5, 90));
    goals.push_back(toPlayerPose(14.34, -23.1, 180));
    goals.push_back(toPlayerPose(-8.89, 22.06, -90));
    goals.push_back(toPlayerPose(0, 0, 180));
}

void Robot::writePosition()
{
    /***This function records the localized position and the odometry.***/
    QString x, y, a;

    x.setNum(p2dProxy->GetXPos());
    y.setNum(p2dProxy->GetYPos());
    a.setNum(p2dProxy->GetYaw());

    QString toWrite = "Position: ("+x+", "+y+", "+a+")";

    PositionLog.push_back(toWrite.toStdString());
}

void Robot::fileIo()
{
    ofstream file;
    file.open("PositionLog.txt");

    for (int x = 0; x < PositionLog.size(); x++)
        file<<PositionLog[x]<<"\n";

    file.close();
}//write position data to the file.

void Robot::Map()
{
    while (1)
    {
        if (hypothesies.size() >= 1)
            updateLocalizedPose();
        Read();
        updateHypothesies();
    }//end while true
}

void Robot::Wander()
{
    buildGoalList();
    int x = 0;

    while (x < goals.size())
    {
        writePosition();

        if (hypothesies.size() >= 1)
            updateLocalizedPose();
        Read();
        writePosition();
        updateHypothesies();

        goTo(goals[x]);

        x++;
    }//end while

    fileIo();
}//make the robot go around to places.
