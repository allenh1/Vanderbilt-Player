#include "Robot.h"

using namespace std;

player_pose2d_t convertToPose(double x, double y, double h)
{
    player_pose2d_t pose;
    pose.px = x;
    pose.py = y;
    pose.pa = h;

    return pose;
}//returns a converted player pose

bool equals (player_pose2d_t a, player_pose2d_t b)
{
    return a.px == b.px && a.py == b.py && a.pa == b.pa;
}

double distance(double x1, double x2, double y1, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

std::string Robot::pose2String(player_pose2d_t pose)
{
    return "("+toString(pose.px, 3)+", "+toString(pose.py, 3)+", "+toString(rtod(pose.pa), 3)+")";
}

void Robot::writeOutFreaks(list<player_pose2d_t> freaks)
{
    for (list<player_pose2d_t>::iterator x = freaks.begin(); x != freaks.end(); x++)
    {
        string position;
        position = pose2String(*x);
        writeLine(position);
    }
}

Robot::Robot(int port, double startX, double startY, double startYaw)
{
    portNumber = port;
    robot = new PlayerClient("localhost", portNumber);
    p2dProxy = new Position2dProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);

    laserProxy->Configure(dtor(1), dtor(180), 0.25, 10, false, 75);
    p2dProxy->SetMotorEnable(1);
    p2dProxy->RequestGeom();
    laserProxy->RequestGeom();
    p2dProxy->SetOdometry(startX, startY, startYaw);
    robot->Read();
    fileName = "Log.txt";
    return;
}//end constructor.

Robot::Robot(int port)
{
    portNumber = port;
    //robot = new PlayerClient("10.20.65.2", portNumber);
    robot = new PlayerClient("localhost", portNumber);
    p2dProxy = new Position2dProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);
    mapProxy = new MapProxy(robot, 0);
    planner = new PlayerClient("localhost", 6666);
    //plannerProxy = new PlannerProxy(robot, 0);
    plannerProxy = new PlannerProxy(planner, 0);

    p2dProxy->SetMotorEnable(1);
    p2dProxy->RequestGeom();
    laserProxy->RequestGeom();
    p2dProxy->SetOdometry(-5, 7, 0);
    robot->Read();
    mapProxy->RequestMap();
    fileName = "Log.txt";
    return;
}
void Robot::updateSpeed()
{
    p2dProxy->SetSpeed(forwardSpeed, dtor(turnSpeed));
}//end void.

void Robot::setForward(double fSpeed)
{
    forwardSpeed = fSpeed;
}

void Robot::setTurn(double tSpeed)
{
    turnSpeed = tSpeed;
}//end void.

double Robot::getFSpeed()
{
    return forwardSpeed;
}//end int

double Robot::getTSpeed()
{
    return turnSpeed;
}//end double

void Robot::read()
{
    robot->Read();
}//end void.

void Robot::setVelHead(double x, double y, double h)
{
    p2dProxy->SetVelHead(x, y, h);
}

bool Robot::atPoint()
{
    return ready;
}

void Robot::goTo(double x, double y, double yaw)
{
    ready = false;

    player_pose2d_t pose = convertToPose(x, y, yaw);

    p2dProxy->GoTo(pose);

    while (p2dProxy->GetXSpeed() != 0 && p2dProxy->GetYawSpeed() != 0)
    {
        /*Do Nothing*/
    }//end while

    ready = true;
}

bool Robot::isStill()
{
    robot->Read();
    return p2dProxy->GetXSpeed() == 0 && p2dProxy->GetYSpeed() == 0 && p2dProxy->GetYawSpeed() == 0;
}

void Robot::writeLine(string data)
{
    ofstream output;
    output.open(fileName.c_str(), ios::app);
    output<<data<<"\n";
}//write a line to the file.

bool Robot::Ready()
{
    return ready;
}

void Robot::planPath()
{
    ready = false;
    bool done = false;
    int freakOuts = 0;
    int successfulSegments = 0;
    bool freaked = false;
    list<player_pose2d_t> freaks;

    plannerProxy->SetGoalPose(10, -6, dtor(180));
    plannerProxy->RequestWaypoints();
    plannerProxy->SetEnable(1);
    while (!done)
    {
        planner->Read();
        robot->Read();
        done = plannerProxy->GetPathDone();

        if (isStill())
        {
            int i = 0;
            bool real = true;

            while (i < 20)
            {
                robot->Read();
                ++i;

                if (!isStill())
                { real = false; break; }
            }//end while

            if (real)
            {
                plannerProxy->SetGoalPose(10, -6, dtor(180));
                freakOuts++;
                freaked = true;
                freaks.push_back(getPose());
            }//end if.
        }//end if.
    }//end while

    plannerProxy->SetEnable(0);
    if (!freaked)
        successfulSegments++;
    done = false;
    plannerProxy->SetGoalPose(-10, -6, dtor(90));
    plannerProxy->RequestWaypoints();
    freaked = false;
    plannerProxy->SetEnable(1);
    while (isStill()){}
    robot->Read();
    while (!done || distance(p2dProxy->GetXPos(), -10, p2dProxy->GetYPos(), -6) < 0.5)
    {
        planner->Read();
        robot->Read();
        done = plannerProxy->GetPathDone();

        if (isStill())
        {
            int i = 0;
            bool real = true;

            while (i < 20)
            {
                robot->Read();
                ++i;

                if (!isStill())
                { real = false; break; }
            }//end while

            if (real)
            {
                plannerProxy->SetGoalPose(-10, -6, dtor(90));
                freakOuts++;
                freaked = true;
                freaks.push_back(getPose());
            }//end if.
        }//end if.
    }//end while

    if (!freaked)
        successfulSegments++;
    done = false;
    freaked = false;
    plannerProxy->SetGoalPose(-5, 7, 0);
    plannerProxy->RequestWaypoints();
    while (isStill()){}
    while (!done)
    {
        robot->Read();
        planner->Read();
        done = plannerProxy->GetPathDone();

        if (isStill())
        {
            int i = 0;
            bool real = true;

            while (i < 20)
            {
                robot->Read();
                ++i;

                if (!isStill())
                { real = false; break; }//end if.
            }//end while

            if (real)
            {
                plannerProxy->SetGoalPose(-5, 7, 0);
                freakOuts++;
                freaked = true;
                freaks.push_back(getPose());
            }//end if.
        }//end if.
    }//end while.

    if (!freaked)
        successfulSegments++;
    writeLine("Number of Freak-outs: "+toString((double) freakOuts, 0));
    writeLine("Successful Segments: "+toString((double) successfulSegments, 0));
    writeLine("Distance to goal: "+toString(distance(p2dProxy->GetXPos(), -5, p2dProxy->GetYPos(), 7), 3));
    writeLine("Angular difference: "+toString(p2dProxy->GetYaw(), 3));
    writeLine("");
    writeOutFreaks(freaks);
    writeLine("");
    writeLine("");
    ready = true;
}

void Robot::kill()
{
    p2dProxy->SetMotorEnable(0);
    forwardSpeed = 0;
    turnSpeed = 0;
    updateSpeed();
}

void Robot::AvoidObstacles()
{
	const double MIN_D = .5;

	if (laserProxy->MinLeft() < MIN_D)
	{
		setCmd("trnR");
	}//end if.

	else if (laserProxy->MinRight() < MIN_D)
	{
		setCmd("trnL");
	}//end else if.

	else
	{
		setCmd("fwd");
	}

    recordPose(getPose());
}//end void

float getPercent(list<double> data)
{
    const double MIN_D = .8;
    int count = 0;

    for (list<double>::iterator it = data.begin(); it != data.end(); it++)
        if (*it < MIN_D)
            count++;

    return (double) count / data.size();
}//get the percent under the min distance/

double getMin(list<double> data)
{
    double min = data.front();

    for (list<double>::iterator x = data.begin(); x != data.end(); x++)
        if (*x < min)
            min = *x;

    return min;
}//get min laser value

double getMean(list<double> data)
{
    double sum = 0;

    for (list<double>::iterator x = data.begin(); x != data.end(); x++)
        sum += *x;

    return sum / data.size();
}//returns mean of the list.

void Robot::Avoidance2(player_pose2d_t pose)
{
    list<double> group1, group2, group3, group4;
    int numPer = laserProxy->GetCount() / 4;

    for (int x = 0; x < numPer; x++)
    {
        group1.push_back(laserProxy->GetRange(x));
        group2.push_back(laserProxy->GetRange(x + numPer));
        group3.push_back(laserProxy->GetRange(x + 2 * numPer));
        group4.push_back(laserProxy->GetRange(x + 3 * numPer));
    }//fill laser readings in the lists.

    float percent1 = getPercent(group1);
    float percent2 = getPercent(group2);
    float percent3 = getPercent(group3);
    float percent4 = getPercent(group4);

    //float preLeft  = percent1 + percent2;
    //float preRight = percent3 + percent4;
    //double normalizer = pow(preLeft + preRight, -1);
    //float left = preLeft * normalizer;
    //float right = preRight * normalizer;//get a percentage per side. Just a comparison value.

    if (percent2 > .5 && percent3 > .5)
    {
        if (percent1 > .4 && percent4 > .4)
        {
            setCmd("bkwd");
        }//no space on left and right.

        else if (percent1 > .25 && !percent4 > .25)
        {
            setCmd("trnR");
        }//no space on left

        else if (percent4 > .25 && !percent1 > .25)
        {
            setCmd("trnL");
        }//no space on right
    }//end if.

    else if (percent1 > .4 && percent2 > .4)
    {
        setCmd("trnR");
    }

    else if (percent1 > .4 && percent4 > .4)
    {
        setCmd("trnL");
    }

    else
    {
        setCmd("fwd");
    }//end else

    recordPose(pose);
}//end void.

std::string Robot::toString(double num, int prec)
{
    stringstream strStream(stringstream::out);

    if(prec > 0)
        strStream << std::setprecision(prec);

    strStream << fixed << num;

    return strStream.str();
}

void Robot::turnRight()
{
    forwardSpeed = 0;
    turnSpeed = -TURN_SPEED;
}//set robot speeds to turn right.

void Robot::turnLeft()
{
    forwardSpeed = 0;
    turnSpeed = TURN_SPEED;
}//set robot speeds to turn left.

void Robot::goForward()
{
    forwardSpeed = 0.5;
    turnSpeed = 0;
}//go forward.

void Robot::goBackward()
{
    forwardSpeed = -0.3;
    turnSpeed = -TURN_SPEED / 2;
}//back up.

bool Robot::getState(int index)
{
    list<bool> nBad = bad;
    nBad.resize(index);

    list<bool>::iterator x = nBad.begin();

    for (; x != nBad.end(); x++)
    {
        /*iterate to the index.*/
    }

    return *x;
}//get the state at instance index.

bool Robot::IsBad()
{
    return laserProxy->MinLeft() < MIN_D || laserProxy->MinRight() < MIN_D;
}//check for a "bad" condition

bool Robot::wasBad(player_pose2d_t pose)
{
    int index = poses.size();

    for (list<player_pose2d_t>::iterator x = poses.end(); x != poses.begin(); x--)
    {
        if (!getState(index))
            return true;

        index--;
    }//end for x.

    return false;
}

bool Robot::hasPastInfo(player_pose2d_t pose)
{
    for (list<player_pose2d_t>::iterator x = poses.begin(); x != poses.end(); x++)
        if (equals(*x, pose))
            return true;

    return false;
}

std::string Robot::getCmd(int index)
{
    list<string> nCmd = commands;
    nCmd.resize(index);

    list<string>::iterator x = nCmd.begin();

    for (; x != nCmd.end(); x++)
    {
        /*iterate to the index.*/
    }

    return *x;
}

std::string Robot::getOpposite(player_pose2d_t pose)
{
    string cmd = getMove(pose);

    if (cmd == "fwd")
        return "bkwd";
    else if (cmd == "bkwd")
        return "fwd";
    else if (cmd == "trnR")
        return "trnL";
    else
        return "trnR";
}

std::string Robot::getMove(player_pose2d_t pose)
{
    int index = poses.size();

    for (list<player_pose2d_t>::iterator x = poses.end(); x != poses.begin(); x--)
    {
        if (equals(pose, *x))
            return getCmd(index);
        index--;
    }

    return "fwd";//default to forward.
}//end string

void Robot::setCmd(string cmd)
{
    if (cmd == "fwd")
        goForward();
    else if (cmd == "bkwd")
        goBackward();
    else if (cmd == "trnL")
        turnLeft();
    else if (cmd == "trnR")
        turnRight();

    rcdCmd(cmd);
}

player_pose2d_t Robot::getPose()
{
    return convertToPose(p2dProxy->GetXPos(), p2dProxy->GetYPos(), p2dProxy->GetYaw());
}
void Robot::rcdCmd(string cmd)
{
    commands.push_back(cmd);
}

void Robot::recordPose(player_pose2d_t pose)
{
    poses.push_back(pose);
}

int Robot::occurances(player_pose2d_t pose)
{
    int count = 0;

    for (list<player_pose2d_t>::iterator x = poses.begin(); x != poses.end(); x++)
    {
        if (equals(pose, *x))
            count++;
    }//end for x.

    return count;
}//returns the number of times a pose has occured in the poses list.

void Robot::makeRTurn()
{
   setCmd("trnR");
   updateSpeed();
//   wait(2);
}

void Robot::Wander()
{
    string cmd = "fwd";//default to forward direction.
    player_pose2d_t pose = convertToPose(p2dProxy->GetXPos(), p2dProxy->GetYPos(), p2dProxy->GetYaw());

    //check position array for previous occurances.
    if (hasPastInfo(pose))
    {
        if (wasBad(pose))
            if (occurances(pose) < 2)
                cmd = getOpposite(pose);
            else
                makeRTurn();
        else
            cmd = getMove(pose);


        setCmd(cmd);
        recordPose(pose);
    }//check for past pose data.

    else
        AvoidObstacles();
}//a simple wandering method.
