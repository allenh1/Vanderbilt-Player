#include "Robot.h"

Robot::Robot(QString hostname, int portNum)
{
    robot = new PlayerClient(hostname.toStdString(), portNum);
    p2dProxy = new Position2dProxy(robot, 0);
    localized = new Position2dProxy(robot, 0);
    localizer = new LocalizeProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);
    plannerProxy = new PlannerProxy(robot, 0);
    mapProxy = new MapProxy(robot, 0); //Request the robot's onboard map

    robot->Read();
    laserProxy->RequestGeom();
    p2dProxy->RequestGeom();
    localized->RequestGeom();
    done = false;
}//Default constructor for the robot.

Robot::Robot(QString hostname, int portNum, bool separatePlanner)
{
    robot = new PlayerClient("localhost", portNum);

    if (separatePlanner)
        planner = new PlayerClient("localhost", portNum + 1);
    p2dProxy = new Position2dProxy(robot, 0);
    localized = new Position2dProxy(robot, 0);
    localizer = new LocalizeProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);

    if (separatePlanner)
        plannerProxy = new PlannerProxy(planner, 0);
    else
        plannerProxy = new PlannerProxy(robot, 0);
    mapProxy = new MapProxy(robot, 0); //Request the robot's onboard map

    robot->Read();
    if (separatePlanner)
        planner->Read();
    laserProxy->RequestGeom();
    p2dProxy->RequestGeom();
    localized->RequestGeom();

    hasSeparatePlanner = separatePlanner;
    done = false;
}//Default constructor for the robot.

Robot::Robot(QString hostname, QString startPos, int portNum)
{
    robot = new PlayerClient(hostname.toStdString(), portNum);
    p2dProxy = new Position2dProxy(robot, 0);
    localized = new Position2dProxy(robot, 0);
    localizer = new LocalizeProxy(robot, 0);
    laserProxy = new LaserProxy(robot, 0);
    mapProxy = new MapProxy(robot, 0); //Request the robot's onboard map

    robot->Read();
    laserProxy->RequestGeom();
    p2dProxy->RequestGeom();
    localized->RequestGeom();

    hadStartData = true;
    initPose = startPos;
    done = false;
}//Constructor with input start position.

void Robot::Read()
{
    if (hasSeparatePlanner)
        planner->Read();
    robot->Read();
}//read from the proxies.

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
    toReturn.pa = dtor(a);

    return toReturn;
}

static double getDistance(player_pose2d_t posea, player_pose2d_t poseb)
{
    double x_dist = pow(poseb.px - posea.px, 2);
    double y_dist = pow(poseb.py - posea.py, 2);

    return sqrt(x_dist + y_dist);
}//get distance between two poses.

static double argmax(QList<double> list)
{
    double argmax = list.at(0);

    for (int x = 1; x < list.size(); x++)
        if (list.at(x) > argmax)
            argmax = list.at(x);

    return argmax;
}//get the max arg.

bool Robot::isStill()
{
    return p2dProxy->GetXSpeed() == 0 && p2dProxy->GetYSpeed() == 0;
}

bool Robot::finished()
{
    return false;
}//returns true when the robot is finished with it's task.

double getX(QString line)
{
    int index1 = line.indexOf('(') + 1;
    int index2 = line.indexOf(',');

    return line.mid(index1, index2 - index1).toDouble();
}

double getY(QString line)
{
    int index1 = line.indexOf(',') + 2;
    int index2 = line.indexOf(',', index1);

    return line.mid(index1, index2 - index1).toDouble();
}

double getA(QString line)
{
    int index1 = line.indexOf(',');
    index1 = line.indexOf(',', index1);
    int index2 = line.indexOf(')');

    return line.mid(index1, index2 - index1).toDouble();
}

void Robot::SetPose(QString pose)
{
    /***Position data should be in the format (x, y, θ) where x & y are in meters and θ is in radians***/
    p2dProxy->SetOdometry(getX(pose), getY(pose), getA(pose));
}

void Robot::goTo(player_pose2d_t goal)
{
    int x = 0;

    Read();

    plannerProxy->SetGoalPose(goal.px, goal.py, goal.pa);

    plannerProxy->SetEnable(1);

    while (!plannerProxy->GetPathDone())
    {
        Read();
        writePosition();

        if (x % 5 == 0)
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

        ++x;
    }//end while
}

void Robot::StorePose()
{
    Position pa;
    pa.position_T = GetPose();
    gettimeofday(&pa.time, NULL);

    Positions.push_back(pa);
}//store robot position for reading previous.

void Robot::updateLocalizedPose()
{
    Read();
    StorePose();
}//reset the position of the robot.

double Robot::getDet(matrix<double> X)
{
    double value;

    Cholesky math(X);
    math.determinant(value);

    return value;
}//get determinant of a 3 x 3 matrix.

matrix<double> Robot::getInv(matrix<double> X)
{
    matrix<double> inverse(X.numcols(), X.numrows());

    Cholesky math(X);
    math.inverse(inverse);

    return inverse;
}//get the inverse of a matrix

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

matrix<double> Robot::getGt(double v_t, double w_t, double theta, double delta_t)
{
    matrix<double> G_t(3, 3);

    G_t[0][0] = 1;
    G_t[0][1] = 0;
    if (w_t > 0)
        G_t[0][2] = (-v_t / w_t) * cos(theta) + (v_t/w_t) * cos(theta + w_t * delta_t);
    else
        G_t[0][2] = 0;
    G_t[1][0] = 0;
    G_t[1][1] = 1;
    if (w_t > 0)
        G_t[1][2] = (-v_t / w_t) * sin(theta) + (v_t / w_t) * sin(theta + w_t * delta_t);
    else
        G_t[1][2] = 0;
    G_t[2][0] = 0;
    G_t[2][1] = 0;
    G_t[2][2] = 1;

    return G_t;
}//Set up matrix G sub t.

matrix<double> Robot::getVt(double v_t, double w_t, double theta, double delta_t)
{
    matrix<double> V_t(3, 2);

    V_t[0][0] = (-sin(theta) + sin(theta + w_t * delta_t)) / w_t;
    V_t[0][1] = (v_t * (sin(theta) - sin(theta + w_t * delta_t))) / pow(w_t, 2) + (v_t * cos(theta + w_t * delta_t) * delta_t) / w_t;
    V_t[1][0] = (cos(theta) - cos(theta + w_t * delta_t)) / w_t;
    V_t[1][1] = (-v_t * (cos(theta) - cos(theta + w_t * delta_t))) / pow(w_t, 2) + (v_t * sin(theta + w_t * delta_t) * delta_t) / w_t;
    V_t[2][0] = 0;
    V_t[2][1] = delta_t;

    return V_t;
}//set up matrix V sub t.

matrix<double> Robot::getMt(double v_t, double w_t)
{
    //Motion error: 0.045, 0.0873, 0, 0.02639
    matrix<double> M_t(2, 2);

    M_t[0][0] = 0.045 * pow(v_t, 2) + 0.0873 * pow(w_t, 2);
    M_t[0][1] = 0;
    M_t[1][0] = 0;
    M_t[1][1] = 0 * pow(v_t, 2) + 0.02639 * pow(w_t, 2);

    return M_t;
}//set up matrix M sub t.

matrix<double> Robot::getPrediction(double v_t, double w_t, double delta_t, double theta)
{
    matrix<double> state(3, 1);

    state[0][0] = (-v_t / w_t) * sin(theta) + (v_t / w_t) * sin(theta + w_t * delta_t);
    state[1][0] = (v_t / w_t) * cos(theta) - (v_t / w_t) * cos(theta + w_t * delta_t);
    state[2][0] = w_t * delta_t;

    return state;
}//set up the state prediction matrix.

void Robot::fillLandmarks()
{
    mapProxy->RequestMap();
    double resolution = mapProxy->GetResolution();

    for (uint x = 1; x < mapProxy->GetHeight(); x++)
    {
        for (uint y = 1; y < mapProxy->GetWidth(); y++)
        {
            cout<<mapProxy->GetCell(x, y)<<"\n";

            if(mapProxy->GetCell(x, y) == 1)
            {
                Cell temp;
                temp.x = x; temp.y = y;
                temp.mx = x * resolution;
                temp.my = y * resolution;
                Landmarks.push_back(temp);
            }//end if.
        }//end for y.
    }//end for x.
}//fill the landmarks matrix.

void Robot::Wander()
{
    Read();
    bool filtered = false;
    fillLandmarks();
    StorePose();
    matrix<double> covariance(3, 3);

    covariance[0][0] = 1; covariance[0][1] = 0; covariance[0][2] = 0;
    covariance[1][0] = 0; covariance[1][1] = 1; covariance[1][2] = 0;
    covariance[2][0] = 0; covariance[2][1] = 0; covariance[2][2] = 1;

    while (!filtered)
    {
        Read();
        StorePose();
        double theta = Positions.at(Positions.size() - 1).position_T.pa;

        timeval t1;
        gettimeofday(&t1, NULL);
        MathTools Math;
        double delta_t = t1.tv_usec - Positions.at(Positions.size() - 1).time.tv_usec;
        player_pose2d_t previous_state = Positions.at(Positions.size() - 1).position_T;
        player_pose2d_t previous_state2 = Positions.at(Positions.size() - 2).position_T;

        matrix<double> previous_pose(3, 1);
        matrix<double> G_t(3, 3);
        matrix<double> V_t(2, 3);
        matrix<double> M_t(2, 2);
        double v_t = p2dProxy->GetXSpeed();
        double w_t = (previous_state.pa - previous_state2.pa) / delta_t;
        cout<<"\n Delta_t = "<<delta_t;

        if (w_t == 0)
            w_t = 0.0001;//epsilon of zero.
        cout<<"\n W_t = "<<w_t;

        previous_pose[0][0] = previous_state.px;
        previous_pose[1][0] = previous_state.py;
        previous_pose[2][0] = previous_state.pa;

        //Math.printMatrix(previous_pose);
        cout<<"\n";

        /** Prediction Step **/
        cout<<"\nBeggining Prediction Step\n";
        G_t = getGt(v_t, w_t, theta, delta_t);
        Math.printMatrix(G_t);
        V_t = getVt(v_t, w_t, theta, delta_t);
        M_t = getMt(v_t, w_t);
        matrix<double> precovariance(3, 3);
        matrix<double> precovariance_G_t(3, 3);
        matrix<double> temp_vt(3, 2);
        matrix<double> temp_gt(3, 3);
        matrix<double> precovariance_V_t(3, 3);
        matrix<double> G_t_T(3, 3);
        matrix<double> V_t_T(2, 3);
        matrix<double> prediction(3, 1);

        Math.matrixTranspose(G_t, G_t_T);
        Math.matrixTranspose(V_t, V_t_T);

        Math.matrixMultiply(G_t, covariance, temp_gt);
        Math.matrixMultiply(V_t, M_t, temp_vt);
        Math.matrixMultiply(temp_gt, G_t_T, precovariance_G_t);
        Math.matrixMultiply(temp_vt, V_t_T, precovariance_V_t);

        //get the predicted state (x, y, theta).
        Math.matrixSum(previous_pose, getPrediction(v_t, w_t, delta_t, theta), prediction);
        //get the predicted covariance.
        Math.matrixSum(precovariance_G_t, precovariance_V_t, precovariance);
        cout<<"\n";
        Math.printMatrix(precovariance);
        /** Correction Step **/
        cout<<"\nBeginning Correction Step\n";
        QList< matrix<double> > measurements;
        vector<double> ranges;
        vector<double> bearings;

        for (uint x = 0; x < laserProxy->GetCount(); x++)
        {
            matrix<double> measure(3, 1);
            measure[0][0] = laserProxy->GetRange(x);
            ranges.push_back(laserProxy->GetRange(x));
            measure[1][0] =  laserProxy->GetBearing(x);
            bearings.push_back(laserProxy->GetBearing(x));
            measure[2][0] = 0;

            measurements.push_back(measure);
        }//store measurements in a measurement matrix.

        matrix<double> Q_t(3, 3);
        const double range_mu = Math.getMean(ranges);
        const double bearings_mu = Math.getMean(bearings);

        Q_t[0][0] = Math.getVariance(ranges, range_mu);
        Q_t[0][1] = 0; Q_t[0][2] = 0;
        Q_t[1][0] = 0;
        Q_t[1][1] = Math.getVariance(bearings, bearings_mu);
        Q_t[1][2] = 0;
        Q_t[2][0] = 0; Q_t[2][1] = 0; Q_t[2][1] = 0;

        for (int x = 0; x < measurements.size(); x++)
        {
            cout<<"\nEntered Measurement for loop.";
            for (int y = 0; y < Landmarks.size(); y++)
            {
                double m_kx = Landmarks.at(y).mx; double m_ky = Landmarks.at(y).my;
                double mu_x = prediction[0][0]; double mu_y = prediction[1][0];
                double q = pow(m_kx- mu_x, 2) + pow(m_ky - mu_y, 2);

                if (q == 0)
                    q = 1;

                matrix<double> z_hat(3, 1);
                z_hat[0][0] = sqrt(q);
                z_hat[1][0] = atan2(m_ky - mu_y, m_kx - mu_x);
                z_hat[2][0] = 0;

                matrix<double> H_kt(3, 3);
                H_kt[0][0] = -(m_kx - mu_x) / sqrt(q);
                H_kt[0][1] = -(m_ky - mu_y) / sqrt(q);
                H_kt[0][2] = 0;
                H_kt[1][0] = (m_ky - mu_y) / q;
                H_kt[1][1] = -(m_kx - mu_x) / q;
                H_kt[1][2] = -1;
                H_kt[2][0] = 0; H_kt[2][1] = 0; H_kt[2][2] = 0;
                matrix<double> S_kt(3, 3);
                matrix<double> temp_skt(3, 3);
                matrix<double> temp2_skt(3, 3);
                matrix<double> H_kt_T(3, 3);

                Math.matrixMultiply(H_kt, precovariance, temp_skt);
                Math.matrixTranspose(H_kt, H_kt_T);
                Math.matrixMultiply(temp_skt, H_kt_T, temp2_skt);
                Math.matrixSum(temp2_skt, Q_t, S_kt);

                Zhats.push_back(z_hat);
                Skt.push_back(S_kt);
                Hkt.push_back(H_kt);
            }//end landmark coorespondence.

            for (int z = 0; z < measurements.size(); z++)
            {
                cout<<"\nEntered Second Measurements loop";
                matrix<double> scalar(3, 3);
                matrix<double> zDiff(3, 1);
                matrix<double> zDiff_T(1, 3);
                matrix<double> Sktinv(3, 3);
                matrix<double> multip1(1, 3);
                matrix<double> exp(1, 1);

                Math.scalarMatrixMultiply(2 * Math.getPi(), Skt.at(z), scalar);
                Math.matrixSub(measurements.at(z), Zhats.at(z), zDiff);
                Math.matrixTranspose(zDiff, zDiff_T);
                Sktinv = getInv(Skt.at(z));
                Math.matrixMultiply(zDiff_T, Sktinv, multip1);
                Math.matrixMultiply(multip1, zDiff, exp);
                cout<<"\nj(i) = "<<pow(getDet(scalar), (-1/2.0) * exp[0][0]);
                cout<<"\nexp: "<<exp[0][0];
                cout<<"\nDet: "<<getDet(scalar);
                cout<<"\n";
                Math.printMatrix(scalar);
                cout<<"\n\nzDiff: \n";
                Math.printMatrix(zDiff);
                cout<<"\n\nH_"<<z<<"t: \n";
                Math.printMatrix(Hkt.at(z));
                j_of_i.push_back(pow(getDet(scalar), (-1/2.0) * exp[0][0]));
            }//end for x.

            cout<<"\nExited Second Meausrements Loop";
            int j_max = argmax(j_of_i);
            cout<<"\nEvaluated j_max: "<<j_max;

            matrix<double> KalmanGain(3, 3);
            matrix<double> H_j = Hkt.at(j_max);
            matrix<double> S_j = Skt.at(j_max);
            matrix<double> S_j_inv = getInv(S_j);
            matrix<double> H_j_T(3, 3);
            matrix<double> temp(3, 3);
            matrix<double> zDiff(3, 1);

            Math.matrixTranspose(H_j, H_j_T);
            Math.matrixMultiply(precovariance, H_j_T, temp);
            Math.matrixMultiply(temp, S_j_inv, KalmanGain);

            Math.matrixSub(measurements.at(x), Zhats.at(x), zDiff);

            matrix<double> multip1(3, 3);
            matrix<double> I(3, 3);
            matrix<double> temp2(3, 3);
            matrix<double> temp3(3, 3);

            I[0][0] = 1; I[0][1] = 0; I[0][2] = 0;
            I[1][0] = 0; I[1][1] = 1; I[1][2] = 0;
            I[2][0] = 0; I[2][1] = 0; I[2][2] = 1;

            Math.matrixSub(measurements.at(x), Zhats.at(x), zDiff);
            Math.matrixMultiply(KalmanGain, zDiff, multip1);
            Math.matrixSum(prediction, multip1, prediction);
            Math.matrixMultiply(KalmanGain, H_j, temp2);
            Math.matrixSub(I, temp2, temp3);
            Math.matrixMultiply(precovariance, temp3, precovariance);
        }//end for.

        matrix<double> state(3, 1);
        matrix<double> variance(3, 3);

        Math.printMatrix(state);
    }//end while
}//make the robot go around to places.
