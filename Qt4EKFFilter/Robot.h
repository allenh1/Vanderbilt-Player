#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include <libplayerc++/playerc++.h>
#include "MathTools.h"
#include "Cholesky.h"
#include <iostream>
#include <sys/time.h>
#include <fstream>
#include <QApplication>
#include <QFont>
#include <QPushButton>

using namespace std;
using namespace PlayerCc;

struct Position
{
    player_pose2d_t position_T;
    timeval time;
};

struct Measurement
{
    double r; //range
    double phi; //bearing
    double s; //signature
};

struct Cell
{
    uint x;
    uint y;
    double mx;//corresponding map x val
    double my;//corresponding map y val
};

class Robot : public QObject
{
    Q_OBJECT
    public:
        Robot(QString, int);
        Robot(QString, int, bool);
        Robot(QString, QString, int);
        bool finished();
        double getDet(matrix<double>);
        double getDet2(matrix<double>);
        matrix<double> getInv(matrix<double>);
        void goTo(player_pose2d_t);
        void Read();
        void StorePose();
        void fileIo();
    public Q_SLOTS:
        void Wander();
    private:
        bool hadStartData;
        bool hasSeparatePlanner;
        bool isStill();
        bool done;
        QString initPose;
        QString Substring(QString, char, char);
        player_pose2d_t GetPose();
        timeval t;
        QList<Position> Positions;
        QList<std::string> PositionLog;
        QList<Cell> Landmarks;
        QList< matrix<double> > Zhats;
        QList< matrix<double> > Hkt;
        QList< matrix<double> > Skt;
        QList<double> j_of_i;
        matrix<double> getGt(double, double, double, double);
        matrix<double> getVt(double, double, double, double);
        matrix<double> getMt(double, double);
        matrix<double> getPrediction(double, double, double, double);
        void SetPose(QString);
        void buildGoalList();
        void fillLandmarks();
        void updateHypothesies();
        void updateLocalizedPose();
        void writePosition();
    protected:
        PlayerCc::PlayerClient *robot;
        PlayerCc::PlayerClient *planner;
        PlayerCc::Position2dProxy *p2dProxy;
        PlayerCc::LaserProxy *laserProxy;
        PlayerCc::LocalizeProxy *localizer;
        PlayerCc::Position2dProxy *localized;
        PlayerCc::MapProxy *mapProxy;
        PlayerCc::PlannerProxy *plannerProxy;
};

#endif // ROBOT_H_INCLUDED
