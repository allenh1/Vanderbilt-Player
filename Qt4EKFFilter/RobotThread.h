#ifndef ROBOTTHREAD_H
#define ROBOTTHREAD_H
#include <QThread>
#include "Robot.h"

/** This file contains the robot thread. This thread is used for robot control. **/

class RobotThread : public QThread {
public:
    RobotThread(Robot& rRobot) : m_Robot(rRobot) {}

    void run() {
        while (!m_Robot.finished()) {/** Do Nothing **/}
        qApp->exit();
    }

private:
    Robot& m_Robot;
};
#endif // ROBOTTHREAD_H
