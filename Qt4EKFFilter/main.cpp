#include "Robot.h"
#include "RobotThread.h"
#include <QApplication>
#include <QFont>
#include <QThread>
#include <QPushButton>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QPushButton go("Go");

    go.resize(75, 30);
    go.setFont(QFont("Times", 18, QFont::Bold));

    cout<<"Constructing Robot on host: \"localhost\":6665\n";
    Robot robot("localhost", 6665, true);

    go.connect(&go, SIGNAL(clicked()), &robot, SLOT(Wander()));

    go.show();

    RobotThread robotThread(robot);
    robotThread.start();

    return app.exec();
}//end main.
