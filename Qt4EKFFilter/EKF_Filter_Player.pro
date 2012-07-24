TEMPLATE = app
CONFIG += qt
CONFIG += thread
QT +=
HEADERS	=	MathStruct.h \
                    PlayerTools.h \
                    Robot.h \
                    MathTools.h \
                    RobotThread.h \
                    Cholesky.h \
    mainwindow.h

SOURCES	=	Robot.cc \
					main.cpp \
    mainwindow.cpp

LIBS      +=

unix:CONFIG += link_pkgconfig
unix:PKGCONFIG += playerc++
unix:INCLUDEPATH += /usr/local/include/player-3.1

FORMS += \
    mainwindow.ui
