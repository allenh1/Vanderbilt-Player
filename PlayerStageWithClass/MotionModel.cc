#include "Robot.h"

Pose MotionModel::sampleMotionModelVelocity(U velocity, Pose pose, int t)
{
    Probability *p = new Probability();
    double v2 = velocity.v + p->sampleNormDistribution(pow(velocity.v, 2) + pow(velocity.w, 2));
    double w2 = velocity.w + p->sampleNormDistribution(pow(velocity.v, 2) + pow(velocity.w, 2));
    double g2 = p->sampleNormDistribution(pow(velocity.v, 2) + pow(velocity.w, 2));

    double x2 = pose.x - (v2 / w2) * sin(pose.h) + (v2 / w2) * sin(pose.h + w2 * t);
    double y2 = pose.y + (v2 / w2) * cos(pose.h) + (v2 / w2) * cos(pose.h + w2 * t);
    double h2 = pose.h + w2 * t + g2 * t;

    Pose *q = new Pose();
    q->x = x2;
    q->y = y2;
    q->h = h2;

    return *q;
}//samples motion model. (projects velocity after t seconds)
