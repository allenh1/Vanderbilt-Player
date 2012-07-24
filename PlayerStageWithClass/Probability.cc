#include "Robot.h"
#include <math.h>
#include <time.h>

const double pi = 3.14159265427989;

double Probability::probNormalDistribution(double a, double b){ return pow((1 / sqrt(2 * pi * b)), (-1 / 2) * (pow(a, 2) * b)); }

double Probability::probTriangularDistribution(double a, double b)
{
    double toReturn = 1 / (sqrt(6 * b)) - (abs(a) / (6 * b));

    if (toReturn > 0)
        return toReturn;
    else
        return 0;
}//returns the triangular distribution with mean a and variance b^2.

double Probability::sampleNormDistribution(double b)
{
    int sum = 0;
    srand(time(NULL));

    for (int t = 1; t <= 12; t++)
    {
        int b2 = sqrt(b) * 2;
        int num = -sqrt(b) + rand() % b2;
        sum += num;
    }//add 12 pseudo random numbers

    return (double) sum / 2;
}//returns a pseudo random number (use to reduce noise)




