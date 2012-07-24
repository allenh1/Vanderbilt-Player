#ifndef CHOLESKY_H_INCLUDED
#define CHOLESKY_H_INCLUDED

#include "MathStruct.h"
#include <math.h>
///Preforms Cholesky decomposition and solves linear algebraic equations

///Setting up a global matrix and initize it to 1x1

using namespace std;

class Cholesky
{
    public:

        ///Preforms Cholesky decomposition
        Cholesky(const matrix<double>& A)
        {
            ///Initialization
            n = A.numrows();
            EL = A;
            int i,j,k;
            vector<double> temp;
            double sum = 0;

            if(EL.numcols() != n)
                throw("Need square matrix");

            ///Cholesky decomposition
            for(i = 0; i < n; i++)
            {
                for(j = i; j < n; j++)
                {
                    for(sum = EL[i][j], k = i-1; k >= 0; k--)
                        sum -= EL[i][k] * EL[j][k];

                    if(i == j)
                    {
                        if(sum <= 0.0)
                            throw("Cholesky failed");

                        EL[i][i] = sqrt(sum);
                    }
                    else
                        EL[j][i] = sum/EL[i][i];
                }
            }

            for(i = 0; i < n; i++)
                for(j = 0; j < i; j++)
                    EL[j][i] = 0.;
        }

        ///Solve linear equation through Cholesky decomposition
        /// A*X = B
        void solve(vector<double> &X, const vector<double> &B)
        {
            int i,k;
            double sum;

            if(static_cast<int>(B.size()) != n || static_cast<int>(X.size()) != n)
                throw("Bad lengths in vectors");

            for(i = 0; i < n; i++)
            {
                for(sum = B[i], k = i-1; k >=0; k--)
                    sum -= EL[i][k] * X[k];

                X[i] = sum/EL[i][i];
            }
            for(i = n-1; i >= 0; i--)
            {
                for(sum = X[i], k = i+1; k < n; k++)
                    sum -= EL[k][i] * X[k];

                X[i] = sum/EL[i][i];
            }
        }

        void inverse( matrix<double> &Ainv )
        {
            double sum;
            Ainv.setDimensions(n,n);

            for(int i = 0; i < n; i++)
                for(int j = 0; j <= i; j++)
                {
                    sum = (i == j ? 1.0 : 0.0);

                    for(int k = i-1; k >= j; k--)
                        sum -= EL[i][k]*Ainv[j][k];

                    Ainv[j][i] = sum/EL[i][i];
                }

            for(int i = n-1; i >= 0; i--)
                for(int j = 0; j <= i; j++)
                {
                    sum = (i<j ? 0.0 : Ainv[j][i]);

                    for(int k = i+1; k < n; k++)
                        sum -= EL[k][i]*Ainv[j][k];

                    Ainv[i][j] = Ainv[j][i] = sum/EL[i][i];
                }
        }

        void determinant( double &det )
        {
            det = 1;

            for(int i = 0; i < n; i++)
                det *= EL[i][i]*EL[i][i];
        }

    private:

        matrix<double> EL;

        int n;

};

#endif // CHOLESKY_H_INCLUDED
