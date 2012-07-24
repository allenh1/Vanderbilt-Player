#ifndef MATHTOOLS_H_INCLUDED
#define MATHTOOLS_H_INCLUDED

#include <math.h>
#include "MathStruct.h"
#include <fstream>

#define MATH_PI 3.14159265358979323846264338327950288

class MathTools
{
    public:

///----------------- Importing --------------------------------------------------------------

        void readFile(string fileName, int dataCount, vector<double>& dataX, vector<double>& dataY)
        {
            dataX.resize(dataCount);
            dataY.resize(dataCount);

            ifstream inputFile;
            inputFile.open (fileName.c_str());

            ///creating loop to insert data from txt file
            ///to two vectors
            for (int count = 0; count < dataCount; count++)
            {
                ///saving first row in dataX
                inputFile >> dataX[count];
                ///saving second row in dataY
                inputFile >> dataY[count];
            }
            inputFile.close();
        }

///----------------- Definition ---------------------------------------------------------------
        ///Returns Pi
        inline double getPi() const
        {
            return MATH_PI;
        }
/// ---------------- Operations ---------------------------------------------------------------
        ///Returns power of two
        inline double pow2(double number) const
        {
            return number*number;
        }

///----------------- Converting ---------------------------------------------------------------

        ///Converts degrees to radians
        inline void convToRad(double& degrees)
        {
            degrees *= getPi()/180;
        }

        ///Conerts radians to degree
        inline void ConvToDeg(double& radians)
        {
            radians *= 180/getPi();
        }
///------------------- Distance ---------------------------------------------------------------
        inline double getDist(double& x1, double& x2, double& y1, double& y2) const
        {
            return( sqrt(pow2(x2-x1)+pow2(y2-y1)) );
        }

///------------------- Statistics -------------------------------------------------------------

        ///Get mean
        double getMean( const vector<double>& dataVec )
        {
            double sum = 0;
            int i = 0;

            for(i = 0; i < static_cast<int>( dataVec.size() ); i++)
            {
                sum += dataVec[i];
            }

            return sum/i;
        }

        ///Calculate variance
        double getVariance( const vector<double>& dataVec, const double& mean)
        {
            double variance = 0;

            for(int i = 0; i < static_cast<int>( dataVec.size() ); i++)
            {
                variance += pow2( (dataVec[i] - mean) );
            }

            return variance;
        }

        ///Calculate covariance
        double getCovariance( const vector<double>& vecX, const vector<double>& vecY, const double& mean_x, const double& mean_y)
        {
            double covariance = 0;

            for(int i = 0; i < static_cast<int>( vecX.size() ); i++)
            {
                covariance += (vecX[i] - mean_x)*(vecY[i] - mean_y);
            }

            return covariance;
        }

        ///Calculation linear regression
        void LinearReg( const vector<double>& dataX, const vector<double>& dataY, double& angle, double& range)
        {
//            cout << "Testing: " << endl;

            double mean_x = getMean(dataX);
            double mean_y = getMean(dataY);
            double var_xx = getVariance(dataX, mean_x);
            double var_yy = getVariance(dataY, mean_y);
            double cov_xy = getCovariance(dataX, dataY, mean_x, mean_y);
/*
            cout << "Mean: " << "X: " << mean_x << " Y: " << mean_y << endl;
            cout << "Variance: " << "X: " << var_xx << " Y: " << var_yy << endl;
            cout << "Covariance: " << cov_xy << endl;
            cout << "-----------------------" << endl;
*/
            //angle = atan( 2*(cov_xy) / (var_xx-var_yy-sqrt( pow(var_xx-var_yy, 2) + 4*pow(cov_xy, 2))));
            angle = atan2( 2*(cov_xy) , (var_xx-var_yy-sqrt( pow(var_xx-var_yy, 2) + 4*pow(cov_xy, 2))) );
            range = mean_x * cos(angle) + mean_y * sin(angle);

            if(range < 0)
            {
//                cout << "Negative Range: " << range << endl;
                angle += getPi();
                range *= -1;

                if(angle > (getPi()*1.1))
                    angle -= 2*getPi();
            }

        ///Tolerance
            if( angle > (getPi()*0.97) || angle < (-getPi()*0.97) )
                angle = getPi();
/*
                    cout << "Angle: " << angle << endl;
                    cout << "Range: " << range << endl;
                    cout << "---------------------" << endl;
*/
        }

        ///Get y values according to x, angle and range values
        inline double LinRegGetY( double& x, double& angle, double& range) const
        {
            return (range - cos(angle)*x)/sin(angle);
        }

        ///Tests if the point p(x,y) is close enough to linie l(range,angle) according to tolerance
        bool LinRegGetFit(double x, double y, double angle, double range, double tolerance)
        {
            double distance = sqrt(pow2(x) + pow2(y)) * cos(atan2(y,x) - angle) - range;

            if(fabs(distance) < tolerance)
                return true;
            else
                return false;
        }



///------------------- Matrix - Vector ---------------------------------------------------------

        ///Prints out a matrix to console
        void printMatrix(const matrix<double>& A) const
        {
                for(int row = 0; row < A.numrows(); row++)
                {
                    for(int col = 0; col < A.numcols(); col++)
                    {
                        cout << A[row][col] << "\t";
                    }
                    cout << "\n";
                }
        }

        void printMatrixOut(const matrix<double>& A, ofstream& file) const
        {
                for(int row = 0; row < A.numrows(); row++)
                {
                    for(int col = 0; col < A.numcols(); col++)
                    {
                        file << A[row][col] << "\t";
                    }
                    file << "\n";
                }
        }

        ///Prints out a vector to console
        void printVector(const vector<double>& A) const
        {
            for(int row = 0; row < static_cast<int>(A.size()); row++)
                cout << A[row] << "\n";
        }

        void printVectorOut(const vector<double>& A, ofstream& file) const
        {
            for(int row = 0; row < static_cast<int>(A.size()); row++)
                file << A[row] << "\n";
        }

        void printPairs(const vector< pair<double,double> >& X) const
        {
            cout << "Number of pair: " << X.size() << endl;
            cout << "First \t Second" << endl;

            for(int row = 0; row < static_cast<int>(X.size()); row++)
                cout << X[row].first << "\t" << X[row].second << endl;
        }

        void printPairsOut(const vector< pair<double,double> >& X, ofstream& file) const
        {
            file << "Number of pair: " << X.size() << endl;
            file << "First \t Second" << endl;

            for(int row = 0; row < static_cast<int>(X.size()); row++)
                file << X[row].first << "\t" << X[row].second << endl;
        }

        ///Multiplies matrix A and B, then returns the result
        /// A*B = AB
        void matrixMultiply(const matrix<double>& A, const matrix<double>& B, matrix<double>& AB)
        {
            if(A.numcols() == B.numrows()){

                    AB.setDimensions(A.numrows(),B.numcols());

                    for(int row = 0; row < A.numrows(); row++){
                        for(int col = 0; col < B.numcols(); col++){

                            double temp = 0;

                            for(int k = 0; k < A.numcols(); k++){
                                temp += A[row][k] * B[k][col];
                            }

                            AB[row][col] = temp;
                        }
                    }
                }
             else
                throw("Wrong matrix size: A[p][n]*B[n][q]");
        }


        ///Calculate the transpone matrix
        void matrixTranspose(const matrix<double>& A, matrix<double>& A_T)
        {
            A_T.setDimensions(A.numcols(), A.numrows());

            for(int row = 0; row < A.numrows(); row++)
                for(int col = 0; col < A.numcols(); col++)
                    A_T[col][row] = A[row][col];
        }

        ///Matrix Vector multiplication
        ///A[][] * X[] = Y[]
        void matrixVectorMultiply(const matrix<double>& A, const vector<double>& X, vector<double>& Y)
        {
            if(A.numcols() != static_cast<int>(X.size()))
                throw("Wrong matrix and vector fit");

            Y.resize(A.numrows());
            double temp;

            for(int row = 0; row < A.numrows(); row++)
            {
                temp = 0;

                for(int col = 0; col < A.numcols(); col++)
                {
                    temp += A[row][col] * X[col];
                }

                Y[row] = temp;
            }
        }

        ///Scalar Matrix multiplication
        ///scalar * A[][] = As[][]
        void scalarMatrixMultiply(double scalar, const matrix<double>& A, matrix<double>& As)
        {
            As = A;

            for(int row = 0; row < A.numrows(); row++)
                for(int col = 0; col < A.numcols(); col++)
                    As[row][col] *= scalar;

        }

        ///Vector Matrix multiplication, note that Vector is assumed transposed in the
        ///calculation but the input is an ordinary Vector
        ///X^T[] *A[][] = Y^T[]
        void vectorMatrixMultiply(const vector<double>& X, const matrix<double>& A, vector<double>& Y_T)
        {
            if(static_cast<int>(X.size()) != A.numrows())
                throw("Wrong vector and matrix fit");

            Y_T.resize(A.numcols());

            for(int col = 0; col < A.numcols(); col++)
            {
                Y_T[col] = 0;

                for(int row = 0; row < A.numrows(); row++)
                    Y_T[col] += X[row]*A[row][col];
            }




        }

        ///Vector Vector multiplication, note that the first vector is assumed transposed in
        ///the calculation but the input is an ordinary Vector
        ///X^T[] * X[] = value
        void vectorVectorMultiply(const vector<double>& X_T, const vector<double>& X, double & value)
        {
            if(X_T.size() != X.size())
                throw("Wrong vector and vector fit");

            value = 0;

            for(int i = 0; i < static_cast<int>(X_T.size()); i++)
                value += X_T[i]*X[i];
        }

        void matrixSum(const matrix<double>& A, const matrix<double>& B, matrix<double>& AB)
        {
            if(A.numcols() != B.numcols() && A.numrows() != B.numrows())
                throw("Wrong fit between matrixes");

            AB.setDimensions(A.numrows(), A.numcols());

            for(int row = 0; row < A.numrows(); row++)
                for(int col = 0; col < A.numcols(); col++)
                {
                    AB[row][col] = A[row][col] + B[row][col];
/*
                    cout << "A[" << row << "][" << col << "] = " << A[row][col] << endl;
                    cout << "B[" << row << "][" << col << "] = " << B[row][col] << endl;
                    cout << "A + B = " << AB[row][col] << endl;
                    cout << " " << endl;
*/
                }

        }

        void matrixSub(const matrix<double>& A, const matrix<double>& B, matrix<double>& AB)
        {
            if(A.numcols() != B.numcols() && A.numrows() != B.numrows())
                throw("Wrong fit between matrixes");

            AB.setDimensions(A.numrows(), A.numcols());

            for(int row = 0; row < A.numrows(); row++)
                for(int col = 0; col < A.numcols(); col++)
                {
                    AB[row][col] = A[row][col] - B[row][col];
/*
                    cout << "A[" << row << "][" << col << "] = " << A[row][col] << endl;
                    cout << "B[" << row << "][" << col << "] = " << B[row][col] << endl;
                    cout << "A + B = " << AB[row][col] << endl;
                    cout << " " << endl;
*/
                }

        }

        void vectorSub(const vector<double>& X1, const vector<double>& X2, vector<double>& X12)
        {
            if(X1.size() != X2.size())
                throw("Wrong fit between vectors");

            X12.resize(X1.size());

            for(int row = 0; row < static_cast<int>(X1.size()); row++)
                X12[row] = X1[row] - X2[row];
        }

        void vectorSum(const vector<double>& X1, const vector<double>& X2, vector<double>& X12)
        {
            if(X1.size() != X2.size())
                throw("Wrong fit between vectors");

            X12.resize(X1.size());

            for(int row = 0; row < static_cast<int>(X1.size()); row++)
                X12[row] = X1[row] + X2[row];
        }//end void
};

#endif // MATHTOOLS_H_INCLUDED

