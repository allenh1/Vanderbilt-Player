#ifndef MATHSTRUCT_H_INCLUDED
#define MATHSTRUCT_H_INCLUDED
#include <vector>
using namespace std;

template <typename Object>
class matrix
{
    public:

        matrix(int rows, int cols) : array(rows)
            {
                setNumCols(cols);
            }

        matrix() : array(0)
            {
                setNumCols(0);
            }

        inline int numrows() const
            {
                return array.size();
            }

        inline int numcols() const
            {
                return numrows() > 0 ? array[0].size() : 0;
            }

        inline void setNumRows(int rows)
            {
                array.resize(rows);
            }

        inline void setNumCols(int cols)
            {
                for(int i = 0; i < numrows(); i++)
                array[i].resize(cols);
            }

        inline void setDimensions(int rows, int cols)
            {
                setNumRows(rows);
                setNumCols(cols);
            }

        inline void clearMatrix()
            {
                array.clear();
            }

        const vector<Object> & operator[](int row) const
            {
                return array[row];
            }

        vector<Object> & operator[](int row)
            {
                return array[row];
            }

    private:
        vector< vector<Object> > array;

};

template <typename Object>
class linkedlist
{
    public:

        linkedlist() : array(0)
            {
                array[0].resize(0);
            }

        inline void addList()
            {
                array.push_back( vector<Object>() );
            }

        inline void addElement(int list, double element) const
            {
                array[list].push_back(element);
            }

        const vector<Object> & operator[](int row) const
            {
                return array[row];
            }

        vector<Object> & operator[](int row)
            {
                return array[row];
            }

    private:
        vector< vector<Object> > array;

};



#endif // MATHSTRUCT_H_INCLUDED
