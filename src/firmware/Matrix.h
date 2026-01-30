#ifndef MATRIX_H
#define MATRIX_H

#include <initializer_list>
#include <algorithm>
#include <cmath>

// Type (float/double), Rows, Columns
template <typename T, int R, int C>
class Matrix 
{
public:
    T data[R * C];

    Matrix() 
    {
        std::fill(data, data + (R * C), static_cast<T>(0));
    }

    Matrix(std::initializer_list<T> list)
    {
        int i = 0;
        for (auto val : list)
        {
            if (i < R * C) data[i++] = val;
        }
    }

    T operator()(int r, int c) const
    {
        return data[r * C + c];
    }
    
    T& operator()(int r, int c)
    {
        return data[r * C + c];
    }

    static Matrix<T, R, C> identity()
    {
        static_assert(R == C, "Identity requires a square matrix");
        Matrix<T, R, C> result;
        for (int i = 0; i < R; i++)
        {
            result(i, i) = static_cast<T>(1);
        }
        return result;
    }

    Matrix<T, C, R> transpose() const
    {
        Matrix<T, C, R> result;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    // Returns submatrix with row and column removed
    template <int R2 = R, int C2 = C>
    typename std::enable_if<(R2 > 1 && C2 > 1), Matrix<T, R-1, C-1>>::type
    submatrix(int row, int col) const
    {
        Matrix<T, R-1, C-1> result;
        int ri = 0;
        for (int i = 0; i < R; i++)
        {
            if (i == row) continue;
            int ci = 0;
            for (int j = 0; j < C; j++)
            {
                if (j == col) continue;
                result(ri, ci) = (*this)(i, j);
                ci++;
            }
            ri++;
        }
        return result;
    }

    // Determinant; Laplace expansion along first row

    // 1x1 determinant; a
    template <int N = R>
    typename std::enable_if<(N == 1), T>::type
    determinant() const
    {
        static_assert(R == C, "Determinant requires a square matrix");
        return data[0];
    }

    // 2x2 determinant; ad - bc
    template <int N = R>
    typename std::enable_if<(N == 2), T>::type
    determinant() const
    {
        static_assert(R == C, "Determinant requires a square matrix");
        return (*this)(0,0) * (*this)(1,1) - (*this)(0,1) * (*this)(1,0);
    }

    // General determinant; Laplace expansion along first row
    template <int N = R>
    typename std::enable_if<(N > 2), T>::type
    determinant() const
    {
        static_assert(R == C, "Determinant requires a square matrix");
        T det = 0;
        for (int j = 0; j < C; j++)
        {
            T sign = ((j % 2) == 0) ? 1 : -1;
            det += sign * (*this)(0, j) * submatrix(0, j).determinant();
        }
        return det;
    }

    // Minor; determinant of submatrix with row i and col j removed
    T minor(int row, int col) const
    {
        static_assert(R == C && R > 1, "Minor requires square matrix larger than 1x1");
        return submatrix(row, col).determinant();
    }

    // Cofactor; (-1)^(i+j) * minor(i,j)
    T cofactor(int row, int col) const
    {
        T sign = (((row + col) % 2) == 0) ? 1 : -1;
        return sign * minor(row, col);
    }

    // Cofactor matrix; matrix of all cofactors
    Matrix<T, R, C> cofactorMatrix() const
    {
        static_assert(R == C, "Cofactor matrix requires square matrix");
        Matrix<T, R, C> result;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                result(i, j) = cofactor(i, j);
            }
        }
        return result;
    }

    // Adjugate; transpose of cofactor matrix
    Matrix<T, R, C> adjugate() const
    {
        return cofactorMatrix().transpose();
    }

    // Inverse for 1x1; optimized scalar inverse
    template <int N = R>
    typename std::enable_if<(N == 1), Matrix<T, R, C>>::type
    inverse() const
    {
        static_assert(R == C, "Inverse requires a square matrix");
        Matrix<T, 1, 1> result;
        if (std::abs(data[0]) < static_cast<T>(1e-10))
        {
            return result;  // Return zero for singular
        }
        result.data[0] = static_cast<T>(1) / data[0];
        return result;
    }

    // Inverse for 2x2; optimized direct formula
    template <int N = R>
    typename std::enable_if<(N == 2), Matrix<T, R, C>>::type
    inverse() const
    {
        static_assert(R == C, "Inverse requires a square matrix");
        T det = determinant();
        if (std::abs(det) < static_cast<T>(1e-10))
        {
            return Matrix<T, R, C>();
        }
        Matrix<T, 2, 2> result;
        T invDet = static_cast<T>(1) / det;
        result(0, 0) =  (*this)(1, 1) * invDet;
        result(0, 1) = -(*this)(0, 1) * invDet;
        result(1, 0) = -(*this)(1, 0) * invDet;
        result(1, 1) =  (*this)(0, 0) * invDet;
        return result;
    }

    // Inverse for NxN (N > 2); adjugate / determinant
    template <int N = R>
    typename std::enable_if<(N > 2), Matrix<T, R, C>>::type
    inverse() const
    {
        static_assert(R == C, "Inverse requires a square matrix");
        T det = determinant();
        if (std::abs(det) < static_cast<T>(1e-10))
        {
            // Prevent an exception
            return Matrix<T, R, C>();
        }
        return adjugate() * (static_cast<T>(1) / det);
    }
};

// Matrix addition
template <typename T, int R, int C>
Matrix<T, R, C> operator+(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
    Matrix<T, R, C> res;
    for (int i = 0; i < R * C; i++) 
    {
        res.data[i] = lhs.data[i] + rhs.data[i];
    }
    return res;
}

// Matrix subtraction
template <typename T, int R, int C>
Matrix<T, R, C> operator-(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
    Matrix<T, R, C> res;
    for (int i = 0; i < R * C; i++) 
    {
        res.data[i] = lhs.data[i] - rhs.data[i];
    }
    return res;
}

// Matrix multiplication
template <typename T, int R1, int C1, int R2, int C2>
Matrix<T, R1, C2> operator*(const Matrix<T, R1, C1>& lhs, const Matrix<T, R2, C2>& rhs) 
{
    static_assert(C1 == R2, "Dimension mismatch in matrix multiplication!");
    
    Matrix<T, R1, C2> res;
    for (int i = 0; i < R1; i++) 
    {
        for (int j = 0; j < C2; j++) 
        {
            T sum = 0;
            for (int k = 0; k < C1; k++) 
            {
                sum += lhs(i, k) * rhs(k, j);
            }
            res(i, j) = sum;
        }
    }
    return res;
}

// Scalar multiplication
template <typename T, int R, int C>
Matrix<T, R, C> operator*(const Matrix<T, R, C>& lhs, T scalar) 
{
    Matrix<T, R, C> res;
    for (int i = 0; i < R * C; i++) 
    {
        res.data[i] = lhs.data[i] * scalar;
    }
    return res;
}

#endif