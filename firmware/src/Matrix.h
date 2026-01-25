#ifndef MATRIX_H
#define MATRIX_H

#include <initializer_list>
#include <algorithm>
#include <cmath>

// Type (float/double), Rows, Columns
template <typename T, int R, int C>
class Matrix {
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

    Matrix<T, R, C> transpose() const
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

    Matrix<T, R, C> inverse() const
    {
        static_assert(R == C, "Inverse requires a square matrix");
        T det = determinant();
        if (det == 0) throw std::runtime_error("Matrix is singular, cannot compute inverse");
        Matrix<T, R, C> adj = adjoint();
        return adj * (1 / det);
    }

    T determinant() const
    {
        static_assert(R == C, "Determinant requires a square matrix");
        T det = 0;
        for (int i = 0; i < R; i++)
        {
            det += (*this)(i, 0) * adjoint()(i, 0);
        }
        return det;
    }

    Matrix<T, R, C> adjoint() const
    {
        Matrix<T, R, C> result;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                result(i, j) = (*this)(j, i);
            }
        }
        return result;
    }

    Matrix<T, R, C> cofactor() const
    {
        Matrix<T, R, C> result;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                result(i, j) = (*this)(i, j) * (i + j) % 2 == 0 ? 1 : -1;
            }
        }
        return result;
    }

    Matrix<T, R, C> minor() const
    {
        Matrix<T, R, C> result;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                result(i, j) = (*this)(i, j) * (i + j) % 2 == 0 ? 1 : -1;
            }
        }
        return result;
    }
};

// Addition
template <typename T, int R, int C>
Matrix<T, R, C> operator+(const Matrix<T, R C>& lhs, const Matrix<T, R, C>& rhs)
{
    Matrix<T, R, C> res;
    for (int i = 0; i < R * C; i++) 
    {
        res.data[i] = lhs.data[i] + rhs.data[i];
    }
    return res;
}

// Subtraction
template <typename T, int R, int C>
Matrix<T, R, C> operator-(const Matrix<T, R C>& lhs, const Matrix<T, R, C>& rhs)
{
    Matrix<T, R, C> res;
    for (int i = 0; i < R * C; i++) 
    {
        res.data[i] = lhs.data[i] - rhs.data[i];
    }
    return res;
}
// Matrix Multiplication
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

// Scalar Multiplication
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