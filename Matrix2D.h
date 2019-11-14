#ifndef MATRIX2D_H_
#define MATRIX2D_H_

#include "Vec2D.h"

class Matrix2D
{
public:
	double a11, a12, a21, a22;

	Matrix2D() : a11(1), a12(0), a21(0), a22(1) {};
	Matrix2D(double a11, double a12, double a21, double a22)
		: a11(a11), a12(a12), a21(a21), a22(a22) {};

	double determinant();
	Matrix2D inverted();
	Matrix2D transposed();

	Matrix2D& operator*= (const Matrix2D &r);
	Matrix2D& operator*= (const double &r);
	Matrix2D& operator/= (const double &r);
	Matrix2D& operator+= (const Matrix2D &r);
	Matrix2D& operator-= (const Matrix2D &r);

};

Matrix2D operator* (const Matrix2D &a, const Matrix2D &b);
Vec2D operator* (const Matrix2D &a, const Vec2D &b);
Matrix2D operator* (Matrix2D a, const double &b);
Matrix2D operator* (const double &a, Matrix2D b);
Matrix2D operator/ (Matrix2D a, const double &b);
Matrix2D operator+ (Matrix2D a, const Matrix2D &b);
Matrix2D operator- (Matrix2D a, const Matrix2D &b);

#endif /* MATRIX2D_H_ */
