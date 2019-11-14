#include "Matrix2D.h"

double Matrix2D::determinant()
{
	return a11*a22 - a12*a21;
}

Matrix2D Matrix2D::inverted()
{
	double s = determinant();
	if (s < 0.001)
		s = s > 0 ? 0.001 : -0.001;
	return Matrix2D(a22, -a12, -a21, a11) /= s;
}

Matrix2D Matrix2D::transposed()
{
	return Matrix2D(a11, a21, a12, a22);
}

Matrix2D& Matrix2D::operator*= (const Matrix2D &r)
{
	return *this = *this * r;
}

Matrix2D& Matrix2D::operator*= (const double &r)
{
	a11 *= r;
	a12 *= r;
	a21 *= r;
	a22 *= r;
	return *this;
}

Matrix2D& Matrix2D::operator/= (const double &r)
{
	a11 /= r;
	a12 /= r;
	a21 /= r;
	a22 /= r;
	return *this;
}

Matrix2D& Matrix2D::operator+= (const Matrix2D &r)
{
	a11 += r.a11;
	a12 += r.a12;
	a21 += r.a21;
	a22 += r.a22;
	return *this;
}
Matrix2D& Matrix2D::operator-= (const Matrix2D &r)
{
	a11 -= r.a11;
	a12 -= r.a12;
	a21 -= r.a21;
	a22 -= r.a22;
	return *this;
}

Matrix2D operator* (const Matrix2D &a, const Matrix2D &b)
{
	return Matrix2D(a.a11 * b.a11 + a.a12 * b.a21, a.a11 * b.a12 + a.a12 * b.a22,
					a.a21 * b.a11 + a.a22 * b.a21, a.a21 * b.a12 + a.a22 * b.a22);
}

Vec2D operator* (const Matrix2D &a, const Vec2D &b)
{
	return Vec2D(a.a11 * b.x + a.a12 * b.y, a.a21 * b.x + a.a22 * b.y);
}

Matrix2D operator* (Matrix2D a, const double &b)
{
	return a *= b;
}

Matrix2D operator* (const double &a, Matrix2D b)
{
	return b *= a;
}

Matrix2D operator/ (Matrix2D a, const double &b)
{
	return a /= b;
}

Matrix2D operator+ (Matrix2D a, const Matrix2D &b)
{
	return a += b;
}

Matrix2D operator- (Matrix2D a, const Matrix2D &b)
{
	return a -= b;
}
