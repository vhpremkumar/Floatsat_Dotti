#include "Vec2D.h"

double Vec2D::abs()
{
	return sqrt(x*x + y*y);
}

Vec2D Vec2D::dirVec(const Vec2D &other)
{
	return other - *this;
}

double Vec2D::distanceFrom(const Vec2D &other)
{
	return dirVec(other).abs();
}

Vec2D Vec2D::normalized()
{
	return *this / abs();
}

Vec2D& Vec2D::operator+= (const Vec2D& r)
{
	x += r.x;
	y += r.y;
	return *this;
}

Vec2D& Vec2D::operator-= (const Vec2D& r)
{
	x -= r.x;
	y -= r.y;
	return *this;
}

Vec2D& Vec2D::operator*= (const double& r)
{
	x *= r;
	y *= r;
	return *this;
}

Vec2D& Vec2D::operator/= (const double& r)
{
	x /= r;
	y /= r;
	return *this;
}

Vec2D operator+ (Vec2D a, const Vec2D& b)
{
	return a += b;
}

Vec2D operator- (Vec2D a, const Vec2D& b)
{
	return a -= b;
}

Vec2D operator* (Vec2D a, const double &b)
{
	return a *= b;
}

Vec2D operator* (const double &a, Vec2D b)
{
	return b *= a;
}

Vec2D operator/ (Vec2D a, const double &b)
{
	return a /= b;
}
