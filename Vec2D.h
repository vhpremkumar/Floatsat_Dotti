#ifndef VEC2D_H_
#define VEC2D_H_

#include <math.h>

class Vec2D
{
public:
	double x, y;

	Vec2D() : x(0), y(0) {};
	Vec2D(double x, double y) : x(x), y(y) {};

	double abs();
	Vec2D dirVec(const Vec2D &other);
	double distanceFrom(const Vec2D &other);
	Vec2D normalized();

	Vec2D& operator+= (const Vec2D &r);
	Vec2D& operator-= (const Vec2D &r);
	Vec2D& operator*= (const double &r);
	Vec2D& operator/= (const double &r);

};

Vec2D operator+ (Vec2D a, const Vec2D& b);
Vec2D operator- (Vec2D a, const Vec2D& b);
Vec2D operator* (Vec2D a, const double &b);
Vec2D operator* (const double &a, Vec2D b);
Vec2D operator/ (Vec2D a, const double &b);

#endif /* VEC2D_H_ */
