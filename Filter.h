/*
 * filter.h
 *
 *  Created on: 28.11.2017
 *      Author: I8FL-PC01-G01
 */

#ifndef FILTER_H_
#define FILTER_H_


#include "matlib.h"
#include "myMath.h"
#include "myStructs.h"



class KalmanFilter{

	Matrix2D I;
	Matrix2D A;
	Matrix2D C;
	Kalman_Q_R qr;

public:
	KalmanFilter();
	virtual ~KalmanFilter();
	Matrix2D Pk;
	Matrix2D Kk;
	Vec2D xk;

	void computeNewValues(Vec2D yk);

	// Add Code here
};




#endif /* FILTER_H_ */
