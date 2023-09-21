

#ifndef PHYSSOLVERVECTOR_H
#define PHYSSOLVERVECTOR_H


//PHYSSolverVector Declaration
//-----------------------------------------------


//include
//-----------------------------------------------
#include <math.h>


//PHYSSolverVector
//-----------------------------------------------
class PHYSSolverVector
{
public:
	//Attributes
	float x, y, z, w;
	float a, b, c, d;
	//Methods
	PHYSSolverVector();
	PHYSSolverVector(float aX, float aY, float aZ);
	PHYSSolverVector(float e1, float e2, float e3, float e4);
	~PHYSSolverVector();
	
	PHYSSolverVector operator+(PHYSSolverVector rhsVec);
	PHYSSolverVector operator-(PHYSSolverVector rhsVec);
	PHYSSolverVector operator*(const float &rhsScalar);
	PHYSSolverVector operator/(const float &rhsScalar);
	bool operator==(PHYSSolverVector &rhsVec);
	bool operator!=(PHYSSolverVector &rhsVec);

	float length();
	PHYSSolverVector normal();
	float angle(PHYSSolverVector &rhsVec);
	float dot(PHYSSolverVector &rhsVec);
	PHYSSolverVector cross(PHYSSolverVector &rhsVec);
};

#endif