

//PHYSSolverVector Implementation
//-----------------------------------------------


//include
//-----------------------------------------------
#include "PHYSSolverVector.h"


//PHYSSolver
//-----------------------------------------------

//Attributes


//Methods

//Default constructor
PHYSSolverVector::PHYSSolverVector()
	:x(0.0),y(0.0),z(0.0),w(1.0)
{}

//Explicit constructor 1
PHYSSolverVector::PHYSSolverVector(float aX, float aY, float aZ)
	:x(aX),y(aY),z(aZ),w(1.0)
{}

//Explicit constructor 2
PHYSSolverVector::PHYSSolverVector(float e1, float e2, float e3, float e4)
	: a(e1), b(e2), c(e3), d(e4), w(1.0)
{}

//Destructor
PHYSSolverVector::~PHYSSolverVector(){}

//Arithmetic Operator Overload: +
PHYSSolverVector PHYSSolverVector::operator+(PHYSSolverVector rhsVec)
{
	float tempX = x + rhsVec.x;
	float tempY = y + rhsVec.y;
	float tempZ = z + rhsVec.z;
	return PHYSSolverVector(tempX,tempY,tempZ);
}

//Arithmetic Operator Overload: -
PHYSSolverVector PHYSSolverVector::operator-(PHYSSolverVector rhsVec)
{
	float tempX = x - rhsVec.x;
	float tempY = y - rhsVec.y;
	float tempZ = z - rhsVec.z;
	return PHYSSolverVector(tempX,tempY,tempZ);
}

//Arithmetic Operator Overload: *
PHYSSolverVector PHYSSolverVector::operator*(const float &rhsScalar)
{
	float tempX = x * rhsScalar;
	float tempY = y * rhsScalar;
	float tempZ = z * rhsScalar;
	return PHYSSolverVector(tempX,tempY,tempZ);
}

//Arithmetic Operator Overload: /
PHYSSolverVector PHYSSolverVector::operator/(const float &rhsScalar)
{
	float tempX = x / rhsScalar;
	float tempY = y / rhsScalar;
	float tempZ = z / rhsScalar;
	return PHYSSolverVector(tempX,tempY,tempZ);
}

//Comparison Operator ==
bool PHYSSolverVector::operator==(PHYSSolverVector &rhsVec)
{
	return (x == rhsVec.x && y == rhsVec.y && z == rhsVec.z);
}

//Negative Comparison Operator
bool PHYSSolverVector::operator!=(PHYSSolverVector &rhsVec)
{
	return (x != rhsVec.x && y != rhsVec.y && z != rhsVec.z);
}

//length
float PHYSSolverVector::length()
{
	float length = float(sqrt((x*x) + (y*y) +(z*z)));
	return length;
}

//normal
PHYSSolverVector PHYSSolverVector::normal()
{
	float length = float(sqrt((x*x) + (y*y) +(z*z)));
	return PHYSSolverVector(x/length, y/length, z/length);
}

//angle
float PHYSSolverVector::angle(PHYSSolverVector &rhsVec)
{
	//dot product
	float dotProduct = dot(rhsVec);

	//rhsVecLength
	float rhsVecLength = rhsVec.length();

	//cosAngle
	float cosAngle = dotProduct / (length() * rhsVecLength);

	//angle
	float angle = acos(cosAngle) * (180.0 / 3.14159265);

	return angle;
}

//dot
float PHYSSolverVector::dot(PHYSSolverVector &rhsVec)
{
	//multiply x,y,z
	float multipliedX = x * rhsVec.x;
	float multipliedY = y * rhsVec.y;
	float multipliedZ = z * rhsVec.z;

	return (multipliedX+multipliedY+multipliedZ);
}

//cross
PHYSSolverVector PHYSSolverVector::cross(PHYSSolverVector &rhsVec)
{
	//crossX
	float crossX = y*rhsVec.z - z*rhsVec.y;
	float crossY = z*rhsVec.x - x*rhsVec.z;
	float crossZ = x*rhsVec.y - y*rhsVec.x;

	return PHYSSolverVector(crossX, crossY, crossZ);
}