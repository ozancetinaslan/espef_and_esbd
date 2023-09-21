

#ifndef PHYSSOLVER_H
#define PHYSSOLVER_H


//PHYSSolver Declaration
//-----------------------------------------------


//include
//-----------------------------------------------
#include <iostream>
#include <stdlib.h>
#include <omp.h>
#include "PHYSSolverTypes.h"




//PHYSSolver 
//-----------------------------------------------
class PHYSSolver
{
public:
	//Attributes
	//Methods
	PHYSSolver(){}
	~PHYSSolver(){}
	
	virtual void solve(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData) = 0;

	virtual solver_t getType(){return BASE;}
	

};



#endif