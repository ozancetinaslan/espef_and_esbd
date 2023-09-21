

#ifndef PHYSSOLVERPLUGIN_H
#define PHYSSOLVERPLUGIN_H

//PHYSSolverPlugin Declaration
//-----------------------------------------------


//include
//-----------------------------------------------
#include "PHYSSolverCPU.h"



//PHYSSolverPlugin 
//-----------------------------------------------
class PHYSSolverPlugin
{
public:
	//Attributes
	staticSolverData_t staticSolverData;
	dynamicSolverData_t dynamicSolverData;
	
	//Methods
	PHYSSolverPlugin();
	~PHYSSolverPlugin();
	
	//PHYSSolverPlugin
	void solve(solver_t solverType);
	void initSolver(solver_t solverType);
	solver_t getType(){return solver->getType();};
	
	//staticSolverData
	staticSolverData_t getStaticSolverData(){return staticSolverData;};
	virtual void setStaticSolverData(void* proprietaryData) = 0;
	bool is_static_solver_data_initialized();
		
	//dynamicSolverData
	dynamicSolverData_t getDynamicSolverData(){return dynamicSolverData;};
	virtual void setDynamicSolverData(void* proprietaryData) = 0;
	
	
	
private:
	//Attributes
	PHYSSolver* solver;
	//Methods

};



#endif