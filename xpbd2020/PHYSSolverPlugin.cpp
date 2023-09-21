

//PHYSSolverPlugin Implementation
//-----------------------------------------------


//include
//-----------------------------------------------
#include "PHYSSolverPlugin.h"


//PHYSSolverPlugin 
//-----------------------------------------------

//Attributes

//Methods
PHYSSolverPlugin::PHYSSolverPlugin()
	:solver(0)//Initialisation list for member vars
{}

PHYSSolverPlugin::~PHYSSolverPlugin(){}

//solve
void PHYSSolverPlugin::solve(solver_t solverType)
{
	
	
	//init?
	//-----------------------------------------------
	if(!solver)
	{
		//create new solver
		initSolver(solverType);
	}
	else if(solverType != solver->getType())
	{
		//delete old solver
		delete solver;
		//create new solver
		initSolver(solverType);
	}

	//solve
	//-----------------------------------------------
	//CPU
	if(solver->getType() == 1)
		solver->solve(staticSolverData, dynamicSolverData);
}


//initSolver
void PHYSSolverPlugin::initSolver(solver_t solverType)
{
	//create solver
	switch(solverType)
	{
		case CPU:
			//if not CPU version
			solver = new PHYSSolverCPU();
			break;
		default:
			break;
	}
}

//is_static_solver_data_initialized
bool PHYSSolverPlugin::is_static_solver_data_initialized()
{
	//check pVertexPositionList
	if (staticSolverData.pVertexPositionList)
		return true;
	return false;
}