

//MPHYSSolverPlugin Implementation
//-----------------------------------------------

//include
//-----------------------------------------------
#include <maya/MFnPlugin.h>
//Solver
#include "mPHYSSolver.h"


//initialize
//-----------------------------------------------
MStatus initializePlugin(MObject obj)
{
	MStatus status;

	//initialize plugin functionset
	MFnPlugin fnPlugin(obj, "Ozan", "1.0", "2017");

	//Solver
	status = fnPlugin.registerNode(MPHYSSolver::typeName, MPHYSSolver::id, &MPHYSSolver::create, &MPHYSSolver::initialize, MPxNode::kLocatorNode, &MPHYSSolver::drawDbClassification);

	//Drawer
	status = MDrawRegistry::registerDrawOverrideCreator(MPHYSSolver::drawDbClassification, MPHYSSolver::drawRegistrantId, MPHYSDrawer::Creator);
	
	return MStatus::kSuccess;
}


//uninitialize
//-----------------------------------------------
MStatus uninitializePlugin(MObject obj)
{
	MStatus status;

	//initialize plugin functionset
	MFnPlugin fnPlugin(obj);

	//Drawer
	status = MDrawRegistry::deregisterDrawOverrideCreator(MPHYSSolver::drawDbClassification, MPHYSSolver::drawRegistrantId);

	//Solver
	status = fnPlugin.deregisterNode(MPHYSSolver::id);
	
	return MStatus::kSuccess;
}