

#ifndef PHYSSOLVERCPU_H
#define PHYSSOLVERCPU_H


//PHYSSolverCPU Declaration
//-----------------------------------------------


//include
//-----------------------------------------------
#include "PHYSSolver.h"


//PHYSSolverCPU 
//-----------------------------------------------
class PHYSSolverCPU: public PHYSSolver
{
public:
	//Attributes
	//Methods
	PHYSSolverCPU();
	~PHYSSolverCPU();
	
	void solve(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	
	void verletIntegration(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void velocityCorrection(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void satisfyConstraints(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	
	void collisionConstraints(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void collisionConstraintGroundplane(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void collisionConstraintSpheres(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void collisionConstraintConvex(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	
	void stretchingConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void centerofMassComputation(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);

	void greenStrainConstraintTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void greenStrainConstraintModifiedTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void greenStrainConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void greenStrainConstraintModifiedTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void areaStrainConstraintTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void volumeStrainConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);

	void stVenantKirchhoffConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void neoHookeanConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);

	void hookeSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void stvkSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void morsePotentialConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void exponentialHookeSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void exponentialStvkSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);

	void exponentialGreenStrainConstraintTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void exponentialGreenStrainConstraintModifiedTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void exponentialGreenStrainConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
	void exponentialGreenStrainConstraintModifiedTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);
			
	void positionConstraints(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData);

	solver_t getType(){return CPU;};

};



#endif