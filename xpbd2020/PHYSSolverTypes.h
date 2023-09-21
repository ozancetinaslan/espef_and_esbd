

#ifndef PHYSSOLVERTYPES_H
#define PHYSSOLVERTYPES_H

//include
//-----------------------------------------------

#include "PHYSSolverVector.h"


//typedef for PHYSSolverVector
typedef PHYSSolverVector vcsVector;

//declarations
//-----------------------------------------------

//staticSolverData_t 
//-----------------------------------------------
struct staticSolverData_t
{
	int vertexCount;
	int edgeCount;
	int faceCount;
	int uvCount;
	
	vcsVector initialCenterofMass;
	vcsVector* pCenterofMass;
	vcsVector* pVertexPositionList;
	vcsVector* pVertexOldpositionList;
	vcsVector* pVertexInitialpositionList;
	vcsVector* pVelocityList;
	vcsVector* pOldVelocityList;
	vcsVector* pEdgeVertexIndexList;
	vcsVector* pFaceVertexIndexList;
	vcsVector* pUVPositionList;
	vcsVector* pFaceUVIndexList;
	vcsVector* pMaterialCoordinateC1List;
	vcsVector* pMaterialCoordinateC2List;
	vcsVector* pMaterialCoordinateInvC1List;
	vcsVector* pMaterialCoordinateInvC2List;
	
	
	float* pEdgeRestlengthList;
	float* pRestVolumeList;

	float relErrorValue;
	float potEneg;
					
	//Methods
	staticSolverData_t() : pVertexPositionList(0), pVertexOldpositionList(0), pVertexInitialpositionList(0), pVelocityList(0), pEdgeVertexIndexList(0), pFaceVertexIndexList(0), pUVPositionList(0), pFaceUVIndexList(0), pEdgeRestlengthList(0),
							pRestVolumeList(0), pCenterofMass(0),
							pMaterialCoordinateC1List(0), pMaterialCoordinateC2List(0), pMaterialCoordinateInvC1List(0), pMaterialCoordinateInvC2List(0), pOldVelocityList(0) {};
	~staticSolverData_t(){};
};


//dynamicSolverData_t 
//-----------------------------------------------
struct dynamicSolverData_t
{
	//Attributes
	float drag;
	int collisionGroundplaneActive;
	float groundplaneHeight;
	vcsVector* pVertexForceListGravity;
	vcsVector* pVertexForceListWind;
	float deltaTime;
	int repetitions;
	float alphaCompliance;
	int positionConstraintCount;
	int* pPositionConstraintActiveList;
	int* pPositionConstraintVertexIndexList;
	vcsVector* pPositionConstraintCoordinateList;
	int collisionConstraintSpheresCount;
	vcsVector* pCollisionConstraintSpheresVecUpVecDownList;
	int collisionConstraintConvexCount;
	int* pCollisionConstraintConvexTriangleCountList;
	int collisionConstraintConvexTriangleVertexCount;
	vcsVector* pCollisionConstraintConvexTriangleVertexPositionList;
	int thread_count;
	int omp;
	float pressureStiffness;
	float meshThickness;
	float strainStiffness11;
	float strainStiffness22;
	float strainStiffness33;
	float strainStiffness12;
	float strainStiffness13;
	float strainStiffness23;
	float youngsModulus;
	float poissonRatio;
	float springStiffness;
	float materialConstant1;
	float materialConstant2;
	float materialConstant3;
	float materialConstant4;
	float youngsModuliKWeft;
	float youngsModuliKWarp;
	float youngsModuliKShear;
	float poissonRatioVWeft;
	float poissonRatioVWarp;
	
	float* totalLagrangeMultiplierStretching1;
	float* totalLagrangeMultiplierStretching2;
	float* totalLagrangeMultiplierVolume1;
	float* totalLagrangeMultiplierVolume2;
	float* totalLagrangeMultiplierVolume3;
	float* totalLagrangeMultiplierStrain1_11;
	float* totalLagrangeMultiplierStrain2_11;
	float* totalLagrangeMultiplierStrain3_11;
	float* totalLagrangeMultiplierStrain1_22;
	float* totalLagrangeMultiplierStrain2_22;
	float* totalLagrangeMultiplierStrain3_22;
	float* totalLagrangeMultiplierStrain1_33;
	float* totalLagrangeMultiplierStrain2_33;
	float* totalLagrangeMultiplierStrain3_33;
	float* totalLagrangeMultiplierStrain1_12;
	float* totalLagrangeMultiplierStrain2_12;
	float* totalLagrangeMultiplierStrain3_12;
	float* totalLagrangeMultiplierStrain1_13;
	float* totalLagrangeMultiplierStrain2_13;
	float* totalLagrangeMultiplierStrain3_13;
	float* totalLagrangeMultiplierStrain1_23;
	float* totalLagrangeMultiplierStrain2_23;
	float* totalLagrangeMultiplierStrain3_23;
	float* totalLagrangeMultiplierStrainArea1;
	float* totalLagrangeMultiplierStrainArea2;
	float* totalLagrangeMultiplierStrainArea3;
	float* totalLagrangeMultiplierStrainVolume1;
	float* totalLagrangeMultiplierStrainVolume2;
	float* totalLagrangeMultiplierStrainVolume3;
	float* totalLagrangeMultiplierSTVK1;
	float* totalLagrangeMultiplierSTVK2;
	float* totalLagrangeMultiplierSTVK3;
	float* totalLagrangeMultiplierNeoH1;
	float* totalLagrangeMultiplierNeoH2;
	float* totalLagrangeMultiplierNeoH3;
	float* totalLagrangeMultiplierHookeSpring1;
	float* totalLagrangeMultiplierHookeSpring2;
	float* totalLagrangeMultiplierSTVKSpring1;
	float* totalLagrangeMultiplierSTVKSpring2;
	float* totalLagrangeMultiplierMorsePot1;
	float* totalLagrangeMultiplierMorsePot2;
	float* totalLagrangeMultiplierExponentialHookeSpring1;
	float* totalLagrangeMultiplierExponentialHookeSpring2;
	float* totalLagrangeMultiplierExponentialSTVKSpring1;
	float* totalLagrangeMultiplierExponentialSTVKSpring2;
	
	//Methods
	dynamicSolverData_t() : pVertexForceListGravity(0), pPositionConstraintActiveList(0), pPositionConstraintVertexIndexList(0), pPositionConstraintCoordinateList(0), pCollisionConstraintSpheresVecUpVecDownList(0),
							pCollisionConstraintConvexTriangleCountList(0), pCollisionConstraintConvexTriangleVertexPositionList(0), totalLagrangeMultiplierStretching1(0), totalLagrangeMultiplierStretching2(0),
							totalLagrangeMultiplierVolume1(0), totalLagrangeMultiplierVolume2(0), totalLagrangeMultiplierVolume3(0),
							totalLagrangeMultiplierStrain1_11(0), totalLagrangeMultiplierStrain2_11(0), totalLagrangeMultiplierStrain3_11(0), 
							totalLagrangeMultiplierStrain1_22(0), totalLagrangeMultiplierStrain2_22(0), totalLagrangeMultiplierStrain3_22(0), totalLagrangeMultiplierStrain1_33(0), totalLagrangeMultiplierStrain2_33(0), 
							totalLagrangeMultiplierStrain3_33(0), totalLagrangeMultiplierStrain1_12(0), totalLagrangeMultiplierStrain2_12(0), totalLagrangeMultiplierStrain3_12(0), totalLagrangeMultiplierStrain1_13(0),
							totalLagrangeMultiplierStrain2_13(0), totalLagrangeMultiplierStrain3_13(0), totalLagrangeMultiplierStrain1_23(0), totalLagrangeMultiplierStrain2_23(0), totalLagrangeMultiplierStrain3_23(0),
							totalLagrangeMultiplierStrainArea1(0), totalLagrangeMultiplierStrainArea2(0), totalLagrangeMultiplierStrainArea3(0), totalLagrangeMultiplierStrainVolume1(0), totalLagrangeMultiplierStrainVolume2(0), 
							totalLagrangeMultiplierStrainVolume3(0), totalLagrangeMultiplierSTVK1(0), 
							totalLagrangeMultiplierSTVK2(0), totalLagrangeMultiplierSTVK3(0),
							totalLagrangeMultiplierNeoH1(0), totalLagrangeMultiplierNeoH2(0), totalLagrangeMultiplierNeoH3(0), 
							pVertexForceListWind(0), totalLagrangeMultiplierHookeSpring1(0),
							totalLagrangeMultiplierHookeSpring2(0), totalLagrangeMultiplierSTVKSpring1(0), totalLagrangeMultiplierSTVKSpring2(0),
							totalLagrangeMultiplierMorsePot1(0), totalLagrangeMultiplierMorsePot2(0), totalLagrangeMultiplierExponentialHookeSpring1(0), totalLagrangeMultiplierExponentialHookeSpring2(0), 
							totalLagrangeMultiplierExponentialSTVKSpring1(0), totalLagrangeMultiplierExponentialSTVKSpring2(0) {};
	~dynamicSolverData_t(){};
};


//drawData_t 
//-----------------------------------------------
struct drawData_t
{
	//Attrs
	int collisionConstraintGroundPlaneVisible;
	float collisionConstraintGroundplaneDispSize;
	float collisionConstraintGroundplaneHeight;
	int drawMeshlinesActive;
};


//solver_t
//-----------------------------------------------
enum solver_t
{
	BASE = 0,
	CPU
};

//grid_size_t
//-----------------------------------------------
enum grid_size_t
{
	VERTEXCOUNT = 0,
	EDGECOUNT,
	FACECOUNT
};

#endif