

#ifndef MPHYSSOLVER_H
#define MPHYSSOLVER_H

//MPHYSSolver Declaration
//-----------------------------------------------


//include
//-----------------------------------------------
#include <maya/MPxLocatorNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDagPath.h>
#include <maya/MGlobal.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MMatrix.h>
#include <maya/MFloatVector.h>
#include <maya/MIntArray.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MTime.h>
#include <maya/MItMeshEdge.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MHardwareRenderer.h>
#include <maya/MGLFunctionTable.h>
#include <maya/MFnMatrixData.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>

#include <maya/MDrawRegistry.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>

#include <vector>
#include <algorithm>

#include "PHYSSolverPlugin.h"

using namespace MHWRender;

//MPHYSSolver
//-----------------------------------------------
class MPHYSSolver: public MPxLocatorNode , public PHYSSolverPlugin
{
public:
	//Attributes

	static MTypeId id;
	static MString typeName;

	static MObject aInputGeo;
	static MObject aOutputGeo;

	static MObject aStartFrame;
	static MObject aCurrentFrame;

	static MObject aGravity;

	static MObject aWindDirection;

	static MObject aDeltaTime;

	static MObject aRepetitions;

	static MObject aAlphaCompliance;

	static MObject aPositionConstraint;
	static MObject aPositionConstraintActive;
	static MObject aPositionConstraintVertexIndex;
	static MObject aPositionConstraintCoordinate;

	static MObject aCollisionConstraintGroundplaneActive;
	static MObject aCollisionConstraintGroundplaneHeight;
	static MObject aCollisionConstraintGroundplaneVisible;
	static MObject aCollisionConstraintGroundplaneDispSize;

	static MObject aCollisionConstraint;
	static MObject aCollisionConstraintActive;
	static MObject aCollisionConstraintType;
	static MObject aCollisionConstraintGeoMatrix;
	static MObject aCollisionConstraintGeo;

	static MObject aDrawMeshlinesActive;
	
	static MObject aDrag;

	static MObject aSolverType;

	static MObject a_thread_count;
	static MObject a_omp;

	static MObject aPressureStiffness;

	static MObject aMeshThickness;
	
	static MObject aStrainStiffness11;
	static MObject aStrainStiffness22;
	static MObject aStrainStiffness33;
	static MObject aStrainStiffness12;
	static MObject aStrainStiffness13;
	static MObject aStrainStiffness23;

	static MObject aYoungsModulus;
	static MObject aPoissonRatio;

	static MObject aSpringStiffness;

	static MObject aMaterialConstant1;
	static MObject aMaterialConstant2;
	static MObject aMaterialConstant3;
	static MObject aMaterialConstant4;

	static MObject aYoungsModuliKWeft;
	static MObject aYoungsModuliKWarp;
	static MObject aYoungsModuliKShear;
	static MObject aPoissonRatioVWeft;
	static MObject aPoissonRatioVWarp;

	static MObject aIsometricBendingStiffness;

	static MObject aMaterialStiffnessShapeMatching;

	static MString drawDbClassification;
	static MString drawRegistrantId;
	

	//Methods
	MPHYSSolver();
	~MPHYSSolver();
	static void* create();
	static MStatus initialize();
	virtual MStatus compute(const MPlug &plug, MDataBlock &data);
		
	
	//staticSolverData
	void setStaticSolverData(void* proprietaryData);
	void setVertexCount(MDataBlock &data);
	void setUVCount(MDataBlock &data);
	void setVertexPositionList(MDataBlock &data);
	void setVertexOldpositionList(MDataBlock &data);
	void setVertexInitialpositionList(MDataBlock &data);
	void setVelocityList(MDataBlock &data);
	void setOldVelocityList(MDataBlock &data);
	void setEdgeVertexIndexListAndCount(MDataBlock &data);
	void setFaceVertexIndexListAndCount(MDataBlock &data);
	void setUVPositionList(MDataBlock &data);
	void setEdgeRestlengthList(MDataBlock &data);
	void setRestVolumeList(MDataBlock &data);
	void setMaterialCoordinatesForTriangle(MDataBlock &data);

		
	//dynamicSolverData
	void setDynamicSolverData(void* proprietaryData);
	void resetTotalLagrangeMultiplierList(MDataBlock &data);
	void setVertexForceList(MDataBlock &data);
	void setPositionConstraintCount(MDataBlock &data);
	void setPositionConstraintActiveList(MDataBlock &data);
	void setPositionConstraintVertexIndexList(MDataBlock &data);
	void setPositionConstraintCoordinateList(MDataBlock &data);
	void setCollisionConstraintSpheresCountAndVecUpVecDownList(MDataBlock &data);
	void setCollisionConstraintConvexCountAndTriangleCountList(MDataBlock &data);
	void setCollisionConstraintConvexTriangleVertexPositionList(MDataBlock &data);
	void set_thread_count_and_omp(MDataBlock&);

	//drawData
	void setDrawData(MDataBlock &data);
		
	//MPHYSSolver
	bool attributeConnected(MString attributeName);
	void setOutputGeo(MDataBlock &data);
	
	drawData_t drawData;

private:
	float lastFrame;
};


class MPHYSDrawerData : public MUserData
{
public:
	MPHYSDrawerData();
	virtual ~MPHYSDrawerData() {};
	MColor collisionGroundPlaneColor;
	MColor meshEdgeColor;
};


class MPHYSDrawer : public MPxDrawOverride, public MPHYSSolver
{
public:
	static MPxDrawOverride* Creator(const MObject& obj)
	{
		return new MPHYSDrawer(obj);
	}

	virtual ~MPHYSDrawer();

	virtual DrawAPI supportedDrawAPIs() const;

	virtual MUserData* prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MFrameContext& frameContext, MUserData* oldData);

	virtual bool hasUIDrawables() const { return true; }

	virtual void addUIDrawables(const MDagPath& objPath, MUIDrawManager& drawManager, const MFrameContext& frameContext, const MUserData* data);

private:
	MPHYSDrawer(const MObject& obj);
};
#endif