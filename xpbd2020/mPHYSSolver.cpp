

//MPHYSSolver Implementation
//-----------------------------------------------


//include
//-----------------------------------------------
#include "mPHYSSolver.h"

using namespace std;


//MPHYSSolver
//-----------------------------------------------

//Attributes
MTypeId MPHYSSolver::id(0x70010);
MString MPHYSSolver::typeName("PHYSSolver");

MObject MPHYSSolver::aInputGeo;
MObject MPHYSSolver::aOutputGeo;

MObject MPHYSSolver::aStartFrame;
MObject MPHYSSolver::aCurrentFrame;

MObject MPHYSSolver::aGravity;

MObject MPHYSSolver::aWindDirection;

MObject MPHYSSolver::aDeltaTime;

MObject MPHYSSolver::aRepetitions;

MObject MPHYSSolver::aAlphaCompliance;

MObject MPHYSSolver::aPositionConstraint;
MObject MPHYSSolver::aPositionConstraintActive;
MObject MPHYSSolver::aPositionConstraintVertexIndex;
MObject MPHYSSolver::aPositionConstraintCoordinate;

MObject MPHYSSolver::aCollisionConstraintGroundplaneActive;
MObject MPHYSSolver::aCollisionConstraintGroundplaneHeight;
MObject MPHYSSolver::aCollisionConstraintGroundplaneVisible;
MObject MPHYSSolver::aCollisionConstraintGroundplaneDispSize;

MObject MPHYSSolver::aCollisionConstraint;
MObject MPHYSSolver::aCollisionConstraintActive;
MObject MPHYSSolver::aCollisionConstraintType;
MObject MPHYSSolver::aCollisionConstraintGeoMatrix;
MObject MPHYSSolver::aCollisionConstraintGeo;

MObject MPHYSSolver::aDrawMeshlinesActive;

MObject MPHYSSolver::aDrag;

MObject MPHYSSolver::aSolverType;

MObject MPHYSSolver::a_thread_count;
MObject MPHYSSolver::a_omp;

MObject MPHYSSolver::aPressureStiffness;

MObject MPHYSSolver::aMeshThickness;

MObject MPHYSSolver::aStrainStiffness11;
MObject MPHYSSolver::aStrainStiffness22;
MObject MPHYSSolver::aStrainStiffness33;
MObject MPHYSSolver::aStrainStiffness12;
MObject MPHYSSolver::aStrainStiffness13;
MObject MPHYSSolver::aStrainStiffness23;

MObject MPHYSSolver::aYoungsModulus;
MObject MPHYSSolver::aPoissonRatio;

MObject MPHYSSolver::aSpringStiffness;

MObject MPHYSSolver::aMaterialConstant1;
MObject MPHYSSolver::aMaterialConstant2;
MObject MPHYSSolver::aMaterialConstant3;
MObject MPHYSSolver::aMaterialConstant4;

MObject MPHYSSolver::aYoungsModuliKWeft;
MObject MPHYSSolver::aYoungsModuliKWarp;
MObject MPHYSSolver::aYoungsModuliKShear;
MObject MPHYSSolver::aPoissonRatioVWeft;
MObject MPHYSSolver::aPoissonRatioVWarp;

MObject MPHYSSolver::aIsometricBendingStiffness;

MObject MPHYSSolver::aMaterialStiffnessShapeMatching;

MString MPHYSSolver::drawDbClassification("drawdb/geometry/PHYSSolver");
MString MPHYSSolver::drawRegistrantId("PHYSDrawer");


//Methods
MPHYSSolver::MPHYSSolver(){}
MPHYSSolver::~MPHYSSolver(){}


//create
//-----------------------------------------------
void* MPHYSSolver::create()
{
	return new MPHYSSolver();
}


//initialize
//-----------------------------------------------
MStatus MPHYSSolver::initialize()
{

	//MFnSets
	MFnNumericAttribute nAttr;
	MFnUnitAttribute uAttr;
	MFnTypedAttribute tAttr;
	MFnMatrixAttribute mAttr;
	MFnEnumAttribute eAttr;
	MFnCompoundAttribute cAttr;

	//aInputGeo
	aInputGeo = tAttr.create("inputGeo", "inputGeo", MFnData::kMesh);
	tAttr.setReadable(false);
	addAttribute(aInputGeo);

	//aOutputGeo
	aOutputGeo = tAttr.create("outputGeo", "outputGeo", MFnData::kMesh);
	tAttr.setWritable(false);
	tAttr.setStorable(false);
	addAttribute(aOutputGeo);

	//aStartFrame
	aStartFrame = nAttr.create("startFrame", "startFrame", MFnNumericData::kFloat, 1.0);
	addAttribute(aStartFrame);

	//aCurrentFrame
	aCurrentFrame = nAttr.create("currentFrame", "currentFrame", MFnNumericData::kFloat);
	addAttribute(aCurrentFrame);

	//aGravity
	aGravity = nAttr.createPoint("gravity", "gravity");
	nAttr.setKeyable(true);
	addAttribute(aGravity);

	//aWindDirection
	aWindDirection = nAttr.createPoint("windDirection", "windDirection");
	nAttr.setKeyable(true);
	addAttribute(aWindDirection);

	//aDeltaTime
	aDeltaTime = nAttr.create("deltaTime", "deltaTime", MFnNumericData::kFloat, 0.042);
	nAttr.setMin(0.005);
	nAttr.setMax(0.16);
	addAttribute(aDeltaTime);

	//aRepetitions
	aRepetitions = nAttr.create("repetitions", "repetitions", MFnNumericData::kInt, 1);
	nAttr.setKeyable(true);
	addAttribute(aRepetitions);

	//aAlphaCompliance
	aAlphaCompliance = nAttr.create("alphaCompliance", "alphaCompliance", MFnNumericData::kFloat, 0.00001);
	nAttr.setKeyable(true);
	addAttribute(aAlphaCompliance);

	//aPositionConstraintActive
	aPositionConstraintActive = eAttr.create("positionConstraintActive", "positionConstraintActive");
	eAttr.addField("Inactive", 0);
	eAttr.addField("Active", 1);
	
	//aPositionConstraintVertexIndex
	aPositionConstraintVertexIndex = nAttr.create("positionConstraintVertexIndex", "positionConstraintVertexIndex", MFnNumericData::kInt, 0);
	nAttr.setKeyable(true);

	//aPositionConstraintCoordinate
	aPositionConstraintCoordinate = nAttr.createPoint("positionConstraintCoordinate", "positionConstraintCoordinate");
	nAttr.setKeyable(true);

	//aPositionConstraint
	aPositionConstraint = cAttr.create("positionConstraint", "positionConstraint");
	cAttr.setArray(true);
	cAttr.addChild(aPositionConstraintActive);
	cAttr.addChild(aPositionConstraintVertexIndex);
	cAttr.addChild(aPositionConstraintCoordinate);
	addAttribute(aPositionConstraint);

	//aCollisionConstraintGroundplaneActive
	aCollisionConstraintGroundplaneActive = nAttr.create("collisionConstraintGroundplaneActive", "collisionConstraintGroundplaneActive", MFnNumericData::kBoolean, 0);
	nAttr.setStorable(true);
	addAttribute(aCollisionConstraintGroundplaneActive);
	
	//aCollisionConstraintGroundplaneHeight
	aCollisionConstraintGroundplaneHeight = nAttr.create("collisionConstraintGroundplaneHeight", "collisionConstraintGroundplaneHeight", MFnNumericData::kFloat, 0.0);
	nAttr.setKeyable(true);
	addAttribute(aCollisionConstraintGroundplaneHeight);
	
	//aCollisionConstraintGroundplaneVisible
	aCollisionConstraintGroundplaneVisible = nAttr.create("collisionConstraintGroundplaneVisible", "collisionConstraintGroundplaneVisible", MFnNumericData::kBoolean, 0);
	nAttr.setStorable(true);
	addAttribute(aCollisionConstraintGroundplaneVisible);
	
	//aCollisionConstraintGroundplaneDispSize
	aCollisionConstraintGroundplaneDispSize = nAttr.create("collisionConstraintGroundplaneDispSize", "collisionConstraintGroundplaneDispSize", MFnNumericData::kFloat, 10.0);
	nAttr.setKeyable(true);
	nAttr.setHidden(false);
	addAttribute(aCollisionConstraintGroundplaneDispSize);

	//aCollisionConstraintActive
	aCollisionConstraintActive = eAttr.create("collisionConstraintActive", "collisionConstraintActive", 0);
	eAttr.addField("Inactive", 0);
	eAttr.addField("Active", 1);

	//aCollisionConstraintType
	aCollisionConstraintType = eAttr.create("collisionConstraintType", "collisionConstraintType", 0);
	eAttr.addField("Sphere", 0);
	eAttr.addField("ConvexPrimitive", 1);

	//aCollisionConstraintGeoMatrix
	aCollisionConstraintGeoMatrix = mAttr.create("collisionConstraintGeoMatrix", "collisionConstraintGeoMatrix");
	mAttr.setReadable(false);

	//aCollisionConstraintGeo
	aCollisionConstraintGeo = tAttr.create("collisionConstraintGeo", "collisionConstraintGeo", MFnData::kMesh);
	tAttr.setWritable(true);
	tAttr.setReadable(false);

	//aCollisionConstraint
	aCollisionConstraint = cAttr.create("collisionConstraint", "collisionConstraint");
	cAttr.setArray(true);
	cAttr.addChild(aCollisionConstraintActive);
	cAttr.addChild(aCollisionConstraintType);
	cAttr.addChild(aCollisionConstraintGeoMatrix);
	cAttr.addChild(aCollisionConstraintGeo);
	addAttribute(aCollisionConstraint);

	//aDrawMeshlinesActive
	aDrawMeshlinesActive = nAttr.create("drawMeshlinesActive", "drawMeshlinesActive", MFnNumericData::kBoolean, 0);
	nAttr.setStorable(true);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aDrawMeshlinesActive);

	//aDrag
	aDrag = nAttr.create("drag", "drag", MFnNumericData::kFloat, 3.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10.0);
	addAttribute(aDrag);

	//AttributeAffects
	attributeAffects(aCurrentFrame, aOutputGeo);

	//aSolverType
	aSolverType = eAttr.create("solverType", "solverType", 1);
	eAttr.addField("CPU", 1);
	addAttribute(aSolverType);

	//a_thread_count
	a_thread_count = nAttr.create("thread_count", "thread_count", MFnNumericData::kInt, 512);
	nAttr.setMin(1);
	nAttr.setMax(1024);
	addAttribute(a_thread_count);

	//a_omp
	a_omp = eAttr.create("omp", "omp", 0);
	eAttr.addField("OMP Support Off", 0);
	eAttr.addField("OMP Support On", 1);
	addAttribute(a_omp);

	//aPressureStiffness
	aPressureStiffness = nAttr.create("pressureStiffness", "pressureStiffness", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(5.0);
	addAttribute(aPressureStiffness);

	//aMeshThickness
	aMeshThickness = nAttr.create("meshThickness", "meshThickness", MFnNumericData::kFloat, 0.0001);
	nAttr.setMin(0.0);
	nAttr.setMax(0.001);
	addAttribute(aMeshThickness);

	//aStrainStiffness11
	aStrainStiffness11 = nAttr.create("strainStiffness11", "strainStiffness11", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aStrainStiffness11);

	//aStrainStiffness22
	aStrainStiffness22 = nAttr.create("strainStiffness22", "strainStiffness22", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aStrainStiffness22);

	//aStrainStiffness33
	aStrainStiffness33 = nAttr.create("strainStiffness33", "strainStiffness33", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aStrainStiffness33);

	//aStrainStiffness12
	aStrainStiffness12 = nAttr.create("strainStiffness12", "strainStiffness12", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aStrainStiffness12);

	//aStrainStiffness13
	aStrainStiffness13 = nAttr.create("strainStiffness13", "strainStiffness13", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aStrainStiffness13);

	//aStrainStiffness23
	aStrainStiffness23 = nAttr.create("strainStiffness23", "strainStiffness23", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aStrainStiffness23);

	//aYoungsModulus
	aYoungsModulus = nAttr.create("youngsModulus", "youngsModulus", MFnNumericData::kFloat, 1000000.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000000000.0);
	addAttribute(aYoungsModulus);

	//aPoissonRatio
	aPoissonRatio = nAttr.create("poissonRatio", "poissonRatio", MFnNumericData::kFloat, 0.3);
	nAttr.setMin(0.0);
	nAttr.setMax(0.49);
	addAttribute(aPoissonRatio);

	//aSpringStiffness
	aSpringStiffness = nAttr.create("springStiffness", "springStiffness", MFnNumericData::kFloat, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000.0);
	addAttribute(aSpringStiffness);

	//aMaterialConstant1
	aMaterialConstant1 = nAttr.create("materialConstant1", "materialConstant1", MFnNumericData::kFloat, 1000000.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000000000.0);
	addAttribute(aMaterialConstant1);

	//aMaterialConstant2
	aMaterialConstant2 = nAttr.create("materialConstant2", "materialConstant2", MFnNumericData::kFloat, 1000000.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000000000.0);
	addAttribute(aMaterialConstant2);

	//aMaterialConstant3
	aMaterialConstant3 = nAttr.create("materialConstant3", "materialConstant3", MFnNumericData::kFloat, 1000000.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000000000.0);
	addAttribute(aMaterialConstant3);

	//aMaterialConstant4
	aMaterialConstant4 = nAttr.create("materialConstant4", "materialConstant4", MFnNumericData::kFloat, 0.00001);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aMaterialConstant4);

	//aYoungsModuliKWeft
	aYoungsModuliKWeft = nAttr.create("youngsModuliKWeft", "youngsModuliKWeft", MFnNumericData::kFloat, 1000.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000000.0);
	addAttribute(aYoungsModuliKWeft);

	//aYoungsModuliKWarp
	aYoungsModuliKWarp = nAttr.create("youngsModuliKWarp", "youngsModuliKWarp", MFnNumericData::kFloat, 1000.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000000.0);
	addAttribute(aYoungsModuliKWarp);

	//aYoungsModuliKShear
	aYoungsModuliKShear = nAttr.create("youngsModuliKShear", "youngsModuliKShear", MFnNumericData::kFloat, 1000.0);
	nAttr.setMin(0.0);
	nAttr.setMax(10000000.0);
	addAttribute(aYoungsModuliKShear);

	//aPoissonRatioVWeft
	aPoissonRatioVWeft = nAttr.create("poissonRatioVWeft", "poissonRatioVWeft", MFnNumericData::kFloat, 0.3);
	nAttr.setMin(0.0);
	nAttr.setMax(0.99);
	addAttribute(aPoissonRatioVWeft);

	//aPoissonRatioVWarp
	aPoissonRatioVWarp = nAttr.create("poissonRatioVWarp", "poissonRatioVWarp", MFnNumericData::kFloat, 0.3);
	nAttr.setMin(0.0);
	nAttr.setMax(0.99);
	addAttribute(aPoissonRatioVWarp);

	//aIsometricBendingStiffness
	aIsometricBendingStiffness = nAttr.create("isometricBendingStiffness", "isometricBendingStiffness", MFnNumericData::kFloat, 0.2);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	addAttribute(aIsometricBendingStiffness);

	//aMaterialStiffnessShapeMatching
	aMaterialStiffnessShapeMatching = nAttr.create("materialStiffnessShapeMatching", "materialStiffnessShapeMatching", MFnNumericData::kFloat, 0.0001);
	nAttr.setMin(0.0);
	nAttr.setMax(0.001);
	addAttribute(aMaterialStiffnessShapeMatching);

	//Drawing
	attributeAffects(aCollisionConstraintGroundplaneHeight, aOutputGeo);
	attributeAffects(aCollisionConstraintGroundplaneVisible, aOutputGeo);
	attributeAffects(aCollisionConstraintGroundplaneDispSize, aOutputGeo);
	attributeAffects(aDrawMeshlinesActive, aOutputGeo);
	
	return MStatus::kSuccess;
}


//compute
//-----------------------------------------------
MStatus MPHYSSolver::compute(const MPlug &plug, MDataBlock &data)
{

	//get attributes from datablock
	//-----------------------------------------------

	MObject inputGeo = data.inputValue(aInputGeo).asMesh();

	float currentFrame = data.inputValue(aCurrentFrame).asFloat();
	float startFrame = data.inputValue(aStartFrame).asFloat();

	int solverType = data.inputValue(aSolverType).asInt();
	
	//Check if all attrs connected
	//-----------------------------------------------
	if (!attributeConnected(MString("inputGeo")))
	{
		MGlobal::displayInfo("Attribute inputGeo not connected");
	}

	//Check if currentFrame <= startframe
	//-----------------------------------------------
	if (currentFrame <= startFrame)
	{
		//set static solver data
		setStaticSolverData(&data);

		//set dynamic solver data
		setDynamicSolverData(&data);

		//set draw data
		setDrawData(data);

		//set lastFrame
		lastFrame = currentFrame;

		//Set output geo
		setOutputGeo(data);

		//set plug clean
		data.setClean(plug);
	}

	//solve
	//-----------------------------------------------
	else
	{
		//set static solver data if not set
		if (!is_static_solver_data_initialized())
		{
			setStaticSolverData(&data);

		}

		//set draw data
		setDrawData(data);

		//If lastFrame < CurrentFrame then solve
		if (lastFrame < currentFrame)
		{
			//set dynamic solver data
			setDynamicSolverData(&data);

			//solve
			solve(solver_t(solverType));

			//set solverType in interface
			data.outputValue(aSolverType).set(int(getType()));
		}

		//set lastFrame
		lastFrame = currentFrame;

		setOutputGeo(data);

		//set plug clean
		data.setClean(plug);
	}
		
	return MStatus::kSuccess;
}


//staticSolverData
//-----------------------------------------------

//setStaticSolverData
//-----------------------------------------------
void MPHYSSolver::setStaticSolverData(void* proprietaryData)
{
	//cast MDataBlockPtr
	MDataBlock* pData = (MDataBlock*)proprietaryData;

	//Execute setting of member data
	setVertexCount(*pData);
	setUVCount(*pData);
	setVertexPositionList(*pData);
	setVertexOldpositionList(*pData);
	setVertexInitialpositionList(*pData);
	setVelocityList(*pData);
	setOldVelocityList(*pData);
	setEdgeVertexIndexListAndCount(*pData);
	setFaceVertexIndexListAndCount(*pData);
	setUVPositionList(*pData);
	setEdgeRestlengthList(*pData);
	setRestVolumeList(*pData);
	setMaterialCoordinatesForTriangle(*pData);
}


//setVertexCount
//-----------------------------------------------
void MPHYSSolver::setVertexCount(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject inputGeo = data.inputValue(aInputGeo).asMesh();
	
	//FnSet
	MFnMesh fsInputGeo(inputGeo);
	
	//numVertices
	staticSolverData.vertexCount = fsInputGeo.numVertices();
}


//setUVCount
//-----------------------------------------------
void MPHYSSolver::setUVCount(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject inputGeo = data.inputValue(aInputGeo).asMesh();

	//FnSet
	MFnMesh fsInputGeo(inputGeo);

	//numUVs
	staticSolverData.uvCount = fsInputGeo.numUVs();
}


//setVertexPositionList
//-----------------------------------------------
void MPHYSSolver::setVertexPositionList(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject inputGeo = data.inputValue(aInputGeo).asMesh();
	
	//FnSet
	MFnMesh fsInputGeo(inputGeo);
	
	//pointArray
	MPointArray inputGeoPointArray;
	fsInputGeo.getPoints(inputGeoPointArray);

	//vcsVector array
	delete[] staticSolverData.pVertexPositionList;
	staticSolverData.pVertexPositionList = new vcsVector[staticSolverData.vertexCount];

	//fill vcsVectorArray with points in ws
	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		MPoint point = inputGeoPointArray[index];
		
		float x = point.x;
		float y = point.y;
		float z = point.z;
		
		staticSolverData.pVertexPositionList[index] = vcsVector(x, y, z);
	}
}


//setVertexOldpositionList
//-----------------------------------------------
void MPHYSSolver::setVertexOldpositionList(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject inputGeo = data.inputValue(aInputGeo).asMesh();
	
	//FnSet
	MFnMesh fsInputGeo(inputGeo);
	
	//pointArray
	MPointArray inputGeoPointArray;
	fsInputGeo.getPoints(inputGeoPointArray);

	//vcsVector array
	delete[] staticSolverData.pVertexOldpositionList;
	staticSolverData.pVertexOldpositionList = new vcsVector[staticSolverData.vertexCount];

	//fill vcsVectorArray with points in ws
	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		MPoint point = inputGeoPointArray[index];
		
		float x = point.x;
		float y = point.y;
		float z = point.z;
		
		staticSolverData.pVertexOldpositionList[index] = vcsVector(x, y, z);
	}
}


//setVertexInitialpositionList
//-----------------------------------------------
void MPHYSSolver::setVertexInitialpositionList(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject inputGeo = data.inputValue(aInputGeo).asMesh();
	
	//FnSet
	MFnMesh fsInputGeo(inputGeo);
	
	//pointArray
	MPointArray inputGeoPointArray;
	fsInputGeo.getPoints(inputGeoPointArray);

	//vcsVector array
	delete[] staticSolverData.pVertexInitialpositionList;
	staticSolverData.pVertexInitialpositionList = new vcsVector[staticSolverData.vertexCount];

	//fill vcsVectorArray with points in ws
	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		MPoint point = inputGeoPointArray[index];
		
		float x = point.x;
		float y = point.y;
		float z = point.z;
		
		staticSolverData.pVertexInitialpositionList[index] = vcsVector(x, y, z);
	}
}


// setVelocityList
//-----------------------------------------------
void MPHYSSolver::setVelocityList(MDataBlock &data)
{
	//vcsVector array
	delete[] staticSolverData.pVelocityList;
	staticSolverData.pVelocityList = new vcsVector[staticSolverData.vertexCount];
	
	//fill vcsVectorArray with points in ws
	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		float x = 0.0;
		float y = 0.0;
		float z = 0.0;

		staticSolverData.pVelocityList[index] = vcsVector(x, y, z);
	}
}


// setOldVelocityList
//-----------------------------------------------
void MPHYSSolver::setOldVelocityList(MDataBlock &data)
{
	//vcsVector array
	delete[] staticSolverData.pOldVelocityList;
	staticSolverData.pOldVelocityList = new vcsVector[staticSolverData.vertexCount];
	
	//fill vcsVectorArray with points in ws
	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		float x = 0.0;
		float y = 0.0;
		float z = 0.0;

		staticSolverData.pOldVelocityList[index] = vcsVector(x, y, z);
	}
}


//setEdgeVertexIndexListAndCount
//-----------------------------------------------
void MPHYSSolver::setEdgeVertexIndexListAndCount(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject oInputGeo = data.inputValue(aInputGeo).asMesh();
	
	//FnSet
	MFnMesh fsInputGeo(oInputGeo);

	//get edge iterator
	MItMeshEdge itMeshEdges(oInputGeo);

	//delete content of old pointer
	delete[] staticSolverData.pEdgeVertexIndexList;

	//Allocate memory for new ptr
	staticSolverData.pEdgeVertexIndexList = new vcsVector[itMeshEdges.count()];

	//iterate edges
	while (!itMeshEdges.isDone())
	{
		//get vertexIds for current edge
		int2 vertexIds;
		fsInputGeo.getEdgeVertices(itMeshEdges.index(), vertexIds);
		
		//store in edgeVertexIndexList
		staticSolverData.pEdgeVertexIndexList[itMeshEdges.index()] = vcsVector(float(vertexIds[0]), float(vertexIds[1]), 0.0);
		
		//next
		itMeshEdges.next();
	}
	//set edgeCount
	staticSolverData.edgeCount = itMeshEdges.count();
}


//setFaceVertexIndexListAndCountForShear
//-----------------------------------------------
void MPHYSSolver::setFaceVertexIndexListAndCount(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject oInputGeo = data.inputValue(aInputGeo).asMesh();
	
	//FnSet
	MFnMesh fsInputGeo(oInputGeo);

	//get face iterator
	MItMeshPolygon itMeshPolygons(oInputGeo);

	//delete content of old pointer
	delete[] staticSolverData.pFaceVertexIndexList;
	
	//Allocate memory for new ptr
	staticSolverData.pFaceVertexIndexList = new vcsVector[itMeshPolygons.count()];
	
	//iterate faces
	while (!itMeshPolygons.isDone())
	{
		//get vertexIds for current face
		MIntArray vertexIds;
		
		fsInputGeo.getPolygonVertices(itMeshPolygons.index(), vertexIds);
		
		//store in pFaceVertexIndexList
		staticSolverData.pFaceVertexIndexList[itMeshPolygons.index()] = vcsVector(float(vertexIds[0]), float(vertexIds[1]), float(vertexIds[2]));
		
		//next
		itMeshPolygons.next();
	}

	//set faceCount
	staticSolverData.faceCount = itMeshPolygons.count();
}


//setUVPositionList
//-----------------------------------------------
void MPHYSSolver::setUVPositionList(MDataBlock &data)
{
	//Get inputGeo from datablock
	MObject inputGeo = data.inputValue(aInputGeo).asMesh();

	//FnSet
	MFnMesh fsInputGeo(inputGeo);

	//UVArray
	MFloatArray inputGeoUArray;
	MFloatArray inputGeoVArray;
	fsInputGeo.getUVs(inputGeoUArray, inputGeoVArray);

	//vcsVector array
	delete[] staticSolverData.pUVPositionList;
	staticSolverData.pUVPositionList = new vcsVector[staticSolverData.uvCount];

	//fill vcsVectorArray with points in UV
	for (int index = 0; index < staticSolverData.uvCount; index++)
	{
		float x = inputGeoUArray[index];
		float y = inputGeoVArray[index];
		
		staticSolverData.pUVPositionList[index] = vcsVector(x, y, 0.0);
	}

	//get face iterator
	MItMeshPolygon itMeshPolygons(inputGeo);
	
	//delete content of old pointer
	delete[] staticSolverData.pFaceUVIndexList;

	//Allocate memory for new ptr
	staticSolverData.pFaceUVIndexList = new vcsVector[staticSolverData.faceCount];

	int UVId0 = 0, UVId1 = 0, UVId2 = 0;

	//iterate faces
	while (!itMeshPolygons.isDone())
	{
		fsInputGeo.getPolygonUVid(itMeshPolygons.index(), 0, UVId0);
		fsInputGeo.getPolygonUVid(itMeshPolygons.index(), 1, UVId1);
		fsInputGeo.getPolygonUVid(itMeshPolygons.index(), 2, UVId2);

		//store in pFaceVertexIndexList
		staticSolverData.pFaceUVIndexList[itMeshPolygons.index()] = vcsVector(float(UVId0), float(UVId1), float(UVId2));

		//next
		itMeshPolygons.next();
	}
}


//setEdgeRestlengthList
//-----------------------------------------------
void MPHYSSolver::setEdgeRestlengthList(MDataBlock &data)
{
	//delete old ptr content and allocate new memory
	delete[] staticSolverData.pEdgeRestlengthList;
	staticSolverData.pEdgeRestlengthList = new float[staticSolverData.edgeCount];

	//iterate edgeVertexIndexList
	for (int index = 0; index < staticSolverData.edgeCount; index++)
	{
		//Get vertexIndices for edge
		int vertexIndex1 = int(staticSolverData.pEdgeVertexIndexList[index].x);
		int vertexIndex2 = int(staticSolverData.pEdgeVertexIndexList[index].y);
		
		//get length from vcsVectors at indices
		vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
		vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
		
		//compute restlength
		float restLength = (vec1 - vec2).length();
		
		//set in pEdgeRestlengthList
		staticSolverData.pEdgeRestlengthList[index] = restLength;
	}
}


//setRestVolumeList
//-----------------------------------------------
void MPHYSSolver::setRestVolumeList(MDataBlock &data)
{
	//delete old ptr content and allocate new memory
	delete[] staticSolverData.pCenterofMass;
	staticSolverData.pCenterofMass = new vcsVector[1];
	
	//delete old ptr content and allocate new memory
	delete[] staticSolverData.pRestVolumeList;
	staticSolverData.pRestVolumeList = new float[staticSolverData.faceCount];

	vcsVector vertexCenterofMassTotal = vcsVector(0.0, 0.0, 0.0);

	//iterate VertexIndexList
	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		vertexCenterofMassTotal = vertexCenterofMassTotal + staticSolverData.pVertexInitialpositionList[index];
	}

	vcsVector centerofMass = vertexCenterofMassTotal / staticSolverData.vertexCount;

	// set initialCenterofMass
	staticSolverData.initialCenterofMass = vertexCenterofMassTotal / staticSolverData.vertexCount;
	//set pCenterofMass
	staticSolverData.pCenterofMass[0] = centerofMass;

	//iterate faceVertexIndexList
	for (int index = 0; index < staticSolverData.faceCount; index++)
	{
		//Get vertexIndices for face
		int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
		int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
		int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

		//get length from vcsVectors at indices
		vcsVector vec0 = centerofMass;
		vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
		vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
		vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

		//vertex differences with center of mass for fake tet
		vcsVector diffVec1nVec0 = vec1 - vec0;
		vcsVector diffVec2nVec0 = vec2 - vec0;
		vcsVector diffVec3nVec0 = vec3 - vec0;

		//compute restAngle
		float restVolume = ((diffVec1nVec0.cross(diffVec2nVec0)).dot(diffVec3nVec0)) / 6.0;

		//set in pRestVolumeList
		staticSolverData.pRestVolumeList[index] = restVolume;
	}
}


//setMaterialCoordinatesForTriangle
//-----------------------------------------------
void MPHYSSolver::setMaterialCoordinatesForTriangle(MDataBlock &data)
{
	//delete old ptr content and allocate new memory
	delete[] staticSolverData.pMaterialCoordinateC1List;
	staticSolverData.pMaterialCoordinateC1List = new vcsVector[staticSolverData.faceCount];

	delete[] staticSolverData.pMaterialCoordinateC2List;
	staticSolverData.pMaterialCoordinateC2List = new vcsVector[staticSolverData.faceCount];

	//delete old ptr content and allocate new memory
	delete[] staticSolverData.pMaterialCoordinateInvC1List;
	staticSolverData.pMaterialCoordinateInvC1List = new vcsVector[staticSolverData.faceCount];

	delete[] staticSolverData.pMaterialCoordinateInvC2List;
	staticSolverData.pMaterialCoordinateInvC2List = new vcsVector[staticSolverData.faceCount];

	//iterate faceVertexIndexList
	for (int index = 0; index < staticSolverData.faceCount; index++)
	{
		//Get vertexIndices for face
		int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
		int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
		int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

		//Get UVIndices for face
		int UVIndex1 = int(staticSolverData.pFaceUVIndexList[index].x);
		int UVIndex2 = int(staticSolverData.pFaceUVIndexList[index].y);
		int UVIndex3 = int(staticSolverData.pFaceUVIndexList[index].z);

		//get Vectors vcsVectors at indices
		vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
		vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
		vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

		//get UVs vcsVectors at indices
		vcsVector uv1 = staticSolverData.pUVPositionList[UVIndex1];
		vcsVector uv2 = staticSolverData.pUVPositionList[UVIndex2];
		vcsVector uv3 = staticSolverData.pUVPositionList[UVIndex3];

		vcsVector vecDifference21 = vec2 - vec1;
		vcsVector vecDifference31 = vec3 - vec1;

		vcsVector uvDifference21 = uv2 - uv1;
		vcsVector uvDifference31 = uv3 - uv1;

		float uvDeterminant = (uvDifference21.x * uvDifference31.y) - (uvDifference31.x * uvDifference21.y);
		if (uvDeterminant == 0.0) uvDeterminant = 0.000001;

		vcsVector uvInverseVector1 = vcsVector(uvDifference31.y, (uvDifference21.y * (-1)), 0.0) * (1 / uvDeterminant);
		vcsVector uvInverseVector2 = vcsVector((uvDifference31.x * (-1)), uvDifference21.x, 0.0) * (1 / uvDeterminant);

		float tUx = (vecDifference21.x * uvInverseVector1.x) + (vecDifference31.x * uvInverseVector1.y);
		float tUy = (vecDifference21.y * uvInverseVector1.x) + (vecDifference31.y * uvInverseVector1.y);
		float tUz = (vecDifference21.z * uvInverseVector1.x) + (vecDifference31.z * uvInverseVector1.y);

		float tVx = (vecDifference21.x * uvInverseVector2.x) + (vecDifference31.x * uvInverseVector2.y);
		float tVy = (vecDifference21.y * uvInverseVector2.x) + (vecDifference31.y * uvInverseVector2.y);
		float tVz = (vecDifference21.z * uvInverseVector2.x) + (vecDifference31.z * uvInverseVector2.y);

		vcsVector tangentVectorU = vcsVector(tUx, tUy, tUz);
		vcsVector tangentVectorV = vcsVector(tVx, tVy, tVz);

		vcsVector tangentVectorUNormal = tangentVectorU.normal();
		vcsVector tangentVectorVNormal = tangentVectorV.normal();

		float c1x = (tangentVectorUNormal.x * vecDifference21.x) + (tangentVectorUNormal.y * vecDifference21.y) + (tangentVectorUNormal.z * vecDifference21.z);
		float c1y = (tangentVectorVNormal.x * vecDifference21.x) + (tangentVectorVNormal.y * vecDifference21.y) + (tangentVectorVNormal.z * vecDifference21.z);
		
		float c2x = (tangentVectorUNormal.x * vecDifference31.x) + (tangentVectorUNormal.y * vecDifference31.y) + (tangentVectorUNormal.z * vecDifference31.z);
		float c2y = (tangentVectorVNormal.x * vecDifference31.x) + (tangentVectorVNormal.y * vecDifference31.y) + (tangentVectorVNormal.z * vecDifference31.z);

		vcsVector materialCoordinateC1 = vcsVector(c1x, c1y, 0.0);
		vcsVector materialCoordinateC2 = vcsVector(c2x, c2y, 0.0);

		float determinantC = (c1x * c2y) - (c2x * c1y);
		if (determinantC == 0.0) determinantC = 0.000001;

		vcsVector inverseMaterialCoordinateC1 = vcsVector(c2y, -c1y, 0.0) * (1 / determinantC);
		vcsVector inverseMaterialCoordinateC2 = vcsVector(-c2x, c1x, 0.0) * (1 / determinantC);
		
		//set in pMaterialCoordinateC1List
		staticSolverData.pMaterialCoordinateC1List[index] = materialCoordinateC1;
		staticSolverData.pMaterialCoordinateC2List[index] = materialCoordinateC2;

		//set in pMaterialCoordinateInvC1List and pMaterialCoordinateInvC2List
		staticSolverData.pMaterialCoordinateInvC1List[index] = inverseMaterialCoordinateC1;
		staticSolverData.pMaterialCoordinateInvC2List[index] = inverseMaterialCoordinateC2;
	}
}


//dynamicSolverData
//-----------------------------------------------

//setDynamicSolverData
//-----------------------------------------------
void MPHYSSolver::setDynamicSolverData(void* proprietaryData)
{
	//cast MDataBlockPtr
	MDataBlock* pData = static_cast<MDataBlock*>(proprietaryData);

	dynamicSolverData.drag = pData->inputValue(aDrag).asFloat();
		
	setVertexForceList(*pData);
	
	dynamicSolverData.deltaTime = pData->inputValue(aDeltaTime).asFloat();
	
	dynamicSolverData.repetitions = pData->inputValue(aRepetitions).asInt();

	dynamicSolverData.alphaCompliance = pData->inputValue(aAlphaCompliance).asFloat();
	
	setPositionConstraintCount(*pData);
	setPositionConstraintActiveList(*pData);
	setPositionConstraintVertexIndexList(*pData);
	setPositionConstraintCoordinateList(*pData);
	
	dynamicSolverData.collisionGroundplaneActive = pData->inputValue(aCollisionConstraintGroundplaneActive).asInt();
	dynamicSolverData.groundplaneHeight = pData->inputValue(aCollisionConstraintGroundplaneHeight).asFloat();
	setCollisionConstraintSpheresCountAndVecUpVecDownList(*pData);	
	setCollisionConstraintConvexCountAndTriangleCountList(*pData);
	setCollisionConstraintConvexTriangleVertexPositionList(*pData);

	dynamicSolverData.pressureStiffness = pData->inputValue(aPressureStiffness).asFloat();
	
	dynamicSolverData.meshThickness = pData->inputValue(aMeshThickness).asFloat();
	
	dynamicSolverData.strainStiffness11 = pData->inputValue(aStrainStiffness11).asFloat();
	dynamicSolverData.strainStiffness22 = pData->inputValue(aStrainStiffness22).asFloat();
	dynamicSolverData.strainStiffness33 = pData->inputValue(aStrainStiffness33).asFloat();
	dynamicSolverData.strainStiffness12 = pData->inputValue(aStrainStiffness12).asFloat();
	dynamicSolverData.strainStiffness13 = pData->inputValue(aStrainStiffness13).asFloat();
	dynamicSolverData.strainStiffness23 = pData->inputValue(aStrainStiffness23).asFloat();

	dynamicSolverData.youngsModulus = pData->inputValue(aYoungsModulus).asFloat();
	dynamicSolverData.poissonRatio = pData->inputValue(aPoissonRatio).asFloat();

	dynamicSolverData.springStiffness = pData->inputValue(aSpringStiffness).asFloat();

	dynamicSolverData.materialConstant1 = pData->inputValue(aMaterialConstant1).asFloat();
	dynamicSolverData.materialConstant2 = pData->inputValue(aMaterialConstant2).asFloat();
	dynamicSolverData.materialConstant3 = pData->inputValue(aMaterialConstant3).asFloat();
	dynamicSolverData.materialConstant4 = pData->inputValue(aMaterialConstant4).asFloat();

	dynamicSolverData.youngsModuliKWeft = pData->inputValue(aYoungsModuliKWeft).asFloat();
	dynamicSolverData.youngsModuliKWarp = pData->inputValue(aYoungsModuliKWarp).asFloat();
	dynamicSolverData.youngsModuliKShear = pData->inputValue(aYoungsModuliKShear).asFloat();
	dynamicSolverData.poissonRatioVWeft = pData->inputValue(aPoissonRatioVWeft).asFloat();
	dynamicSolverData.poissonRatioVWarp = pData->inputValue(aPoissonRatioVWarp).asFloat();

	resetTotalLagrangeMultiplierList(*pData);

	set_thread_count_and_omp(*pData);
}


//resetTotalLagrangeMultiplierList
//-----------------------------------------------
void MPHYSSolver::resetTotalLagrangeMultiplierList(MDataBlock &data)
{
	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStretching1;
	dynamicSolverData.totalLagrangeMultiplierStretching1 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStretching2;
	dynamicSolverData.totalLagrangeMultiplierStretching2 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierHookeSpring1;
	dynamicSolverData.totalLagrangeMultiplierHookeSpring1 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierHookeSpring2;
	dynamicSolverData.totalLagrangeMultiplierHookeSpring2 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierSTVKSpring1;
	dynamicSolverData.totalLagrangeMultiplierSTVKSpring1 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierSTVKSpring2;
	dynamicSolverData.totalLagrangeMultiplierSTVKSpring2 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierMorsePot1;
	dynamicSolverData.totalLagrangeMultiplierMorsePot1 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierMorsePot2;
	dynamicSolverData.totalLagrangeMultiplierMorsePot2 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring1;
	dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring1 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring2;
	dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring2 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring1;
	dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring1 = new float[staticSolverData.edgeCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring2;
	dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring2 = new float[staticSolverData.edgeCount];

	//fill floatArray with zeros
	for (int index = 0; index < staticSolverData.edgeCount; index++)
	{
		dynamicSolverData.totalLagrangeMultiplierStretching1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStretching2[index] = 0.0;

		dynamicSolverData.totalLagrangeMultiplierHookeSpring1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierHookeSpring2[index] = 0.0;

		dynamicSolverData.totalLagrangeMultiplierSTVKSpring1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierSTVKSpring2[index] = 0.0;

		dynamicSolverData.totalLagrangeMultiplierMorsePot1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierMorsePot2[index] = 0.0;

		dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring2[index] = 0.0;

		dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring2[index] = 0.0;
	}

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierVolume1;
	dynamicSolverData.totalLagrangeMultiplierVolume1 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierVolume2;
	dynamicSolverData.totalLagrangeMultiplierVolume2 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierVolume3;
	dynamicSolverData.totalLagrangeMultiplierVolume3 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain1_11;
	dynamicSolverData.totalLagrangeMultiplierStrain1_11 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain2_11;
	dynamicSolverData.totalLagrangeMultiplierStrain2_11 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain3_11;
	dynamicSolverData.totalLagrangeMultiplierStrain3_11 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain1_22;
	dynamicSolverData.totalLagrangeMultiplierStrain1_22 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain2_22;
	dynamicSolverData.totalLagrangeMultiplierStrain2_22 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain3_22;
	dynamicSolverData.totalLagrangeMultiplierStrain3_22 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain1_33;
	dynamicSolverData.totalLagrangeMultiplierStrain1_33 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain2_33;
	dynamicSolverData.totalLagrangeMultiplierStrain2_33 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain3_33;
	dynamicSolverData.totalLagrangeMultiplierStrain3_33 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain1_12;
	dynamicSolverData.totalLagrangeMultiplierStrain1_12 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain2_12;
	dynamicSolverData.totalLagrangeMultiplierStrain2_12 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain3_12;
	dynamicSolverData.totalLagrangeMultiplierStrain3_12 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain1_13;
	dynamicSolverData.totalLagrangeMultiplierStrain1_13 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain2_13;
	dynamicSolverData.totalLagrangeMultiplierStrain2_13 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain3_13;
	dynamicSolverData.totalLagrangeMultiplierStrain3_13 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain1_23;
	dynamicSolverData.totalLagrangeMultiplierStrain1_23 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain2_23;
	dynamicSolverData.totalLagrangeMultiplierStrain2_23 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrain3_23;
	dynamicSolverData.totalLagrangeMultiplierStrain3_23 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrainArea1;
	dynamicSolverData.totalLagrangeMultiplierStrainArea1 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrainArea2;
	dynamicSolverData.totalLagrangeMultiplierStrainArea2 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrainArea3;
	dynamicSolverData.totalLagrangeMultiplierStrainArea3 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrainVolume1;
	dynamicSolverData.totalLagrangeMultiplierStrainVolume1 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrainVolume2;
	dynamicSolverData.totalLagrangeMultiplierStrainVolume2 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierStrainVolume3;
	dynamicSolverData.totalLagrangeMultiplierStrainVolume3 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierSTVK1;
	dynamicSolverData.totalLagrangeMultiplierSTVK1 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierSTVK2;
	dynamicSolverData.totalLagrangeMultiplierSTVK2 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierSTVK3;
	dynamicSolverData.totalLagrangeMultiplierSTVK3 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierNeoH1;
	dynamicSolverData.totalLagrangeMultiplierNeoH1 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierNeoH2;
	dynamicSolverData.totalLagrangeMultiplierNeoH2 = new float[staticSolverData.faceCount];

	//float array
	delete[] dynamicSolverData.totalLagrangeMultiplierNeoH3;
	dynamicSolverData.totalLagrangeMultiplierNeoH3 = new float[staticSolverData.faceCount];

	//fill floatArray with zeros
	for (int index = 0; index < staticSolverData.faceCount; index++)
	{
		dynamicSolverData.totalLagrangeMultiplierVolume1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierVolume2[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierVolume3[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain1_33[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain2_33[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain3_33[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain1_13[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain2_13[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain3_13[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain1_23[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain2_23[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrain3_23[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrainArea1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrainArea2[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrainArea3[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrainVolume1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrainVolume2[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierStrainVolume3[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierSTVK1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierSTVK2[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierSTVK3[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierNeoH1[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierNeoH2[index] = 0.0;
		dynamicSolverData.totalLagrangeMultiplierNeoH3[index] = 0.0;
	}
}


//setVertexForceList
//-----------------------------------------------
void MPHYSSolver::setVertexForceList(MDataBlock &data)
{
	//get gravity from datablock
	MFloatVector gravity = data.inputValue(aGravity).asFloatVector();

	//get WindDirection from datablock
	MFloatVector windDirection = data.inputValue(aWindDirection).asFloatVector();

	//vcsVector array
	delete[] dynamicSolverData.pVertexForceListGravity;
	dynamicSolverData.pVertexForceListGravity = new vcsVector[staticSolverData.vertexCount];

	//vcsVector array
	delete[] dynamicSolverData.pVertexForceListWind;
	dynamicSolverData.pVertexForceListWind = new vcsVector[staticSolverData.vertexCount];

	//fill vcsVectorArray with points in ws
	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		float x = gravity.x;
		float y = gravity.y;
		float z = gravity.z;
		
		//set gravity
		dynamicSolverData.pVertexForceListGravity[index] = vcsVector(x, y, z);

		//initialize wind
		dynamicSolverData.pVertexForceListWind[index] = vcsVector(0.0, 0.0, 0.0);
	}

	//fill vcsVectorArray with points in ws
	for (int index = 0; index < staticSolverData.faceCount; index++)
	{
		//vertexIndices
		int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
		int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
		int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

		//vcsVector for indices
		vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
		vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
		vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

		vcsVector vecDifference21 = vec2 - vec1;
		vcsVector vecDifference31 = vec3 - vec1;

		vcsVector triangleCross = vecDifference21.cross(vecDifference31);
		float triangleCrossLength = triangleCross.length();
		if (triangleCrossLength == 0.0) triangleCrossLength = 0.000001;
		vcsVector triangleNormal = triangleCross / triangleCrossLength;
		
		float x = windDirection.x;
		float y = windDirection.y;
		float z = windDirection.z;

		vcsVector windDir = vcsVector(x, y, z);

		vcsVector windForce = triangleNormal * (triangleNormal.dot(windDir));

		dynamicSolverData.pVertexForceListWind[vertexIndex1] = dynamicSolverData.pVertexForceListWind[vertexIndex1] + windForce;
		dynamicSolverData.pVertexForceListWind[vertexIndex2] = dynamicSolverData.pVertexForceListWind[vertexIndex2] + windForce;
		dynamicSolverData.pVertexForceListWind[vertexIndex3] = dynamicSolverData.pVertexForceListWind[vertexIndex3] + windForce;
	}
}


//setPositionConstraintCount
//-----------------------------------------------
void MPHYSSolver::setPositionConstraintCount(MDataBlock &data)
{
	//get handle to positionConstraint Array attr
	MArrayDataHandle hPositionConstraintArray = data.inputArrayValue(aPositionConstraint);
	//setData
	dynamicSolverData.positionConstraintCount = hPositionConstraintArray.elementCount();
}


//setPositionConstraintActiveList
//-----------------------------------------------
void MPHYSSolver::setPositionConstraintActiveList(MDataBlock &data)
{
	if (!dynamicSolverData.positionConstraintCount)//No posCons
	{
		//if there was posCons before delete them
		if (dynamicSolverData.pPositionConstraintActiveList)
		{
			delete[] dynamicSolverData.pPositionConstraintActiveList;
			dynamicSolverData.pPositionConstraintActiveList = 0;
		}
	}
	else//posCons 
	{
		if (dynamicSolverData.pPositionConstraintActiveList)//if there was posCons before delete them
		{
			delete[] dynamicSolverData.pPositionConstraintActiveList;

			//For Loop to assign new posConsActive
			//get handle to positionConstraint Array attr
			MArrayDataHandle hPositionConstraintArray = data.inputArrayValue(aPositionConstraint);

			//allocate new memory for pPosConActiveList
			dynamicSolverData.pPositionConstraintActiveList = new int[hPositionConstraintArray.elementCount()];

			//iterate ArrayElements
			for (unsigned int index = 0; index < hPositionConstraintArray.elementCount(); index++)
			{
				hPositionConstraintArray.jumpToArrayElement(index);
				
				//get handle to active attr for current array element
				MDataHandle hPositionConstraintArrayElement = hPositionConstraintArray.inputValue();
				MDataHandle hPositionConstraintActive = hPositionConstraintArrayElement.child(aPositionConstraintActive);
				
				dynamicSolverData.pPositionConstraintActiveList[index] = int(hPositionConstraintActive.asShort());
			}

		}
		else//there were no posCons before 
		{
			//For Loop to assign new posConsActive
			//get handle to positionConstraint Array attr
			MArrayDataHandle hPositionConstraintArray = data.inputArrayValue(aPositionConstraint);

			//allocate new memory for pPosConActiveList
			dynamicSolverData.pPositionConstraintActiveList = new int[hPositionConstraintArray.elementCount()];

			//iterate ArrayElements
			for (unsigned int index = 0; index < hPositionConstraintArray.elementCount(); index++)
			{
				hPositionConstraintArray.jumpToArrayElement(index);
				
				//get handle to active attr for current array element
				MDataHandle hPositionConstraintArrayElement = hPositionConstraintArray.inputValue();
				MDataHandle hPositionConstraintActive = hPositionConstraintArrayElement.child(aPositionConstraintActive);
				
				dynamicSolverData.pPositionConstraintActiveList[index] = int(hPositionConstraintActive.asShort());
			}
		}
	}
}


//setPositionConstraintVertexIndexList
//-----------------------------------------------
void MPHYSSolver::setPositionConstraintVertexIndexList(MDataBlock &data)
{
	if (!dynamicSolverData.positionConstraintCount)//No posCons
	{
		//if there was posCons before delete them
		if (dynamicSolverData.pPositionConstraintVertexIndexList)
		{
			delete[] dynamicSolverData.pPositionConstraintVertexIndexList;
			dynamicSolverData.pPositionConstraintVertexIndexList = 0;
		}
	}
	else//posCons 
	{
		if (dynamicSolverData.pPositionConstraintVertexIndexList)//if there was posCons before delete them
		{
			delete[] dynamicSolverData.pPositionConstraintVertexIndexList;

			//For Loop to assign new posConsActive
			//get handle to positionConstraint Array attr
			MArrayDataHandle hPositionConstraintArray = data.inputArrayValue(aPositionConstraint);

			//allocate new memory for pPosConVertexIndexList
			dynamicSolverData.pPositionConstraintVertexIndexList = new int[hPositionConstraintArray.elementCount()];

			//iterate ArrayElements
			for (unsigned int index = 0; index < hPositionConstraintArray.elementCount(); index++)
			{
				hPositionConstraintArray.jumpToArrayElement(index);
				
				//get handle to active attr for current array element
				MDataHandle hPositionConstraintArrayElement = hPositionConstraintArray.inputValue();
				MDataHandle hPositionConstraintVertexIndex = hPositionConstraintArrayElement.child(aPositionConstraintVertexIndex);
				
				dynamicSolverData.pPositionConstraintVertexIndexList[index] = hPositionConstraintVertexIndex.asInt();
			}

		}
		else//there were no posCons before 
		{
			//For Loop to assign new posConsActive
			//get handle to positionConstraint Array attr
			MArrayDataHandle hPositionConstraintArray = data.inputArrayValue(aPositionConstraint);

			//allocate new memory for pPosConVertexIndexList
			dynamicSolverData.pPositionConstraintVertexIndexList = new int[hPositionConstraintArray.elementCount()];

			//iterate ArrayElements
			for (unsigned int index = 0; index < hPositionConstraintArray.elementCount(); index++)
			{
				hPositionConstraintArray.jumpToArrayElement(index);
				
				//get handle to active attr for current array element
				MDataHandle hPositionConstraintArrayElement = hPositionConstraintArray.inputValue();
				MDataHandle hPositionConstraintVertexIndex = hPositionConstraintArrayElement.child(aPositionConstraintVertexIndex);
				
				dynamicSolverData.pPositionConstraintVertexIndexList[index] = hPositionConstraintVertexIndex.asInt();
			}
		}
	}
}


//setPositionConstraintCoordinateList
//-----------------------------------------------
void MPHYSSolver::setPositionConstraintCoordinateList(MDataBlock &data)
{
	if (!dynamicSolverData.positionConstraintCount)//No posCons
	{
		//if there was posCons before delete them
		if (dynamicSolverData.pPositionConstraintCoordinateList)
		{
			delete[] dynamicSolverData.pPositionConstraintCoordinateList;
			dynamicSolverData.pPositionConstraintCoordinateList = 0;
		}
	}
	else//posCons 
	{
		if (dynamicSolverData.pPositionConstraintCoordinateList)//if there was posCons before delete them
		{
			delete[] dynamicSolverData.pPositionConstraintCoordinateList;

			//For Loop to assign new posConsActive
			//get handle to positionConstraint Array attr
			MArrayDataHandle hPositionConstraintArray = data.inputArrayValue(aPositionConstraint);

			//allocate new memory for pPosConVertexIndexList
			dynamicSolverData.pPositionConstraintCoordinateList = new vcsVector[hPositionConstraintArray.elementCount()];

			//iterate ArrayElements
			for (unsigned int index = 0; index < hPositionConstraintArray.elementCount(); index++)
			{
				hPositionConstraintArray.jumpToArrayElement(index);
				
				//get handle to active attr for current array element
				MDataHandle hPositionConstraintArrayElement = hPositionConstraintArray.inputValue();
				MDataHandle hPositionConstraintCoordinate = hPositionConstraintArrayElement.child(aPositionConstraintCoordinate);
				
				//Create new vcsVector
				float x = hPositionConstraintCoordinate.asFloatVector().x;
				float y = hPositionConstraintCoordinate.asFloatVector().y;
				float z = hPositionConstraintCoordinate.asFloatVector().z;
				
				dynamicSolverData.pPositionConstraintCoordinateList[index] = vcsVector(x, y, z);
			}

		}
		else//there were no posCons before 
		{
			//For Loop to assign new posConsActive
			//get handle to positionConstraint Array attr
			MArrayDataHandle hPositionConstraintArray = data.inputArrayValue(aPositionConstraint);

			//allocate new memory for pPosConVertexIndexList
			dynamicSolverData.pPositionConstraintCoordinateList = new vcsVector[hPositionConstraintArray.elementCount()];

			//iterate ArrayElements
			for (unsigned int index = 0; index < hPositionConstraintArray.elementCount(); index++)
			{
				hPositionConstraintArray.jumpToArrayElement(index);
				
				//get handle to active attr for current array element
				MDataHandle hPositionConstraintArrayElement = hPositionConstraintArray.inputValue();
				MDataHandle hPositionConstraintCoordinate = hPositionConstraintArrayElement.child(aPositionConstraintCoordinate);
				
				//Create new vcsVector
				float x = hPositionConstraintCoordinate.asFloatVector().x;
				float y = hPositionConstraintCoordinate.asFloatVector().y;
				float z = hPositionConstraintCoordinate.asFloatVector().z;
				
				dynamicSolverData.pPositionConstraintCoordinateList[index] = vcsVector(x, y, z);
			}
		}
	}
}


//setCollisionConstraintSpheresCountAndVecUpVecDownList
//-----------------------------------------------
void MPHYSSolver::setCollisionConstraintSpheresCountAndVecUpVecDownList(MDataBlock &data)
{
	//get handle to collision constraint array attr
	MArrayDataHandle hCollisionConstraintCompoundArray = data.inputArrayValue(aCollisionConstraint);
	//elementCount
	int elementCount = hCollisionConstraintCompoundArray.elementCount();


	//Iterate to get spheresConstraintCount
	//-----------------------------------------------

	//collisionConstraintSpheresCount
	int collisionConstraintSpheresCount = 0;

	for (int index = 0; index < elementCount; index++)
	{
		hCollisionConstraintCompoundArray.jumpToArrayElement(index);

		//get datahandles to Attrs for current compoundAttr
		MDataHandle hCollisionConstraintCompound = hCollisionConstraintCompoundArray.inputValue();
		
		short collisionConstraintActive = hCollisionConstraintCompound.child(aCollisionConstraintActive).asShort();
		short collisionConstraintType = hCollisionConstraintCompound.child(aCollisionConstraintType).asShort();

		//check collisionConstraintType
		if (collisionConstraintType == 0)
		{
			//check active
			if (collisionConstraintActive)
			{
				//oThisNode
				MObject oThisNode = thisMObject();

				//Get pCollisionConstraintCompoundMatrix
				MPlug pCollisionConstraintCompoundArray = MPlug(oThisNode, aCollisionConstraint);
				MPlug pCollisionConstraintCompound = pCollisionConstraintCompoundArray.elementByPhysicalIndex(index);
				MPlug pCollisionConstraintCompoundMatrix = pCollisionConstraintCompound.child(2);

				//check matrix connected
				if (pCollisionConstraintCompoundMatrix.isConnected())
				{
					//get pCollisionConstraintCompoundGeo
					MPlug pCollisionConstraintCompoundGeo = pCollisionConstraintCompound.child(3);

					//check geo connected
					if (pCollisionConstraintCompoundGeo.isConnected())
					{
						collisionConstraintSpheresCount++;
					}
				}
			}
		}
	}

	//set dynamicSolverData
	dynamicSolverData.collisionConstraintSpheresCount = collisionConstraintSpheresCount;
	
	//Iterate to get vecUpvecDownList items
	//-----------------------------------------------

	//delete old ptr
	delete[] dynamicSolverData.pCollisionConstraintSpheresVecUpVecDownList;
	
	//create new ptr
	dynamicSolverData.pCollisionConstraintSpheresVecUpVecDownList = new vcsVector[collisionConstraintSpheresCount * 2];

	//collisionConstraintSpheresIndex
	int collisionConstraintSpheresIndex = 0;

	for (int index = 0; index < elementCount; index++)
	{
		hCollisionConstraintCompoundArray.jumpToArrayElement(index);

		//get datahandles to Attrs for current compoundAttr
		MDataHandle hCollisionConstraintCompound = hCollisionConstraintCompoundArray.inputValue();
		
		short collisionConstraintActive = hCollisionConstraintCompound.child(aCollisionConstraintActive).asShort();
		short collisionConstraintType = hCollisionConstraintCompound.child(aCollisionConstraintType).asShort();

		//check collisionConstraintType
		if (collisionConstraintType == 0)
		{
			//check active
			if (collisionConstraintActive)
			{
				//oThisNode
				MObject oThisNode = thisMObject();

				//Get pCollisionConstraintCompoundMatrix
				MPlug pCollisionConstraintCompoundArray = MPlug(oThisNode, aCollisionConstraint);
				MPlug pCollisionConstraintCompound = pCollisionConstraintCompoundArray.elementByPhysicalIndex(index);
				MPlug pCollisionConstraintCompoundMatrix = pCollisionConstraintCompound.child(2);

				//check matrix connected
				if (pCollisionConstraintCompoundMatrix.isConnected())
				{
					//get pCollisionConstraintCompoundGeo
					MPlug pCollisionConstraintCompoundGeo = pCollisionConstraintCompound.child(3);

					//check geo connected
					if (pCollisionConstraintCompoundGeo.isConnected())
					{
						//Get collisionGeo
						MObject collisionGeo = hCollisionConstraintCompound.child(aCollisionConstraintGeo).asMesh();
						
						//get collisionGeoMatrix
						MMatrix collisionGeoMatrix = hCollisionConstraintCompound.child(aCollisionConstraintGeoMatrix).asMatrix();

						//fnCollisionGeo
						MFnMesh fnCollisionGeo(collisionGeo);
						
						//collisionGeoPointList
						MPointArray collisionGeoPointList;
						fnCollisionGeo.getPoints(collisionGeoPointList);

						//Get vecUp and vecDown
						int collisionGeoPointListLength = collisionGeoPointList.length();
						
						MPoint mpVecUp = collisionGeoPointList[collisionGeoPointListLength - 1] * collisionGeoMatrix;
						MPoint mpVecDown = collisionGeoPointList[collisionGeoPointListLength - 2] * collisionGeoMatrix;

						//cast to vcsVector
						vcsVector vecUp = vcsVector(mpVecUp.x, mpVecUp.y, mpVecUp.z);
						vcsVector vecDown = vcsVector(mpVecDown.x, mpVecDown.y, mpVecDown.z);

						//set dynamicSolverData
						dynamicSolverData.pCollisionConstraintSpheresVecUpVecDownList[collisionConstraintSpheresIndex] = vecUp;
						dynamicSolverData.pCollisionConstraintSpheresVecUpVecDownList[collisionConstraintSpheresIndex + 1] = vecDown;

						//collisionConstraintSpheresIndex
						collisionConstraintSpheresIndex += 2;
					}
				}
			}
		}
	}
}


//setCollisionConstraintConvexCountAndTriangleCountList
//-----------------------------------------------
void MPHYSSolver::setCollisionConstraintConvexCountAndTriangleCountList(MDataBlock &data)
{
	//get handle to collision constraint array attr
	MArrayDataHandle hCollisionConstraintCompoundArray = data.inputArrayValue(aCollisionConstraint);
	//elementCount
	int elementCount = hCollisionConstraintCompoundArray.elementCount();
	//oThisNode
	MObject oThisNode = thisMObject();

	//Iterate to get convexConstraintCount
	//-----------------------------------------------

	//collisionConstraintConvexArrayIndexList
	std::vector<int> collisionConstraintConvexArrayIndexList;

	for (int index = 0; index < elementCount; index++)
	{
		//Jump to array element
		hCollisionConstraintCompoundArray.jumpToArrayElement(index);

		//get datahandles to Attrs for current compoundAttr
		MDataHandle hCollisionConstraintCompound = hCollisionConstraintCompoundArray.inputValue();
		short collisionConstraintActive = hCollisionConstraintCompound.child(aCollisionConstraintActive).asShort();
		short collisionConstraintType = hCollisionConstraintCompound.child(aCollisionConstraintType).asShort();

		//check collisionConstraintType
		if (collisionConstraintType == 1)
		{
			//check active
			if (collisionConstraintActive)
			{
				//Get pCollisionConstraintCompoundMatrix
				MPlug pCollisionConstraintCompoundArray = MPlug(oThisNode, aCollisionConstraint);
				MPlug pCollisionConstraintCompound = pCollisionConstraintCompoundArray.elementByPhysicalIndex(index);
				MPlug pCollisionConstraintCompoundMatrix = pCollisionConstraintCompound.child(2);

				//check matrix connected
				if (pCollisionConstraintCompoundMatrix.isConnected())
				{
					//get pCollisionConstraintCompoundGeo
					MPlug pCollisionConstraintCompoundGeo = pCollisionConstraintCompound.child(3);

					//check geo connected
					if (pCollisionConstraintCompoundGeo.isConnected())
					{
						collisionConstraintConvexArrayIndexList.push_back(index);
					}
				}
			}
		}
	}

	//collisionConstraintConvexCount
	int collisionConstraintConvexCount = collisionConstraintConvexArrayIndexList.size();

	//set dynamicSolverData convex constraint count
	dynamicSolverData.collisionConstraintConvexCount = collisionConstraintConvexCount;

	//deallocate memory where current ptr points to
	delete[] dynamicSolverData.pCollisionConstraintConvexTriangleCountList;
	//reallocate
	dynamicSolverData.pCollisionConstraintConvexTriangleCountList = new int[collisionConstraintConvexCount];

	//Fill triangleCountList
	for (int index = 0; index < collisionConstraintConvexCount; index++)
	{
		//arrayIndex
		int collisionConstraintConvexArrayIndex = collisionConstraintConvexArrayIndexList[index];

		//Get pCollisionConstraintCompoundGeo
		MPlug pCollisionConstraintCompoundArray = MPlug(oThisNode, aCollisionConstraint);
		MPlug pCollisionConstraintCompound = pCollisionConstraintCompoundArray.elementByPhysicalIndex(collisionConstraintConvexArrayIndex);
		MPlug pCollisionConstraintCompoundGeo = pCollisionConstraintCompound.child(3);

		//triangleCountPerPolygonList & triangleVertexIndexList
		MIntArray triangleCountPerPolygonList;
		MIntArray triangleVertexIndexList;

		//fnMeshCollisionGeo
		MObject oCollisionGeo = pCollisionConstraintCompoundGeo.asMObject();
		MFnMesh fnMeshCollisionGeo(oCollisionGeo);

		//get triangles
		fnMeshCollisionGeo.getTriangles(triangleCountPerPolygonList, triangleVertexIndexList);

		//collisionGeoTriangleCount
		int collisionGeoTriangleCount = 0;
		for (int i = 0; i < triangleCountPerPolygonList.length(); i++)
		{
			int triangleCountPerPolygon = triangleCountPerPolygonList[i];
			collisionGeoTriangleCount += triangleCountPerPolygon;
		}

		//set to dynamicSolverData
		dynamicSolverData.pCollisionConstraintConvexTriangleCountList[index] = collisionGeoTriangleCount;
	}
}


//setCollisionConstraintConvexTriangleVertexPositionList
//-----------------------------------------------
void MPHYSSolver::setCollisionConstraintConvexTriangleVertexPositionList(MDataBlock &data)
{

	//get handle to collision constraint array attr
	MArrayDataHandle hCollisionConstraintCompoundArray = data.inputArrayValue(aCollisionConstraint);
	//elementCount
	int elementCount = hCollisionConstraintCompoundArray.elementCount();
	//oThisNode
	MObject oThisNode = thisMObject();

	//Iterate to get convexConstraintCount
	//-----------------------------------------------

	//collisionConstraintConvexArrayIndexList
	std::vector<int> collisionConstraintConvexArrayIndexList;

	for (int index = 0; index < elementCount; index++)
	{
		//Jump to array element
		hCollisionConstraintCompoundArray.jumpToArrayElement(index);

		//get datahandles to Attrs for current compoundAttr
		MDataHandle hCollisionConstraintCompound = hCollisionConstraintCompoundArray.inputValue();
		short collisionConstraintActive = hCollisionConstraintCompound.child(aCollisionConstraintActive).asShort();
		short collisionConstraintType = hCollisionConstraintCompound.child(aCollisionConstraintType).asShort();

		//check collisionConstraintType
		if (collisionConstraintType == 1)
		{
			//check active
			if (collisionConstraintActive)
			{
				//Get pCollisionConstraintCompoundMatrix
				MPlug pCollisionConstraintCompoundArray = MPlug(oThisNode, aCollisionConstraint);
				MPlug pCollisionConstraintCompound = pCollisionConstraintCompoundArray.elementByPhysicalIndex(index);
				MPlug pCollisionConstraintCompoundMatrix = pCollisionConstraintCompound.child(2);

				//check matrix connected
				if (pCollisionConstraintCompoundMatrix.isConnected())
				{
					//get pCollisionConstraintCompoundGeo
					MPlug pCollisionConstraintCompoundGeo = pCollisionConstraintCompound.child(3);

					//check geo connected
					if (pCollisionConstraintCompoundGeo.isConnected())
					{
						collisionConstraintConvexArrayIndexList.push_back(index);
					}
				}
			}
		}
	}

	//collisionConstraintConvexCount
	int collisionConstraintConvexCount = collisionConstraintConvexArrayIndexList.size();

	//triangleVertexPositionList
	std::vector<vcsVector> triangleVertexPositionList;

	//Fill triangleCountList
	for (int index = 0; index < collisionConstraintConvexCount; index++)
	{
		//arrayIndex
		int collisionConstraintConvexArrayIndex = collisionConstraintConvexArrayIndexList[index];

		//Get pCollisionConstraintCompoundMatrix & pCollisionConstraintCompoundGeo
		MPlug pCollisionConstraintCompoundArray = MPlug(oThisNode, aCollisionConstraint);
		MPlug pCollisionConstraintCompound = pCollisionConstraintCompoundArray.elementByPhysicalIndex(collisionConstraintConvexArrayIndex);
		MPlug pCollisionConstraintCompoundMatrix = pCollisionConstraintCompound.child(2);
		MPlug pCollisionConstraintCompoundGeo = pCollisionConstraintCompound.child(3);

		//transformMatrix
		MObject oTransformMatrix = pCollisionConstraintCompoundMatrix.asMObject();
		//fnMatrixDataTransformMatrix
		MFnMatrixData fnMatrixDataTransformMatrix(oTransformMatrix);
		MMatrix transformMatrix = fnMatrixDataTransformMatrix.matrix();

		//triangleCountPerPolygonList & triangleVertexIndexList
		MIntArray triangleCountPerPolygonList;
		MIntArray triangleVertexIndexList;

		//fnMeshCollisionGeo
		MObject oCollisionGeo = pCollisionConstraintCompoundGeo.asMObject();
		MFnMesh fnMeshCollisionGeo(oCollisionGeo);

		//collisionGeoPointArray
		MPointArray collisionGeoPointArray;
		fnMeshCollisionGeo.getPoints(collisionGeoPointArray);

		//get triangles
		fnMeshCollisionGeo.getTriangles(triangleCountPerPolygonList, triangleVertexIndexList);

		//get triangle vertex positions
		for (int triangleIndex = 0; triangleIndex < triangleVertexIndexList.length(); triangleIndex += 3)
		{
			//triangleVertexIndices
			int triangleVertexIndex1 = triangleVertexIndexList[triangleIndex];
			int triangleVertexIndex2 = triangleVertexIndexList[triangleIndex + 1];
			int triangleVertexIndex3 = triangleVertexIndexList[triangleIndex + 2];

			//get vertexPositions
			MPoint triangleVertexPosition1 = collisionGeoPointArray[triangleVertexIndex1] * transformMatrix;
			MPoint triangleVertexPosition2 = collisionGeoPointArray[triangleVertexIndex2] * transformMatrix;
			MPoint triangleVertexPosition3 = collisionGeoPointArray[triangleVertexIndex3] * transformMatrix;

			//convert to vcsVector
			vcsVector vecTriangleVertexPosition1 = vcsVector(triangleVertexPosition1.x, triangleVertexPosition1.y, triangleVertexPosition1.z);
			vcsVector vecTriangleVertexPosition2 = vcsVector(triangleVertexPosition2.x, triangleVertexPosition2.y, triangleVertexPosition2.z);
			vcsVector vecTriangleVertexPosition3 = vcsVector(triangleVertexPosition3.x, triangleVertexPosition3.y, triangleVertexPosition3.z);

			//append to vector
			triangleVertexPositionList.push_back(vecTriangleVertexPosition1);
			triangleVertexPositionList.push_back(vecTriangleVertexPosition2);
			triangleVertexPositionList.push_back(vecTriangleVertexPosition3);
		}
	}

	//free memory
	delete[] dynamicSolverData.pCollisionConstraintConvexTriangleVertexPositionList;
	//reallocate
	dynamicSolverData.pCollisionConstraintConvexTriangleVertexPositionList = new vcsVector[triangleVertexPositionList.size()];

	//assign values to dynamicSolverData
	for (int index = 0; index < triangleVertexPositionList.size(); index++)
	{
		dynamicSolverData.pCollisionConstraintConvexTriangleVertexPositionList[index] = triangleVertexPositionList[index];
	}

	//Set collisionConstraintConvexTriangleVertexCount
	dynamicSolverData.collisionConstraintConvexTriangleVertexCount = triangleVertexPositionList.size();
}


//set_thread_count
//-----------------------------------------------
void MPHYSSolver::set_thread_count_and_omp(MDataBlock &data)
{
	//get thread_count from datablock
	int thread_count = data.inputValue(a_thread_count).asInt();

	//get omp from datablock
	int omp = static_cast<int> (data.inputValue(a_omp).asShort());

	//set in dynamicSolverData
	dynamicSolverData.thread_count = thread_count;

	//set in dynamicSolverData
	dynamicSolverData.omp = omp;
}


//drawData
//-----------------------------------------------

//setDrawData
//-----------------------------------------------
void MPHYSSolver::setDrawData(MDataBlock &data)
{
	//Set Data
	drawData.collisionConstraintGroundPlaneVisible = data.inputValue(aCollisionConstraintGroundplaneVisible).asBool();
	drawData.collisionConstraintGroundplaneDispSize = data.inputValue(aCollisionConstraintGroundplaneDispSize).asFloat();
	drawData.collisionConstraintGroundplaneHeight = data.inputValue(aCollisionConstraintGroundplaneHeight).asFloat();

	drawData.drawMeshlinesActive = data.inputValue(aDrawMeshlinesActive).asBool();
}


//MPHYSSolver Methods
//-----------------------------------------------

//attributeConnected
//-----------------------------------------------
bool MPHYSSolver::attributeConnected(MString attributeName)
{
	//fs for this node
	MFnDependencyNode fsDepNode(thisMObject());
	
	//Get plug to attr
	MPlug pAttr = fsDepNode.findPlug(attributeName, true);

	return pAttr.isConnected();
}


//setOutputGeo
//-----------------------------------------------
void MPHYSSolver::setOutputGeo(MDataBlock &data)
{
	//Get vertexPositionList
	MPointArray vertexPositionList(staticSolverData.vertexCount);

	for (int index = 0; index < staticSolverData.vertexCount; index++)
	{
		float x = staticSolverData.pVertexPositionList[index].x;
		float y = staticSolverData.pVertexPositionList[index].y;
		float z = staticSolverData.pVertexPositionList[index].z;
		
		vertexPositionList.set(MPoint(x, y, z), index);
	}

	//Get fsInputGeo
	MObject oInputGeo = data.inputValue(aInputGeo).asMesh();
	MFnMesh fsInputGeo(oInputGeo);

	//Create MeshDataBlock
	MFnMeshData fsMeshData;
	MObject oMeshDataBlock = fsMeshData.create();

	fsInputGeo.copy(oInputGeo, oMeshDataBlock);

	MFnMesh mfnNewMesh(oMeshDataBlock);
	
	mfnNewMesh.setPoints(vertexPositionList);
	mfnNewMesh.updateSurface();
	
	//set outputGeo in datablock
	MDataHandle hOutputGeo = data.outputValue(aOutputGeo);
	hOutputGeo.setMObject(oMeshDataBlock);
}


//MPHYSDrawer Methods
//-----------------------------------------------

MPHYSDrawerData::MPHYSDrawerData() : MUserData(false), collisionGroundPlaneColor(1.0f, 0.0f, 0.0f, 0.5f), meshEdgeColor(0.0f, 0.0f, 1.0f, 0.5f)
{
}

MPHYSDrawer::MPHYSDrawer(const MObject& obj) : MPxDrawOverride(obj, NULL)
{
}

MPHYSDrawer::~MPHYSDrawer()
{
}

DrawAPI MPHYSDrawer::supportedDrawAPIs() const
{
	// this plugin supports both GL and DX
	return (kOpenGL | kDirectX11 | kOpenGLCoreProfile);
}

MUserData* MPHYSDrawer::prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MFrameContext& frameContext, MUserData* oldData)
{
	MPHYSDrawerData* data = dynamic_cast<MPHYSDrawerData*>(oldData);

	if (!data) {
		data = new MPHYSDrawerData();
	}

	return data;
}

void MPHYSDrawer::addUIDrawables(const MDagPath& objPath, MUIDrawManager& drawManager, const MFrameContext& frameContext, const MUserData* data)
{
	MStatus status;
	MObject physNode = objPath.node(&status);
	MFnDependencyNode dnNode(physNode, &status);
	MPHYSSolver* dataFromPHYSSolverClass = dynamic_cast<MPHYSSolver*>(dnNode.userNode());

	//get drawData
	int displayCollisionGroundplane = dataFromPHYSSolverClass->drawData.collisionConstraintGroundPlaneVisible;
	float collisionGroundplaneDisplaySize = dataFromPHYSSolverClass->drawData.collisionConstraintGroundplaneDispSize;
	float collisionConstraintGroundplaneHeight = dataFromPHYSSolverClass->drawData.collisionConstraintGroundplaneHeight;

	if (displayCollisionGroundplane)
	{
		const MPHYSDrawerData* thisdata = dynamic_cast<const MPHYSDrawerData*>(data);

		// Begin the drawing
		drawManager.beginDrawable();
		{
			MPoint position(0.0, collisionConstraintGroundplaneHeight, 0.0);
			MVector normal(0.0, 1.0, 0.0);
			MVector rectUp(0.0, 0.0, 1.0);
			//Draw Quad
			//Color
			drawManager.setColor(thisdata->collisionGroundPlaneColor);
			drawManager.setLineWidth(3.0f);
			drawManager.setLineStyle(MUIDrawManager::kSolid);
			drawManager.rect(position, rectUp, normal, collisionGroundplaneDisplaySize, collisionGroundplaneDisplaySize, true);
		}
		drawManager.endDrawable();
	}

	//Get drawData
	int drawMeshlines = dataFromPHYSSolverClass->drawData.drawMeshlinesActive;

	if (drawMeshlines)
	{
		const MPHYSDrawerData* thisdata = dynamic_cast<const MPHYSDrawerData*>(data);

		drawManager.beginDrawable();
		{
			drawManager.setColor(thisdata->meshEdgeColor);
			drawManager.setLineWidth(3.0f);
			drawManager.setLineStyle(MUIDrawManager::kSolid);

			//Iterate edgeVertexIndexList
			for (int index = 0; index < (dataFromPHYSSolverClass->staticSolverData.edgeCount); index++)
			{
				//Indices
				int vertexIndex1 = int(dataFromPHYSSolverClass->staticSolverData.pEdgeVertexIndexList[index].x);
				int vertexIndex2 = int(dataFromPHYSSolverClass->staticSolverData.pEdgeVertexIndexList[index].y);

				//Position
				//get vcsVector for edgeIndex
				vcsVector vec1 = dataFromPHYSSolverClass->staticSolverData.pVertexPositionList[vertexIndex1];
				vcsVector vec2 = dataFromPHYSSolverClass->staticSolverData.pVertexPositionList[vertexIndex2];

				MPoint vec1forDraw = MPoint(vec1.x, vec1.y, vec1.z);
				MPoint vec2forDraw = MPoint(vec2.x, vec2.y, vec2.z);

				drawManager.line(vec1forDraw, vec2forDraw);
			}
		}
		drawManager.endDrawable();
	}
}