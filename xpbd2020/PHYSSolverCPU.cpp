

//PHYSSolverCPU Implementation
//-----------------------------------------------


//include
//-----------------------------------------------
#include "PHYSSolverCPU.h"


using namespace std;

#define NUMBER_OF_THREADS 8


//PHYSSolverCPU
//-----------------------------------------------

//Attributes


//Methods
PHYSSolverCPU::PHYSSolverCPU(){};
PHYSSolverCPU::~PHYSSolverCPU(){};


//solve
void PHYSSolverCPU::solve(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	//int start_s = clock();

	verletIntegration(staticSolverData, dynamicSolverData);

	//staticSolverData.potEneg = 0.0;

	for(int index = 1; index <= dynamicSolverData.repetitions; index++)
	{
		satisfyConstraints(staticSolverData, dynamicSolverData);
	}

	//cout << staticSolverData.relErrorValue << endl;
	//cout << staticSolverData.potEneg / dynamicSolverData.repetitions << endl;
	//cout << staticSolverData.potEneg << endl;

	velocityCorrection(staticSolverData, dynamicSolverData);

	//int stop_s = clock();

	//MGlobal::displayInfo(MString("Time: ") + (stop_s - start_s) / double(CLOCKS_PER_SEC));
	//cout << "time: " << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << endl;
}


//verletIntegration
void PHYSSolverCPU::verletIntegration(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		for (int index = 0; index < staticSolverData.vertexCount; index++)
		{
			//Get vars for integration
			vcsVector vecPos = staticSolverData.pVertexPositionList[index];
			vcsVector vecOldpos = staticSolverData.pVertexOldpositionList[index];
			vcsVector currentVelocity = staticSolverData.pVelocityList[index];
			vcsVector vecForceGravity = dynamicSolverData.pVertexForceListGravity[index];
			vcsVector vecForceWind = dynamicSolverData.pVertexForceListWind[index];
			vcsVector externalForces = vecForceGravity + vecForceWind;
			
			//Explicit Position Euler integration
			//vcsVector vecNewPos = vecPos + (vecPos - vecOldpos) + (externalForces * dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			vcsVector vecNewPos = vecPos + (currentVelocity * dynamicSolverData.deltaTime) + (externalForces * dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			
			//update solverData
			staticSolverData.pVertexOldpositionList[index] = vecPos;
			staticSolverData.pVertexPositionList[index] = vecNewPos;
		}
	}
}


//velocityCorrection
void PHYSSolverCPU::velocityCorrection(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		for (int index = 0; index < staticSolverData.vertexCount; index++)
		{
			//Get vars for integration
			vcsVector vecPos = staticSolverData.pVertexPositionList[index];
			vcsVector vecOldpos = staticSolverData.pVertexOldpositionList[index];

			//last velocity update within integration
			vcsVector velocityCorrection = (vecPos - vecOldpos) / dynamicSolverData.deltaTime;
			//vcsVector velocityCorrection = ((vecPos - vecOldpos) / dynamicSolverData.deltaTime) * (1 - (dynamicSolverData.drag * 0.1));

			//update solverData
			staticSolverData.pVelocityList[index] = velocityCorrection;
		}
	}
}


//satisfyConstraints
void PHYSSolverCPU::satisfyConstraints(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	//constraint hierarchy
	//stretchingConstraint(staticSolverData, dynamicSolverData);
	//centerofMassComputation(staticSolverData, dynamicSolverData);

	//greenStrainConstraintTri(staticSolverData, dynamicSolverData);
	//greenStrainConstraintModifiedTri(staticSolverData, dynamicSolverData);
	//greenStrainConstraintTet(staticSolverData, dynamicSolverData);
	//greenStrainConstraintModifiedTet(staticSolverData, dynamicSolverData);
	//areaStrainConstraintTri(staticSolverData, dynamicSolverData);
	//volumeStrainConstraintTet(staticSolverData, dynamicSolverData);

	//stVenantKirchhoffConstraintTet(staticSolverData, dynamicSolverData);
	//neoHookeanConstraintTet(staticSolverData, dynamicSolverData);

	//hookeSpringConstraint(staticSolverData, dynamicSolverData);
	//stvkSpringConstraint(staticSolverData, dynamicSolverData);
	//morsePotentialConstraint(staticSolverData, dynamicSolverData);
	//exponentialHookeSpringConstraint(staticSolverData, dynamicSolverData);
	//exponentialStvkSpringConstraint(staticSolverData, dynamicSolverData);
	//volumeStrainConstraintTet(staticSolverData, dynamicSolverData);

	exponentialGreenStrainConstraintTri(staticSolverData, dynamicSolverData);
	//exponentialGreenStrainConstraintModifiedTri(staticSolverData, dynamicSolverData);
	//exponentialGreenStrainConstraintTet(staticSolverData, dynamicSolverData);
	//exponentialGreenStrainConstraintModifiedTet(staticSolverData, dynamicSolverData);
	//areaStrainConstraintTri(staticSolverData, dynamicSolverData);
	//volumeStrainConstraintTet(staticSolverData, dynamicSolverData);
	
	collisionConstraints(staticSolverData, dynamicSolverData);
	positionConstraints(staticSolverData, dynamicSolverData);
}


//collisionConstraints
void PHYSSolverCPU::collisionConstraints(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	//CollisionGroundplane
	if (dynamicSolverData.collisionGroundplaneActive) 
		collisionConstraintGroundplane(staticSolverData, dynamicSolverData);
	
	//CollisionSpheres
	collisionConstraintSpheres(staticSolverData, dynamicSolverData);

	//CollisionConstraintConvex
	collisionConstraintConvex(staticSolverData, dynamicSolverData);
}


//collisionConstraintGroundplane
void PHYSSolverCPU::collisionConstraintGroundplane(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		for (int index = 0; index < staticSolverData.vertexCount; index++)
		{
			if (staticSolverData.pVertexPositionList[index].y < dynamicSolverData.groundplaneHeight)
				staticSolverData.pVertexPositionList[index].y = dynamicSolverData.groundplaneHeight;
		}
	}
}


//collisionConstraintSpheres
void PHYSSolverCPU::collisionConstraintSpheres(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	//iterate dynamicSolverData.pCollisionConstraintSpheresVecUpVecDownList
	for (int collisionSphereIndex = 0; collisionSphereIndex < dynamicSolverData.collisionConstraintSpheresCount * 2; collisionSphereIndex += 2)
	{
		//vecDownToUp and radiusSphere
		vcsVector vecUp = dynamicSolverData.pCollisionConstraintSpheresVecUpVecDownList[collisionSphereIndex];
		vcsVector vecDown = dynamicSolverData.pCollisionConstraintSpheresVecUpVecDownList[collisionSphereIndex + 1];
		vcsVector vecDownToUp = vecUp - vecDown;
		vcsVector vecSphereCenter = vecDown + (vecDownToUp * 0.5);
		float radiusSphere = (vecDownToUp * 0.52).length();

		omp_set_num_threads(NUMBER_OF_THREADS);
		//iterate each vertex in vertexPositionList and perform collision projection
		#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
		{
			#pragma omp for schedule(static) ordered
			for (int index = 0; index < staticSolverData.vertexCount; index++)
			{
				//get current vcsVector
				vcsVector vecCurrentPoint = staticSolverData.pVertexPositionList[index];
				//get vector sphereCenter to currentPoint and its length
				vcsVector vecSphereCenterToCurrentPoint = vecCurrentPoint - vecSphereCenter;
				float vecSphereCenterToCurrentPointLength = vecSphereCenterToCurrentPoint.length();

				//check if length < radius
				if (vecSphereCenterToCurrentPointLength < radiusSphere)
				{
					//project currentPoint outward
					vcsVector currentPointProjected = vecSphereCenter + (vecSphereCenterToCurrentPoint.normal() * radiusSphere);
					//set to solverData
					staticSolverData.pVertexPositionList[index] = currentPointProjected;
				}
			}
		}
	}
}


//collisionConstraintConvex
void PHYSSolverCPU::collisionConstraintConvex(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	//triangleCount
	int triangleCount = 0;
	for (int index = 0; index < dynamicSolverData.collisionConstraintConvexCount; index++)
	{
		triangleCount += dynamicSolverData.pCollisionConstraintConvexTriangleCountList[index];
	}

	//triangleVertexCount
	int triangleVertexCount = triangleCount * 3;

	//For each triangle in collisionConvexGeo
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		for (int triangleVertexIndex = 0; triangleVertexIndex < triangleVertexCount; triangleVertexIndex += 3)
		{
			//Get vecNormal and planeDistance
			//-----------------------------------------------

			//Get triangle vertices
			vcsVector triangleVertex1 = dynamicSolverData.pCollisionConstraintConvexTriangleVertexPositionList[triangleVertexIndex];
			vcsVector triangleVertex2 = dynamicSolverData.pCollisionConstraintConvexTriangleVertexPositionList[triangleVertexIndex + 1];
			vcsVector triangleVertex3 = dynamicSolverData.pCollisionConstraintConvexTriangleVertexPositionList[triangleVertexIndex + 2];

			//get triangle edges
			vcsVector vecTriangleEdge1 = triangleVertex2 - triangleVertex1;
			vcsVector vecTriangleEdge2 = triangleVertex3 - triangleVertex1;

			//crossProduct
			vcsVector crossProduct12 = vecTriangleEdge1.cross(vecTriangleEdge2);
			float crossProduct12Length = crossProduct12.length();
			if (crossProduct12Length == 0.0) crossProduct12Length = 0.000001;
			vcsVector crossProductNormalized = crossProduct12 / crossProduct12Length;

			//vcsVector crossProductNormalized = vecTriangleEdge1.cross(vecTriangleEdge2).normal();

			//For each point in vertexPositionList
			for (int vertexIndex = 0; vertexIndex < staticSolverData.vertexCount; vertexIndex++)
			{
				//CollisionCheckValue for vertexPositions
				//-----------------------------------------------

				//vertexPosition
				vcsVector vertexPosition = staticSolverData.pVertexPositionList[vertexIndex];
				float collisionCheckValueVertexPosition = (vertexPosition - triangleVertex1).dot(crossProductNormalized);
				
				//vertexOldposition
				vcsVector vertexOldposition = staticSolverData.pVertexOldpositionList[vertexIndex];
				float collisionCheckValueVertexOldposition = (vertexOldposition - triangleVertex1).dot(crossProductNormalized);
				
				//check if both are behind or in front
				if (!(collisionCheckValueVertexPosition < 0 && collisionCheckValueVertexOldposition > 0))
					continue;
				

				//Ray intersection with hyperplane
				//-----------------------------------------------

				//vecRay
				vcsVector vecRay = vertexPosition - vertexOldposition;

				float denom = vecRay.dot(crossProductNormalized);
				if (denom == 0.0) denom = 0.000001;

				//intersectionMultiplier
				float intersectionMultiplier = (triangleVertex1 - vertexOldposition).dot(crossProductNormalized) / denom;
				
				//vecIntersection
				vcsVector vecIntersection = vertexOldposition + (vecRay * intersectionMultiplier);

				//Check if intersection is on triangle
				//-----------------------------------------------

				//courses.cs.washington.edu/courses/cse457/05au/lectures/triangle_intersection.pdf
				float checkCorner1 = ((triangleVertex2 - triangleVertex1).cross(vecIntersection - triangleVertex1)).dot(crossProductNormalized);
				float checkCorner2 = ((triangleVertex3 - triangleVertex2).cross(vecIntersection - triangleVertex2)).dot(crossProductNormalized);
				float checkCorner3 = ((triangleVertex1 - triangleVertex3).cross(vecIntersection - triangleVertex3)).dot(crossProductNormalized);

				if (checkCorner1 >= 0.0 && checkCorner2 >= 0.0 && checkCorner3 >= 0.0)
				{
					vcsVector vecDiff = vertexOldposition - vertexPosition;
					float vecDiffLength = vecDiff.length();
					if (vecDiffLength == 0.0) vecDiffLength = 0.000001;
					vcsVector vecDiffNormalized = vecDiff / vecDiffLength;

					//vecOffset
					//vcsVector vecOffset = ((vertexOldposition - vecIntersection).normal()) * dynamicSolverData.meshThickness;
					vcsVector vecOffset = vecDiffNormalized * dynamicSolverData.meshThickness;

					//set position
					staticSolverData.pVertexPositionList[vertexIndex] = vecIntersection + vecOffset;
					//dynamicSolverData.deltaTime = 0.005;
				}
			}
		}
	}
}


//stretchingConstraint
void PHYSSolverCPU::stretchingConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate edges
		for (int index = 0; index < staticSolverData.edgeCount; index++)
		{
			//restLength
			float restLength = staticSolverData.pEdgeRestlengthList[index];
						
			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pEdgeVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pEdgeVertexIndexList[index].y);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			
			//Compute new vector positions for stretching
			vcsVector delta = vec2 - vec1;
			float deltaLength = delta.length();
			if (deltaLength == 0.0) deltaLength = 0.000001;
			float difference = deltaLength - restLength;

			/*if (index == 472)
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += difference;
				staticSolverData.potEneg = difference;
			}*/

			//gradients
			vcsVector gradientConstraint1 = (delta * (-1.0)) / deltaLength;
			vcsVector gradientConstraint2 = delta / deltaLength;
			float constraintLengthSum = pow(gradientConstraint1.length(), 2) + pow(gradientConstraint2.length(), 2);
			if (constraintLengthSum == 0.0) constraintLengthSum = 0.000001;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStretching1[index]);
			//float deltaLagrangeMultiplierDenominator = constraintLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator / deltaLagrangeMultiplierDenominator;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;

			float deltaLagrangeMultiplierNumerator1 = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStretching1[index]) + (gama * (gradientConstraint1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStretching2[index]) + (gama * (gradientConstraint2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			dynamicSolverData.totalLagrangeMultiplierStretching1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierStretching2[index] += deltaLagrangeMultiplier2;

			vec1 = vec1 + (gradientConstraint1 * 0.5 * deltaLagrangeMultiplier1);
			vec2 = vec2 + (gradientConstraint2 * 0.5 * deltaLagrangeMultiplier2);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
		}
	}
}


//centerofMassComputation
void PHYSSolverCPU::centerofMassComputation(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	vcsVector vertexCenterofMassTotal = vcsVector(0.0, 0.0, 0.0);

	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		for (int index = 0; index < staticSolverData.vertexCount; index++)
		{
			vertexCenterofMassTotal = vertexCenterofMassTotal + staticSolverData.pVertexPositionList[index];
		}

		staticSolverData.pCenterofMass[0] = vertexCenterofMassTotal / staticSolverData.vertexCount;
	}
}


//greenStrainConstraintTri
void PHYSSolverCPU::greenStrainConstraintTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
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

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			vcsVector vecDifference21 = vec2 - vec1;
			vcsVector vecDifference31 = vec3 - vec1;

			//materialCoordinate
			vcsVector materialCoordinateC1 = staticSolverData.pMaterialCoordinateInvC1List[index];
			vcsVector materialCoordinateC2 = staticSolverData.pMaterialCoordinateInvC2List[index];

			//deformationGradient matrix elements 3x2
			float deformationGradientF1X = (vecDifference21.x * materialCoordinateC1.x) + (vecDifference31.x * materialCoordinateC1.y);
			float deformationGradientF1Y = (vecDifference21.y * materialCoordinateC1.x) + (vecDifference31.y * materialCoordinateC1.y);
			float deformationGradientF1Z = (vecDifference21.z * materialCoordinateC1.x) + (vecDifference31.z * materialCoordinateC1.y);

			float deformationGradientF2X = (vecDifference21.x * materialCoordinateC2.x) + (vecDifference31.x * materialCoordinateC2.y);
			float deformationGradientF2Y = (vecDifference21.y * materialCoordinateC2.x) + (vecDifference31.y * materialCoordinateC2.y);
			float deformationGradientF2Z = (vecDifference21.z * materialCoordinateC2.x) + (vecDifference31.z * materialCoordinateC2.y);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);

			//strainTensor matrix elements 2x2
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, 0.0);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, 0.0);

			float strainConstraint11 = strainTensor1.x - 1.0;
			float strainConstraint12 = strainTensor2.x;
			float strainConstraint22 = strainTensor2.y - 1.0;

			/*if (index == 472)
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += difference;
				staticSolverData.potEneg = strainConstraint22;
			}*/

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * materialCoordinateC1.x,
													 deformationGradientF1Y * materialCoordinateC1.x,
													 deformationGradientF1Z * materialCoordinateC1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * materialCoordinateC1.y,
													 deformationGradientF1Y * materialCoordinateC1.y,
													 deformationGradientF1Z * materialCoordinateC1.y) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * materialCoordinateC2.x,
													 deformationGradientF2Y * materialCoordinateC2.x,
													 deformationGradientF2Z * materialCoordinateC2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * materialCoordinateC2.y,
													 deformationGradientF2Y * materialCoordinateC2.y,
													 deformationGradientF2Z * materialCoordinateC2.y) * 2.0;

			vcsVector strainGradient12_1 = vcsVector((deformationGradientF2X * materialCoordinateC1.x) + (deformationGradientF1X * materialCoordinateC2.x),
													 (deformationGradientF2Y * materialCoordinateC1.x) + (deformationGradientF1Y * materialCoordinateC2.x),
													 (deformationGradientF2Z * materialCoordinateC1.x) + (deformationGradientF1Z * materialCoordinateC2.x));
			vcsVector strainGradient12_2 = vcsVector((deformationGradientF2X * materialCoordinateC1.y) + (deformationGradientF1X * materialCoordinateC2.y),
													 (deformationGradientF2Y * materialCoordinateC1.y) + (deformationGradientF1Y * materialCoordinateC2.y),
													 (deformationGradientF2Z * materialCoordinateC1.y) + (deformationGradientF1Z * materialCoordinateC2.y));

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec2 = strainGradient11_1;
			vcsVector strainGradient11Vec3 = strainGradient11_2;
			vcsVector strainGradient11Vec1 = (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec2 = strainGradient12_1;
			vcsVector strainGradient12Vec3 = strainGradient12_2;
			vcsVector strainGradient12Vec1 = (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec2 = strainGradient22_1;
			vcsVector strainGradient22Vec3 = strainGradient22_2;
			vcsVector strainGradient22Vec1 = (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(strainGradient11Vec1.length(), 2) + pow(strainGradient11Vec2.length(), 2) + pow(strainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0.0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(strainGradient12Vec1.length(), 2) + pow(strainGradient12Vec2.length(), 2) + pow(strainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0.0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(strainGradient22Vec1.length(), 2) + pow(strainGradient22Vec2.length(), 2) + pow(strainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0.0) gradientLengthSum22 = 0.000001;

			float oneOverVertexWeights = 0.33;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (strainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (strainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (strainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (strainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (strainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (strainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (strainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (strainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (strainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 stretching
			vec1 = vec1 + (strainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (strainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (strainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (strainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (strainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (strainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (strainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (strainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (strainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//greenStrainConstraintModifiedTri
void PHYSSolverCPU::greenStrainConstraintModifiedTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
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

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			vcsVector vecDifference21 = vec2 - vec1;
			vcsVector vecDifference31 = vec3 - vec1;

			//materialCoordinate
			vcsVector materialCoordinateC1 = staticSolverData.pMaterialCoordinateInvC1List[index];
			vcsVector materialCoordinateC2 = staticSolverData.pMaterialCoordinateInvC2List[index];

			//deformationGradient matrix elements 3x2
			float deformationGradientF1X = (vecDifference21.x * materialCoordinateC1.x) + (vecDifference31.x * materialCoordinateC1.y);
			float deformationGradientF1Y = (vecDifference21.y * materialCoordinateC1.x) + (vecDifference31.y * materialCoordinateC1.y);
			float deformationGradientF1Z = (vecDifference21.z * materialCoordinateC1.x) + (vecDifference31.z * materialCoordinateC1.y);

			float deformationGradientF2X = (vecDifference21.x * materialCoordinateC2.x) + (vecDifference31.x * materialCoordinateC2.y);
			float deformationGradientF2Y = (vecDifference21.y * materialCoordinateC2.x) + (vecDifference31.y * materialCoordinateC2.y);
			float deformationGradientF2Z = (vecDifference21.z * materialCoordinateC2.x) + (vecDifference31.z * materialCoordinateC2.y);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);

			//strainTensor matrix elements 2x2
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, 0.0);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, 0.0);

			float strainConstraint11 = sqrt(strainTensor1.x) - 1.0;
			float strainConstraint12 = strainTensor2.x / (deformationGradientVector1.length() * deformationGradientVector2.length());
			float strainConstraint22 = sqrt(strainTensor2.y) - 1.0;

			float strain12DividedbyCubes = strainTensor2.x / (pow(deformationGradientVector1.length(), 3) * pow(deformationGradientVector2.length(), 3));
			float strain12DividedbyCubesMultDefGrad2 = pow(deformationGradientVector2.length(), 2) * strain12DividedbyCubes;
			float strain12DividedbyCubesMultDefGrad1 = pow(deformationGradientVector1.length(), 2) * strain12DividedbyCubes;
			float oneOverDefGradLengths12 = 1.0 / (deformationGradientVector1.length() * deformationGradientVector2.length());

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * materialCoordinateC1.x,
													 deformationGradientF1Y * materialCoordinateC1.x,
													 deformationGradientF1Z * materialCoordinateC1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * materialCoordinateC1.y,
													 deformationGradientF1Y * materialCoordinateC1.y,
													 deformationGradientF1Z * materialCoordinateC1.y) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * materialCoordinateC2.x,
													 deformationGradientF2Y * materialCoordinateC2.x,
													 deformationGradientF2Z * materialCoordinateC2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * materialCoordinateC2.y,
													 deformationGradientF2Y * materialCoordinateC2.y,
													 deformationGradientF2Z * materialCoordinateC2.y) * 2.0;

			vcsVector modifiedStrainGradient12_1_LeftofMinus = vcsVector((deformationGradientF2X * materialCoordinateC1.x) + (deformationGradientF1X * materialCoordinateC2.x),
																		 (deformationGradientF2Y * materialCoordinateC1.x) + (deformationGradientF1Y * materialCoordinateC2.x),
																		 (deformationGradientF2Z * materialCoordinateC1.x) + (deformationGradientF1Z * materialCoordinateC2.x)) * oneOverDefGradLengths12;
			vcsVector modifiedStrainGradient12_2_LeftofMinus = vcsVector((deformationGradientF2X * materialCoordinateC1.y) + (deformationGradientF1X * materialCoordinateC2.y),
																		 (deformationGradientF2Y * materialCoordinateC1.y) + (deformationGradientF1Y * materialCoordinateC2.y),
																		 (deformationGradientF2Z * materialCoordinateC1.y) + (deformationGradientF1Z * materialCoordinateC2.y)) * oneOverDefGradLengths12;

			vcsVector strainGradient12_1_RightofMinusPart1 = vcsVector(deformationGradientF1X * materialCoordinateC1.x,
																	   deformationGradientF1Y * materialCoordinateC1.x,
																	   deformationGradientF1Z * materialCoordinateC1.x) * strain12DividedbyCubesMultDefGrad2;
			vcsVector strainGradient12_2_RightofMinusPart1 = vcsVector(deformationGradientF1X * materialCoordinateC1.y,
																	   deformationGradientF1Y * materialCoordinateC1.y,
																	   deformationGradientF1Z * materialCoordinateC1.y) * strain12DividedbyCubesMultDefGrad2;

			vcsVector strainGradient12_1_RightofMinusPart2 = vcsVector(deformationGradientF2X * materialCoordinateC2.x,
																	   deformationGradientF2Y * materialCoordinateC2.x,
																	   deformationGradientF2Z * materialCoordinateC2.x) * strain12DividedbyCubesMultDefGrad1;
			vcsVector strainGradient12_2_RightofMinusPart2 = vcsVector(deformationGradientF2X * materialCoordinateC2.y,
																	   deformationGradientF2Y * materialCoordinateC2.y,
																	   deformationGradientF2Z * materialCoordinateC2.y) * strain12DividedbyCubesMultDefGrad1;

			vcsVector modifiedStrainGradient12_1 = modifiedStrainGradient12_1_LeftofMinus - (strainGradient12_1_RightofMinusPart1 + strainGradient12_1_RightofMinusPart2);
			vcsVector modifiedStrainGradient12_2 = modifiedStrainGradient12_2_LeftofMinus - (strainGradient12_2_RightofMinusPart1 + strainGradient12_2_RightofMinusPart2);

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec2 = strainGradient11_1;
			vcsVector strainGradient11Vec3 = strainGradient11_2;
			vcsVector strainGradient11Vec1 = (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec2 = modifiedStrainGradient12_1;
			vcsVector strainGradient12Vec3 = modifiedStrainGradient12_2;
			vcsVector strainGradient12Vec1 = (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec2 = strainGradient22_1;
			vcsVector strainGradient22Vec3 = strainGradient22_2;
			vcsVector strainGradient22Vec1 = (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(strainGradient11Vec1.length(), 2) + pow(strainGradient11Vec2.length(), 2) + pow(strainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0.0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(strainGradient12Vec1.length(), 2) + pow(strainGradient12Vec2.length(), 2) + pow(strainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0.0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(strainGradient22Vec1.length(), 2) + pow(strainGradient22Vec2.length(), 2) + pow(strainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0.0) gradientLengthSum22 = 0.000001;

			float oneOverVertexWeights = 0.33;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = (2.0 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = (2.0 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (strainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = (2.0 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (strainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = (2.0 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (strainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (strainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (strainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (strainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (strainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (strainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (strainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 stretching
			vec1 = vec1 + (strainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (strainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (strainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = (2.0 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = (2.0 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (strainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = (2.0 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (strainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = (2.0 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (strainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (strainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (strainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (strainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//greenStrainConstraintTet
void PHYSSolverCPU::greenStrainConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec0 = staticSolverData.pCenterofMass[0];
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vertex differences with center of mass for fake tet
			vcsVector diffVec1nVec0 = vec1 - vec0;
			vcsVector diffVec2nVec0 = vec2 - vec0;
			vcsVector diffVec3nVec0 = vec3 - vec0;

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			//materialCoordinate with initial center of mass differences for fake tet
			vcsVector materialCoordinateC1 = staticSolverData.pVertexInitialpositionList[vertexIndex1] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC2 = staticSolverData.pVertexInitialpositionList[vertexIndex2] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC3 = staticSolverData.pVertexInitialpositionList[vertexIndex3] - staticSolverData.initialCenterofMass;

			float determinantMatCoord = (materialCoordinateC1.x * materialCoordinateC2.y * materialCoordinateC3.z) +
										(materialCoordinateC2.x * materialCoordinateC3.y * materialCoordinateC1.z) +
										(materialCoordinateC3.x * materialCoordinateC1.y * materialCoordinateC2.z) -
										(materialCoordinateC3.x * materialCoordinateC2.y * materialCoordinateC1.z) -
										(materialCoordinateC1.x * materialCoordinateC3.y * materialCoordinateC2.z) -
										(materialCoordinateC2.x * materialCoordinateC1.y * materialCoordinateC3.z);
			if (determinantMatCoord == 0.0) determinantMatCoord = 0.000001;

			vcsVector inverseMatCoord1 = vcsVector(materialCoordinateC2.y * materialCoordinateC3.z - materialCoordinateC3.y * materialCoordinateC2.z,
												   materialCoordinateC3.y * materialCoordinateC1.z - materialCoordinateC1.y * materialCoordinateC3.z,
												   materialCoordinateC1.y * materialCoordinateC2.z - materialCoordinateC2.y * materialCoordinateC1.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord2 = vcsVector(materialCoordinateC3.x * materialCoordinateC2.z - materialCoordinateC2.x * materialCoordinateC3.z,
												   materialCoordinateC1.x * materialCoordinateC3.z - materialCoordinateC3.x * materialCoordinateC1.z,
												   materialCoordinateC2.x * materialCoordinateC1.z - materialCoordinateC1.x * materialCoordinateC2.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord3 = vcsVector(materialCoordinateC2.x * materialCoordinateC3.y - materialCoordinateC3.x * materialCoordinateC2.y,
												   materialCoordinateC3.x * materialCoordinateC1.y - materialCoordinateC1.x * materialCoordinateC3.y,
												   materialCoordinateC1.x * materialCoordinateC2.y - materialCoordinateC2.x * materialCoordinateC1.y) * (1.0 / determinantMatCoord);


			//deformationGradient matrix elements 3x3
			float deformationGradientF1X = (diffVec1nVec0.x * inverseMatCoord1.x) + (diffVec2nVec0.x * inverseMatCoord1.y) + (diffVec3nVec0.x * inverseMatCoord1.z);
			float deformationGradientF1Y = (diffVec1nVec0.y * inverseMatCoord1.x) + (diffVec2nVec0.y * inverseMatCoord1.y) + (diffVec3nVec0.y * inverseMatCoord1.z);
			float deformationGradientF1Z = (diffVec1nVec0.z * inverseMatCoord1.x) + (diffVec2nVec0.z * inverseMatCoord1.y) + (diffVec3nVec0.z * inverseMatCoord1.z);

			float deformationGradientF2X = (diffVec1nVec0.x * inverseMatCoord2.x) + (diffVec2nVec0.x * inverseMatCoord2.y) + (diffVec3nVec0.x * inverseMatCoord2.z);
			float deformationGradientF2Y = (diffVec1nVec0.y * inverseMatCoord2.x) + (diffVec2nVec0.y * inverseMatCoord2.y) + (diffVec3nVec0.y * inverseMatCoord2.z);
			float deformationGradientF2Z = (diffVec1nVec0.z * inverseMatCoord2.x) + (diffVec2nVec0.z * inverseMatCoord2.y) + (diffVec3nVec0.z * inverseMatCoord2.z);

			float deformationGradientF3X = (diffVec1nVec0.x * inverseMatCoord3.x) + (diffVec2nVec0.x * inverseMatCoord3.y) + (diffVec3nVec0.x * inverseMatCoord3.z);
			float deformationGradientF3Y = (diffVec1nVec0.y * inverseMatCoord3.x) + (diffVec2nVec0.y * inverseMatCoord3.y) + (diffVec3nVec0.y * inverseMatCoord3.z);
			float deformationGradientF3Z = (diffVec1nVec0.z * inverseMatCoord3.x) + (diffVec2nVec0.z * inverseMatCoord3.y) + (diffVec3nVec0.z * inverseMatCoord3.z);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);
			vcsVector deformationGradientVector3 = vcsVector(deformationGradientF3X, deformationGradientF3Y, deformationGradientF3Z);

			//strainTensor matrix elements 3x3
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);
			float strainTensorS1Z = deformationGradientVector3.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);
			float strainTensorS2Z = deformationGradientVector3.dot(deformationGradientVector2);

			float strainTensorS3X = deformationGradientVector1.dot(deformationGradientVector3);
			float strainTensorS3Y = deformationGradientVector2.dot(deformationGradientVector3);
			float strainTensorS3Z = deformationGradientVector3.dot(deformationGradientVector3);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, strainTensorS1Z);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, strainTensorS2Z);
			vcsVector strainTensor3 = vcsVector(strainTensorS3X, strainTensorS3Y, strainTensorS3Z);

			float strainConstraint11 = strainTensor1.x - 1.0;
			float strainConstraint22 = strainTensor2.y - 1.0;
			float strainConstraint33 = strainTensor3.z - 1.0;
			float strainConstraint12 = strainTensor2.x;
			float strainConstraint13 = strainTensor3.x;
			float strainConstraint23 = strainTensor3.y;

			/*if (index == 242)
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += difference;
				staticSolverData.potEneg = strainConstraint12 + strainConstraint13 + strainConstraint23;
			}*/

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
													 deformationGradientF1Y * inverseMatCoord1.x,
													 deformationGradientF1Z * inverseMatCoord1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
													 deformationGradientF1Y * inverseMatCoord1.y,
													 deformationGradientF1Z * inverseMatCoord1.y) * 2.0;
			vcsVector strainGradient11_3 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
													 deformationGradientF1Y * inverseMatCoord1.z,
													 deformationGradientF1Z * inverseMatCoord1.z) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
													 deformationGradientF2Y * inverseMatCoord2.x,
													 deformationGradientF2Z * inverseMatCoord2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
													 deformationGradientF2Y * inverseMatCoord2.y,
													 deformationGradientF2Z * inverseMatCoord2.y) * 2.0;
			vcsVector strainGradient22_3 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
													 deformationGradientF2Y * inverseMatCoord2.z,
													 deformationGradientF2Z * inverseMatCoord2.z) * 2.0;

			vcsVector strainGradient33_1 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
													 deformationGradientF3Y * inverseMatCoord3.x,
													 deformationGradientF3Z * inverseMatCoord3.x) * 2.0;
			vcsVector strainGradient33_2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
													 deformationGradientF3Y * inverseMatCoord3.y,
													 deformationGradientF3Z * inverseMatCoord3.y) * 2.0;
			vcsVector strainGradient33_3 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
													 deformationGradientF3Y * inverseMatCoord3.z,
													 deformationGradientF3Z * inverseMatCoord3.z) * 2.0;

			vcsVector strainGradient12_1 = vcsVector(((deformationGradientF2X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord2.x)),
													 ((deformationGradientF2Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord2.x)),
													 ((deformationGradientF2Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord2.x)));
			vcsVector strainGradient12_2 = vcsVector(((deformationGradientF2X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord2.y)),
													 ((deformationGradientF2Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord2.y)),
													 ((deformationGradientF2Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord2.y)));
			vcsVector strainGradient12_3 = vcsVector(((deformationGradientF2X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord2.z)),
													 ((deformationGradientF2Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord2.z)),
													 ((deformationGradientF2Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord2.z)));

			vcsVector strainGradient13_1 = vcsVector(((deformationGradientF3X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord3.x)),
													 ((deformationGradientF3Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord3.x)),
													 ((deformationGradientF3Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord3.x)));
			vcsVector strainGradient13_2 = vcsVector(((deformationGradientF3X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord3.y)),
													 ((deformationGradientF3Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord3.y)),
													 ((deformationGradientF3Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord3.y)));
			vcsVector strainGradient13_3 = vcsVector(((deformationGradientF3X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord3.z)),
													 ((deformationGradientF3Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord3.z)),
													 ((deformationGradientF3Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord3.z)));

			vcsVector strainGradient23_1 = vcsVector(((deformationGradientF3X * inverseMatCoord2.x) + (deformationGradientF2X * inverseMatCoord3.x)),
													 ((deformationGradientF3Y * inverseMatCoord2.x) + (deformationGradientF2Y * inverseMatCoord3.x)),
													 ((deformationGradientF3Z * inverseMatCoord2.x) + (deformationGradientF2Z * inverseMatCoord3.x)));
			vcsVector strainGradient23_2 = vcsVector(((deformationGradientF3X * inverseMatCoord2.y) + (deformationGradientF2X * inverseMatCoord3.y)),
													 ((deformationGradientF3Y * inverseMatCoord2.y) + (deformationGradientF2Y * inverseMatCoord3.y)),
													 ((deformationGradientF3Z * inverseMatCoord2.y) + (deformationGradientF2Z * inverseMatCoord3.y)));
			vcsVector strainGradient23_3 = vcsVector(((deformationGradientF3X * inverseMatCoord2.z) + (deformationGradientF2X * inverseMatCoord3.z)),
													 ((deformationGradientF3Y * inverseMatCoord2.z) + (deformationGradientF2Y * inverseMatCoord3.z)),
													 ((deformationGradientF3Z * inverseMatCoord2.z) + (deformationGradientF2Z * inverseMatCoord3.z)));

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec1 = strainGradient11_1;
			vcsVector strainGradient11Vec2 = strainGradient11_2;
			vcsVector strainGradient11Vec3 = strainGradient11_3;
			vcsVector strainGradient11Vec0 = (strainGradient11Vec1 * (-1.0)) + (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec1 = strainGradient22_1;
			vcsVector strainGradient22Vec2 = strainGradient22_2;
			vcsVector strainGradient22Vec3 = strainGradient22_3;
			vcsVector strainGradient22Vec0 = (strainGradient22Vec1 * (-1.0)) + (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			//strainGradientOverVertices 33 matrix
			vcsVector strainGradient33Vec1 = strainGradient33_1;
			vcsVector strainGradient33Vec2 = strainGradient33_2;
			vcsVector strainGradient33Vec3 = strainGradient33_3;
			vcsVector strainGradient33Vec0 = (strainGradient33Vec1 * (-1.0)) + (strainGradient33Vec2 * (-1.0)) + (strainGradient33Vec3 * (-1.0));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec1 = strainGradient12_1;
			vcsVector strainGradient12Vec2 = strainGradient12_2;
			vcsVector strainGradient12Vec3 = strainGradient12_3;
			vcsVector strainGradient12Vec0 = (strainGradient12Vec1 * (-1.0)) + (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			//strainGradientOverVertices 13 matrix
			vcsVector strainGradient13Vec1 = strainGradient13_1;
			vcsVector strainGradient13Vec2 = strainGradient13_2;
			vcsVector strainGradient13Vec3 = strainGradient13_3;
			vcsVector strainGradient13Vec0 = (strainGradient13Vec1 * (-1.0)) + (strainGradient13Vec2 * (-1.0)) + (strainGradient13Vec3 * (-1.0));

			//strainGradientOverVertices 23 matrix
			vcsVector strainGradient23Vec1 = strainGradient23_1;
			vcsVector strainGradient23Vec2 = strainGradient23_2;
			vcsVector strainGradient23Vec3 = strainGradient23_3;
			vcsVector strainGradient23Vec0 = (strainGradient23Vec1 * (-1.0)) + (strainGradient23Vec2 * (-1.0)) + (strainGradient23Vec3 * (-1.0));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(strainGradient11Vec0.length(), 2) + pow(strainGradient11Vec1.length(), 2) + pow(strainGradient11Vec2.length(), 2) + pow(strainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(strainGradient22Vec0.length(), 2) + pow(strainGradient22Vec1.length(), 2) + pow(strainGradient22Vec2.length(), 2) + pow(strainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0) gradientLengthSum22 = 0.000001;

			//gradientLengthSum33
			float gradientLengthSum33 = pow(strainGradient33Vec0.length(), 2) + pow(strainGradient33Vec1.length(), 2) + pow(strainGradient33Vec2.length(), 2) + pow(strainGradient33Vec3.length(), 2);
			if (gradientLengthSum33 == 0) gradientLengthSum33 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(strainGradient12Vec0.length(), 2) + pow(strainGradient12Vec1.length(), 2) + pow(strainGradient12Vec2.length(), 2) + pow(strainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum13
			float gradientLengthSum13 = pow(strainGradient13Vec0.length(), 2) + pow(strainGradient13Vec1.length(), 2) + pow(strainGradient13Vec2.length(), 2) + pow(strainGradient13Vec3.length(), 2);
			if (gradientLengthSum13 == 0) gradientLengthSum13 = 0.000001;

			//gradientLengthSum23
			float gradientLengthSum23 = pow(strainGradient23Vec0.length(), 2) + pow(strainGradient23Vec1.length(), 2) + pow(strainGradient23Vec2.length(), 2) + pow(strainGradient23Vec3.length(), 2);
			if (gradientLengthSum23 == 0) gradientLengthSum23 = 0.000001;

			float oneOverVertexWeights = 0.25;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (strainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (strainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (strainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (strainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (strainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (strainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (strainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (strainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (strainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (strainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (strainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (strainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_33 = strainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]);
			//float deltaLagrangeMultiplierDenominator1_33 = gradientLengthSum33 + alphaHat;
			//deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator1_33 = strainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]) + (gama * (strainGradient33Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator2_33 = strainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_33[index]) + (gama * (strainGradient33Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier2_33 = -deltaLagrangeMultiplierNumerator2_33 / deltaLagrangeMultiplierDenominator2_33;

			float deltaLagrangeMultiplierNumerator3_33 = strainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_33[index]) + (gama * (strainGradient33Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier3_33 = -deltaLagrangeMultiplierNumerator3_33 / deltaLagrangeMultiplierDenominator3_33;

			dynamicSolverData.totalLagrangeMultiplierStrain1_33[index] += deltaLagrangeMultiplier1_33;
			dynamicSolverData.totalLagrangeMultiplierStrain2_33[index] += deltaLagrangeMultiplier2_33;
			dynamicSolverData.totalLagrangeMultiplierStrain3_33[index] += deltaLagrangeMultiplier3_33;

			//Compute new vector positions for strainTensor33 stretching
			vec1 = vec1 + (strainGradient33Vec1 * deltaLagrangeMultiplier1_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec2 = vec2 + (strainGradient33Vec2 * deltaLagrangeMultiplier2_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec3 = vec3 + (strainGradient33Vec3 * deltaLagrangeMultiplier3_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (strainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (strainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (strainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 shearing
			vec1 = vec1 + (strainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (strainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (strainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]);
			//float deltaLagrangeMultiplierDenominator1_13 = gradientLengthSum13 + alphaHat;
			//deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator1_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]) + (gama * (strainGradient13Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator2_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_13[index]) + (gama * (strainGradient13Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier2_13 = -deltaLagrangeMultiplierNumerator2_13 / deltaLagrangeMultiplierDenominator2_13;

			float deltaLagrangeMultiplierNumerator3_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_13[index]) + (gama * (strainGradient13Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier3_13 = -deltaLagrangeMultiplierNumerator3_13 / deltaLagrangeMultiplierDenominator3_13;

			dynamicSolverData.totalLagrangeMultiplierStrain1_13[index] += deltaLagrangeMultiplier1_13;
			dynamicSolverData.totalLagrangeMultiplierStrain2_13[index] += deltaLagrangeMultiplier2_13;
			dynamicSolverData.totalLagrangeMultiplierStrain3_13[index] += deltaLagrangeMultiplier3_13;

			//Compute new vector positions for strainTensor13 shearing
			vec1 = vec1 + (strainGradient13Vec1 * deltaLagrangeMultiplier1_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec2 = vec2 + (strainGradient13Vec2 * deltaLagrangeMultiplier2_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec3 = vec3 + (strainGradient13Vec3 * deltaLagrangeMultiplier3_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]);
			//float deltaLagrangeMultiplierDenominator1_23 = gradientLengthSum23 + alphaHat;
			//deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator1_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]) + (gama * (strainGradient23Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator2_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_23[index]) + (gama * (strainGradient23Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier2_23 = -deltaLagrangeMultiplierNumerator2_23 / deltaLagrangeMultiplierDenominator2_23;

			float deltaLagrangeMultiplierNumerator3_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_23[index]) + (gama * (strainGradient23Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier3_23 = -deltaLagrangeMultiplierNumerator3_23 / deltaLagrangeMultiplierDenominator3_23;

			dynamicSolverData.totalLagrangeMultiplierStrain1_23[index] += deltaLagrangeMultiplier1_23;
			dynamicSolverData.totalLagrangeMultiplierStrain2_23[index] += deltaLagrangeMultiplier2_23;
			dynamicSolverData.totalLagrangeMultiplierStrain3_23[index] += deltaLagrangeMultiplier3_23;

			//Compute new vector positions for strainTensor23 shearing
			vec1 = vec1 + (strainGradient23Vec1 * deltaLagrangeMultiplier1_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec2 = vec2 + (strainGradient23Vec2 * deltaLagrangeMultiplier2_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec3 = vec3 + (strainGradient23Vec3 * deltaLagrangeMultiplier3_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//greenStrainConstraintModifiedTet
void PHYSSolverCPU::greenStrainConstraintModifiedTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec0 = staticSolverData.pCenterofMass[0];
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vertex differences with center of mass for fake tet
			vcsVector diffVec1nVec0 = vec1 - vec0;
			vcsVector diffVec2nVec0 = vec2 - vec0;
			vcsVector diffVec3nVec0 = vec3 - vec0;

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			//materialCoordinate with initial center of mass differences for fake tet
			vcsVector materialCoordinateC1 = staticSolverData.pVertexInitialpositionList[vertexIndex1] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC2 = staticSolverData.pVertexInitialpositionList[vertexIndex2] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC3 = staticSolverData.pVertexInitialpositionList[vertexIndex3] - staticSolverData.initialCenterofMass;

			float determinantMatCoord = (materialCoordinateC1.x * materialCoordinateC2.y * materialCoordinateC3.z) +
										(materialCoordinateC2.x * materialCoordinateC3.y * materialCoordinateC1.z) +
										(materialCoordinateC3.x * materialCoordinateC1.y * materialCoordinateC2.z) -
										(materialCoordinateC3.x * materialCoordinateC2.y * materialCoordinateC1.z) -
										(materialCoordinateC1.x * materialCoordinateC3.y * materialCoordinateC2.z) -
										(materialCoordinateC2.x * materialCoordinateC1.y * materialCoordinateC3.z);
			if (determinantMatCoord == 0.0) determinantMatCoord = 0.000001;


			vcsVector inverseMatCoord1 = vcsVector(materialCoordinateC2.y * materialCoordinateC3.z - materialCoordinateC3.y * materialCoordinateC2.z,
												   materialCoordinateC3.y * materialCoordinateC1.z - materialCoordinateC1.y * materialCoordinateC3.z,
												   materialCoordinateC1.y * materialCoordinateC2.z - materialCoordinateC2.y * materialCoordinateC1.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord2 = vcsVector(materialCoordinateC3.x * materialCoordinateC2.z - materialCoordinateC2.x * materialCoordinateC3.z,
												   materialCoordinateC1.x * materialCoordinateC3.z - materialCoordinateC3.x * materialCoordinateC1.z,
												   materialCoordinateC2.x * materialCoordinateC1.z - materialCoordinateC1.x * materialCoordinateC2.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord3 = vcsVector(materialCoordinateC2.x * materialCoordinateC3.y - materialCoordinateC3.x * materialCoordinateC2.y,
												   materialCoordinateC3.x * materialCoordinateC1.y - materialCoordinateC1.x * materialCoordinateC3.y,
												   materialCoordinateC1.x * materialCoordinateC2.y - materialCoordinateC2.x * materialCoordinateC1.y) * (1.0 / determinantMatCoord);


			//deformationGradient matrix elements 3x3
			float deformationGradientF1X = (diffVec1nVec0.x * inverseMatCoord1.x) + (diffVec2nVec0.x * inverseMatCoord1.y) + (diffVec3nVec0.x * inverseMatCoord1.z);
			float deformationGradientF1Y = (diffVec1nVec0.y * inverseMatCoord1.x) + (diffVec2nVec0.y * inverseMatCoord1.y) + (diffVec3nVec0.y * inverseMatCoord1.z);
			float deformationGradientF1Z = (diffVec1nVec0.z * inverseMatCoord1.x) + (diffVec2nVec0.z * inverseMatCoord1.y) + (diffVec3nVec0.z * inverseMatCoord1.z);

			float deformationGradientF2X = (diffVec1nVec0.x * inverseMatCoord2.x) + (diffVec2nVec0.x * inverseMatCoord2.y) + (diffVec3nVec0.x * inverseMatCoord2.z);
			float deformationGradientF2Y = (diffVec1nVec0.y * inverseMatCoord2.x) + (diffVec2nVec0.y * inverseMatCoord2.y) + (diffVec3nVec0.y * inverseMatCoord2.z);
			float deformationGradientF2Z = (diffVec1nVec0.z * inverseMatCoord2.x) + (diffVec2nVec0.z * inverseMatCoord2.y) + (diffVec3nVec0.z * inverseMatCoord2.z);

			float deformationGradientF3X = (diffVec1nVec0.x * inverseMatCoord3.x) + (diffVec2nVec0.x * inverseMatCoord3.y) + (diffVec3nVec0.x * inverseMatCoord3.z);
			float deformationGradientF3Y = (diffVec1nVec0.y * inverseMatCoord3.x) + (diffVec2nVec0.y * inverseMatCoord3.y) + (diffVec3nVec0.y * inverseMatCoord3.z);
			float deformationGradientF3Z = (diffVec1nVec0.z * inverseMatCoord3.x) + (diffVec2nVec0.z * inverseMatCoord3.y) + (diffVec3nVec0.z * inverseMatCoord3.z);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);
			vcsVector deformationGradientVector3 = vcsVector(deformationGradientF3X, deformationGradientF3Y, deformationGradientF3Z);

			//strainTensor matrix elements 3x3
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);
			float strainTensorS1Z = deformationGradientVector3.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);
			float strainTensorS2Z = deformationGradientVector3.dot(deformationGradientVector2);

			float strainTensorS3X = deformationGradientVector1.dot(deformationGradientVector3);
			float strainTensorS3Y = deformationGradientVector2.dot(deformationGradientVector3);
			float strainTensorS3Z = deformationGradientVector3.dot(deformationGradientVector3);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, strainTensorS1Z);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, strainTensorS2Z);
			vcsVector strainTensor3 = vcsVector(strainTensorS3X, strainTensorS3Y, strainTensorS3Z);

			float strainConstraint11 = sqrt(strainTensor1.x) - 1.0;
			float strainConstraint22 = sqrt(strainTensor2.y) - 1.0;
			float strainConstraint33 = sqrt(strainTensor3.z) - 1.0;
			float strainConstraint12 = strainTensor2.x / (deformationGradientVector1.length() * deformationGradientVector2.length());
			float strainConstraint13 = strainTensor3.x / (deformationGradientVector1.length() * deformationGradientVector3.length());
			float strainConstraint23 = strainTensor3.y / (deformationGradientVector2.length() * deformationGradientVector3.length());

			float squareofDefGrad1 = pow(deformationGradientVector1.length(), 2);
			if (squareofDefGrad1 > 10.0) squareofDefGrad1 = 10.0;
			float squareofDefGrad2 = pow(deformationGradientVector2.length(), 2);
			if (squareofDefGrad2 > 10.0) squareofDefGrad2 = 10.0;
			float squareofDefGrad3 = pow(deformationGradientVector3.length(), 2);
			if (squareofDefGrad3 > 10.0) squareofDefGrad3 = 10.0;

			float cubeofDefGrad1 = pow(deformationGradientVector1.length(), 3);
			if (cubeofDefGrad1 > 20.0) cubeofDefGrad1 = 20.0;
			float cubeofDefGrad2 = pow(deformationGradientVector2.length(), 3);
			if (cubeofDefGrad2 > 20.0) cubeofDefGrad2 = 20.0;
			float cubeofDefGrad3 = pow(deformationGradientVector3.length(), 3);
			if (cubeofDefGrad3 > 20.0) cubeofDefGrad3 = 20.0;

			float strain12DividedbyCubes = strainTensor2.x / (cubeofDefGrad1 * cubeofDefGrad2);
			float oneOverDefGradLengths12 = 1.0 / (deformationGradientVector1.length() * deformationGradientVector2.length());

			float strain13DividedbyCubes = strainTensor3.x / (cubeofDefGrad1 * cubeofDefGrad3);
			float oneOverDefGradLengths13 = 1.0 / (deformationGradientVector1.length() * deformationGradientVector3.length());

			float strain23DividedbyCubes = strainTensor3.y / (cubeofDefGrad2 * cubeofDefGrad3);
			float oneOverDefGradLengths23 = 1.0 / (deformationGradientVector2.length() * deformationGradientVector3.length());

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
													 deformationGradientF1Y * inverseMatCoord1.x,
													 deformationGradientF1Z * inverseMatCoord1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
													 deformationGradientF1Y * inverseMatCoord1.y,
													 deformationGradientF1Z * inverseMatCoord1.y) * 2.0;
			vcsVector strainGradient11_3 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
													 deformationGradientF1Y * inverseMatCoord1.z,
													 deformationGradientF1Z * inverseMatCoord1.z) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
													 deformationGradientF2Y * inverseMatCoord2.x,
													 deformationGradientF2Z * inverseMatCoord2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
													 deformationGradientF2Y * inverseMatCoord2.y,
													 deformationGradientF2Z * inverseMatCoord2.y) * 2.0;
			vcsVector strainGradient22_3 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
													 deformationGradientF2Y * inverseMatCoord2.z,
													 deformationGradientF2Z * inverseMatCoord2.z) * 2.0;

			vcsVector strainGradient33_1 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
													 deformationGradientF3Y * inverseMatCoord3.x,
													 deformationGradientF3Z * inverseMatCoord3.x) * 2.0;
			vcsVector strainGradient33_2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
													 deformationGradientF3Y * inverseMatCoord3.y,
													 deformationGradientF3Z * inverseMatCoord3.y) * 2.0;
			vcsVector strainGradient33_3 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
													 deformationGradientF3Y * inverseMatCoord3.z,
													 deformationGradientF3Z * inverseMatCoord3.z) * 2.0;

			vcsVector modifiedStrainGradient12_1_LeftofMinus = vcsVector((deformationGradientF2X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord2.x),
																		 (deformationGradientF2Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord2.x),
																		 (deformationGradientF2Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord2.x));
			vcsVector modifiedStrainGradient12_2_LeftofMinus = vcsVector((deformationGradientF2X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord2.y),
																		 (deformationGradientF2Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord2.y),
																		 (deformationGradientF2Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord2.y));
			vcsVector modifiedStrainGradient12_3_LeftofMinus = vcsVector((deformationGradientF2X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord2.z),
																		 (deformationGradientF2Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord2.z),
																		 (deformationGradientF2Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord2.z));

			vcsVector strainGradient12_1_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
																	   deformationGradientF1Y * inverseMatCoord1.x,
																	   deformationGradientF1Z * inverseMatCoord1.x);
			vcsVector strainGradient12_2_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
																	   deformationGradientF1Y * inverseMatCoord1.y,
																	   deformationGradientF1Z * inverseMatCoord1.y);
			vcsVector strainGradient12_3_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
																	   deformationGradientF1Y * inverseMatCoord1.z,
																	   deformationGradientF1Z * inverseMatCoord1.z);

			vcsVector strainGradient12_1_RightofMinusPart2 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
																	   deformationGradientF2Y * inverseMatCoord2.x,
																	   deformationGradientF2Z * inverseMatCoord2.x);
			vcsVector strainGradient12_2_RightofMinusPart2 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
																	   deformationGradientF2Y * inverseMatCoord2.y,
																	   deformationGradientF2Z * inverseMatCoord2.y);
			vcsVector strainGradient12_3_RightofMinusPart2 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
																	   deformationGradientF2Y * inverseMatCoord2.z,
																	   deformationGradientF2Z * inverseMatCoord2.z);

			vcsVector modifiedStrainGradient12_1_RightofMinus = (strainGradient12_1_RightofMinusPart1 * squareofDefGrad2) + (strainGradient12_1_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient12_2_RightofMinus = (strainGradient12_2_RightofMinusPart1 * squareofDefGrad2) + (strainGradient12_2_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient12_3_RightofMinus = (strainGradient12_3_RightofMinusPart1 * squareofDefGrad2) + (strainGradient12_3_RightofMinusPart2 * squareofDefGrad1);

			vcsVector modifiedStrainGradient12_1 = (modifiedStrainGradient12_1_LeftofMinus * oneOverDefGradLengths12) - (modifiedStrainGradient12_1_RightofMinus * strain12DividedbyCubes);
			vcsVector modifiedStrainGradient12_2 = (modifiedStrainGradient12_2_LeftofMinus * oneOverDefGradLengths12) - (modifiedStrainGradient12_2_RightofMinus * strain12DividedbyCubes);
			vcsVector modifiedStrainGradient12_3 = (modifiedStrainGradient12_3_LeftofMinus * oneOverDefGradLengths12) - (modifiedStrainGradient12_3_RightofMinus * strain12DividedbyCubes);

			vcsVector modifiedStrainGradient13_1_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord3.x),
																		 (deformationGradientF3Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord3.x),
																		 (deformationGradientF3Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord3.x));
			vcsVector modifiedStrainGradient13_2_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord3.y),
																		 (deformationGradientF3Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord3.y),
																		 (deformationGradientF3Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord3.y));
			vcsVector modifiedStrainGradient13_3_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord3.z),
																		 (deformationGradientF3Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord3.z),
																		 (deformationGradientF3Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord3.z));

			vcsVector strainGradient13_1_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
																	   deformationGradientF1Y * inverseMatCoord1.x,
																	   deformationGradientF1Z * inverseMatCoord1.x);
			vcsVector strainGradient13_2_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
																	   deformationGradientF1Y * inverseMatCoord1.y,
																	   deformationGradientF1Z * inverseMatCoord1.y);
			vcsVector strainGradient13_3_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
																	   deformationGradientF1Y * inverseMatCoord1.z,
																	   deformationGradientF1Z * inverseMatCoord1.z);

			vcsVector strainGradient13_1_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
																	   deformationGradientF3Y * inverseMatCoord3.x,
																	   deformationGradientF3Z * inverseMatCoord3.x);
			vcsVector strainGradient13_2_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
																	   deformationGradientF3Y * inverseMatCoord3.y,
																	   deformationGradientF3Z * inverseMatCoord3.y);
			vcsVector strainGradient13_3_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
																	   deformationGradientF3Y * inverseMatCoord3.z,
																	   deformationGradientF3Z * inverseMatCoord3.z);

			vcsVector modifiedStrainGradient13_1_RightofMinus = (strainGradient13_1_RightofMinusPart1 * squareofDefGrad3) + (strainGradient13_1_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient13_2_RightofMinus = (strainGradient13_2_RightofMinusPart1 * squareofDefGrad3) + (strainGradient13_2_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient13_3_RightofMinus = (strainGradient13_3_RightofMinusPart1 * squareofDefGrad3) + (strainGradient13_3_RightofMinusPart2 * squareofDefGrad1);

			vcsVector modifiedStrainGradient13_1 = (modifiedStrainGradient13_1_LeftofMinus * oneOverDefGradLengths13) - (modifiedStrainGradient13_1_RightofMinus * strain13DividedbyCubes);
			vcsVector modifiedStrainGradient13_2 = (modifiedStrainGradient13_2_LeftofMinus * oneOverDefGradLengths13) - (modifiedStrainGradient13_2_RightofMinus * strain13DividedbyCubes);
			vcsVector modifiedStrainGradient13_3 = (modifiedStrainGradient13_3_LeftofMinus * oneOverDefGradLengths13) - (modifiedStrainGradient13_3_RightofMinus * strain13DividedbyCubes);

			vcsVector modifiedStrainGradient23_1_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord2.x) + (deformationGradientF2X * inverseMatCoord3.x),
																		 (deformationGradientF3Y * inverseMatCoord2.x) + (deformationGradientF2Y * inverseMatCoord3.x),
																		 (deformationGradientF3Z * inverseMatCoord2.x) + (deformationGradientF2Z * inverseMatCoord3.x));
			vcsVector modifiedStrainGradient23_2_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord2.y) + (deformationGradientF2X * inverseMatCoord3.y),
																		 (deformationGradientF3Y * inverseMatCoord2.y) + (deformationGradientF2Y * inverseMatCoord3.y),
																		 (deformationGradientF3Z * inverseMatCoord2.y) + (deformationGradientF2Z * inverseMatCoord3.y));
			vcsVector modifiedStrainGradient23_3_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord2.z) + (deformationGradientF2X * inverseMatCoord3.z),
																		 (deformationGradientF3Y * inverseMatCoord2.z) + (deformationGradientF2Y * inverseMatCoord3.z),
																		 (deformationGradientF3Z * inverseMatCoord2.z) + (deformationGradientF2Z * inverseMatCoord3.z));

			vcsVector strainGradient23_1_RightofMinusPart1 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
																	   deformationGradientF2Y * inverseMatCoord2.x,
																	   deformationGradientF2Z * inverseMatCoord2.x);
			vcsVector strainGradient23_2_RightofMinusPart1 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
																	   deformationGradientF2Y * inverseMatCoord2.y,
																	   deformationGradientF2Z * inverseMatCoord2.y);
			vcsVector strainGradient23_3_RightofMinusPart1 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
																	   deformationGradientF2Y * inverseMatCoord2.z,
																	   deformationGradientF2Z * inverseMatCoord2.z);

			vcsVector strainGradient23_1_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
																	   deformationGradientF3Y * inverseMatCoord3.x,
																	   deformationGradientF3Z * inverseMatCoord3.x);
			vcsVector strainGradient23_2_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
																	   deformationGradientF3Y * inverseMatCoord3.y,
																	   deformationGradientF3Z * inverseMatCoord3.y);
			vcsVector strainGradient23_3_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
																	   deformationGradientF3Y * inverseMatCoord3.z,
																	   deformationGradientF3Z * inverseMatCoord3.z);

			vcsVector modifiedStrainGradient23_1_RightofMinus = (strainGradient23_1_RightofMinusPart1 * squareofDefGrad3) + (strainGradient23_1_RightofMinusPart2 * squareofDefGrad2);
			vcsVector modifiedStrainGradient23_2_RightofMinus = (strainGradient23_2_RightofMinusPart1 * squareofDefGrad3) + (strainGradient23_2_RightofMinusPart2 * squareofDefGrad2);
			vcsVector modifiedStrainGradient23_3_RightofMinus = (strainGradient23_3_RightofMinusPart1 * squareofDefGrad3) + (strainGradient23_3_RightofMinusPart2 * squareofDefGrad2);

			vcsVector modifiedStrainGradient23_1 = (modifiedStrainGradient23_1_LeftofMinus * oneOverDefGradLengths23) - (modifiedStrainGradient23_1_RightofMinus * strain23DividedbyCubes);
			vcsVector modifiedStrainGradient23_2 = (modifiedStrainGradient23_2_LeftofMinus * oneOverDefGradLengths23) - (modifiedStrainGradient23_2_RightofMinus * strain23DividedbyCubes);
			vcsVector modifiedStrainGradient23_3 = (modifiedStrainGradient23_3_LeftofMinus * oneOverDefGradLengths23) - (modifiedStrainGradient23_3_RightofMinus * strain23DividedbyCubes);

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec1 = strainGradient11_1;
			vcsVector strainGradient11Vec2 = strainGradient11_2;
			vcsVector strainGradient11Vec3 = strainGradient11_3;
			vcsVector strainGradient11Vec0 = (strainGradient11Vec1 * (-1.0)) + (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec1 = strainGradient22_1;
			vcsVector strainGradient22Vec2 = strainGradient22_2;
			vcsVector strainGradient22Vec3 = strainGradient22_3;
			vcsVector strainGradient22Vec0 = (strainGradient22Vec1 * (-1.0)) + (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			//strainGradientOverVertices 33 matrix
			vcsVector strainGradient33Vec1 = strainGradient33_1;
			vcsVector strainGradient33Vec2 = strainGradient33_2;
			vcsVector strainGradient33Vec3 = strainGradient33_3;
			vcsVector strainGradient33Vec0 = (strainGradient33Vec1 * (-1.0)) + (strainGradient33Vec2 * (-1.0)) + (strainGradient33Vec3 * (-1.0));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec1 = modifiedStrainGradient12_1;
			vcsVector strainGradient12Vec2 = modifiedStrainGradient12_2;
			vcsVector strainGradient12Vec3 = modifiedStrainGradient12_3;
			vcsVector strainGradient12Vec0 = (strainGradient12Vec1 * (-1.0)) + (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			//strainGradientOverVertices 13 matrix
			vcsVector strainGradient13Vec1 = modifiedStrainGradient13_1;
			vcsVector strainGradient13Vec2 = modifiedStrainGradient13_2;
			vcsVector strainGradient13Vec3 = modifiedStrainGradient13_3;
			vcsVector strainGradient13Vec0 = (strainGradient13Vec1 * (-1.0)) + (strainGradient13Vec2 * (-1.0)) + (strainGradient13Vec3 * (-1.0));

			//strainGradientOverVertices 23 matrix
			vcsVector strainGradient23Vec1 = modifiedStrainGradient23_1;
			vcsVector strainGradient23Vec2 = modifiedStrainGradient23_2;
			vcsVector strainGradient23Vec3 = modifiedStrainGradient23_3;
			vcsVector strainGradient23Vec0 = (strainGradient23Vec1 * (-1.0)) + (strainGradient23Vec2 * (-1.0)) + (strainGradient23Vec3 * (-1.0));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(strainGradient11Vec0.length(), 2) + pow(strainGradient11Vec1.length(), 2) + pow(strainGradient11Vec2.length(), 2) + pow(strainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(strainGradient22Vec0.length(), 2) + pow(strainGradient22Vec1.length(), 2) + pow(strainGradient22Vec2.length(), 2) + pow(strainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0) gradientLengthSum22 = 0.000001;

			//gradientLengthSum33
			float gradientLengthSum33 = pow(strainGradient33Vec0.length(), 2) + pow(strainGradient33Vec1.length(), 2) + pow(strainGradient33Vec2.length(), 2) + pow(strainGradient33Vec3.length(), 2);
			if (gradientLengthSum33 == 0) gradientLengthSum33 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(strainGradient12Vec0.length(), 2) + pow(strainGradient12Vec1.length(), 2) + pow(strainGradient12Vec2.length(), 2) + pow(strainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum13
			float gradientLengthSum13 = pow(strainGradient13Vec0.length(), 2) + pow(strainGradient13Vec1.length(), 2) + pow(strainGradient13Vec2.length(), 2) + pow(strainGradient13Vec3.length(), 2);
			if (gradientLengthSum13 == 0) gradientLengthSum13 = 0.000001;

			//gradientLengthSum23
			float gradientLengthSum23 = pow(strainGradient23Vec0.length(), 2) + pow(strainGradient23Vec1.length(), 2) + pow(strainGradient23Vec2.length(), 2) + pow(strainGradient23Vec3.length(), 2);
			if (gradientLengthSum23 == 0) gradientLengthSum23 = 0.000001;

			float oneOverVertexWeights = 0.25;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = (2 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = (2 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (strainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = (2 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (strainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = (2 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (strainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (strainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (strainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (strainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = (2 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = (2 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (strainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = (2 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (strainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = (2 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (strainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (strainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (strainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (strainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_33 = (2 * strainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]);
			//float deltaLagrangeMultiplierDenominator1_33 = gradientLengthSum33 + alphaHat;
			//deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator1_33 = (2 * strainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]) + (gama * (strainGradient33Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator2_33 = (2 * strainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_33[index]) + (gama * (strainGradient33Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier2_33 = -deltaLagrangeMultiplierNumerator2_33 / deltaLagrangeMultiplierDenominator2_33;

			float deltaLagrangeMultiplierNumerator3_33 = (2 * strainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_33[index]) + (gama * (strainGradient33Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier3_33 = -deltaLagrangeMultiplierNumerator3_33 / deltaLagrangeMultiplierDenominator3_33;

			dynamicSolverData.totalLagrangeMultiplierStrain1_33[index] += deltaLagrangeMultiplier1_33;
			dynamicSolverData.totalLagrangeMultiplierStrain2_33[index] += deltaLagrangeMultiplier2_33;
			dynamicSolverData.totalLagrangeMultiplierStrain3_33[index] += deltaLagrangeMultiplier3_33;

			//Compute new vector positions for strainTensor33 stretching
			vec1 = vec1 + (strainGradient33Vec1 * deltaLagrangeMultiplier1_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec2 = vec2 + (strainGradient33Vec2 * deltaLagrangeMultiplier2_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec3 = vec3 + (strainGradient33Vec3 * deltaLagrangeMultiplier3_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (strainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (strainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (strainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 shearing
			vec1 = vec1 + (strainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (strainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (strainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]);
			//float deltaLagrangeMultiplierDenominator1_13 = gradientLengthSum13 + alphaHat;
			//deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator1_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]) + (gama * (strainGradient13Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator2_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_13[index]) + (gama * (strainGradient13Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier2_13 = -deltaLagrangeMultiplierNumerator2_13 / deltaLagrangeMultiplierDenominator2_13;

			float deltaLagrangeMultiplierNumerator3_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_13[index]) + (gama * (strainGradient13Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier3_13 = -deltaLagrangeMultiplierNumerator3_13 / deltaLagrangeMultiplierDenominator3_13;

			dynamicSolverData.totalLagrangeMultiplierStrain1_13[index] += deltaLagrangeMultiplier1_13;
			dynamicSolverData.totalLagrangeMultiplierStrain2_13[index] += deltaLagrangeMultiplier2_13;
			dynamicSolverData.totalLagrangeMultiplierStrain3_13[index] += deltaLagrangeMultiplier3_13;

			//Compute new vector positions for strainTensor13 shearing
			vec1 = vec1 + (strainGradient13Vec1 * deltaLagrangeMultiplier1_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec2 = vec2 + (strainGradient13Vec2 * deltaLagrangeMultiplier2_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec3 = vec3 + (strainGradient13Vec3 * deltaLagrangeMultiplier3_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]);
			//float deltaLagrangeMultiplierDenominator1_23 = gradientLengthSum23 + alphaHat;
			//deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator1_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]) + (gama * (strainGradient23Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator2_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_23[index]) + (gama * (strainGradient23Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier2_23 = -deltaLagrangeMultiplierNumerator2_23 / deltaLagrangeMultiplierDenominator2_23;

			float deltaLagrangeMultiplierNumerator3_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_23[index]) + (gama * (strainGradient23Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier3_23 = -deltaLagrangeMultiplierNumerator3_23 / deltaLagrangeMultiplierDenominator3_23;

			dynamicSolverData.totalLagrangeMultiplierStrain1_23[index] += deltaLagrangeMultiplier1_23;
			dynamicSolverData.totalLagrangeMultiplierStrain2_23[index] += deltaLagrangeMultiplier2_23;
			dynamicSolverData.totalLagrangeMultiplierStrain3_23[index] += deltaLagrangeMultiplier3_23;

			//Compute new vector positions for strainTensor23 shearing
			vec1 = vec1 + (strainGradient23Vec1 * deltaLagrangeMultiplier1_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec2 = vec2 + (strainGradient23Vec2 * deltaLagrangeMultiplier2_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec3 = vec3 + (strainGradient23Vec3 * deltaLagrangeMultiplier3_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//areaStrainConstraintTri
void PHYSSolverCPU::areaStrainConstraintTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
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

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			vcsVector vecDifference21 = vec2 - vec1;
			vcsVector vecDifference31 = vec3 - vec1;

			//materialCoordinate
			vcsVector q1 = staticSolverData.pMaterialCoordinateC1List[index];
			vcsVector q2 = staticSolverData.pMaterialCoordinateC2List[index];

			float areaConstraint = pow((vecDifference21.cross(vecDifference31)).length(), 2) - pow((q1.cross(q2)).length(), 2);

			/* //restArea
			//float restArea = staticSolverData.pRestAreaList[index];
			if (index == 419)
				staticSolverData.relErrorValue = areaConstraint;
				//staticSolverData.relErrorValue = log10(abs(volumeConstraint - restVolume) / abs(restVolume)); */

			vcsVector areaGradientVec2 = vecDifference31.cross(vecDifference21.cross(vecDifference31)) * 2.0;
			vcsVector areaGradientVec3 = vecDifference21.cross(vecDifference31.cross(vecDifference21)) * 2.0;
			vcsVector areaGradientVec1 = (areaGradientVec2 * (-1.0)) + (areaGradientVec3 * (-1.0));

			float gradientLengthSum = pow(areaGradientVec1.length(), 2) + pow(areaGradientVec2.length(), 2) + pow(areaGradientVec3.length(), 2);

			float oneOverVertexWeights = 0.33;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1 = areaConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainArea1[index]);
			//float deltaLagrangeMultiplierDenominator1 = gradientLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1 = areaConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainArea1[index]) + (gama * (areaGradientVec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = areaConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainArea2[index]) + (gama * (areaGradientVec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			float deltaLagrangeMultiplierNumerator3 = areaConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainArea3[index]) + (gama * (areaGradientVec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier3 = -deltaLagrangeMultiplierNumerator3 / deltaLagrangeMultiplierDenominator3;

			dynamicSolverData.totalLagrangeMultiplierStrainArea1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierStrainArea2[index] += deltaLagrangeMultiplier2;
			dynamicSolverData.totalLagrangeMultiplierStrainArea3[index] += deltaLagrangeMultiplier3;

			vec1 = vec1 + (areaGradientVec1 * deltaLagrangeMultiplier1 * oneOverVertexWeights);
			vec2 = vec2 + (areaGradientVec2 * deltaLagrangeMultiplier2 * oneOverVertexWeights);
			vec3 = vec3 + (areaGradientVec3 * deltaLagrangeMultiplier3 * oneOverVertexWeights);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//volumeStrainConstraintTet
void PHYSSolverCPU::volumeStrainConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec0 = staticSolverData.pCenterofMass[0];
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vertex differences with center of mass for fake tet
			vcsVector diffVec1nVec0 = vec1 - vec0;
			vcsVector diffVec2nVec0 = vec2 - vec0;
			vcsVector diffVec3nVec0 = vec3 - vec0;

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			//materialCoordinate
			vcsVector q1 = staticSolverData.pVertexInitialpositionList[vertexIndex1] - staticSolverData.initialCenterofMass;
			vcsVector q2 = staticSolverData.pVertexInitialpositionList[vertexIndex2] - staticSolverData.initialCenterofMass;
			vcsVector q3 = staticSolverData.pVertexInitialpositionList[vertexIndex3] - staticSolverData.initialCenterofMass;

			vcsVector vec2Crossvec3 = diffVec2nVec0.cross(diffVec3nVec0);
			float detP = diffVec1nVec0.dot(vec2Crossvec3);

			vcsVector q2Crossq3 = q2.cross(q3);
			float detQ = q1.dot(q2Crossq3);

			float volumeConstraint = detP - detQ;

			/* //restVolume
			//float restVolume = staticSolverData.pRestVolumeList[index];
			if (index == 3378)
				staticSolverData.relErrorValue = volumeConstraint;
				//staticSolverData.relErrorValue = log10(abs(volumeConstraint - restVolume) / abs(restVolume)); */

			//Compute new vector positions for volume
			vcsVector volumeGradientVec1 = diffVec2nVec0.cross(diffVec3nVec0);
			vcsVector volumeGradientVec2 = diffVec3nVec0.cross(diffVec1nVec0);
			vcsVector volumeGradientVec3 = diffVec1nVec0.cross(diffVec2nVec0);
			vcsVector volumeGradientVec0 = (volumeGradientVec1 * (-1.0)) + (volumeGradientVec2 * (-1.0)) + (volumeGradientVec3 * (-1.0));

			float gradientLengthSum = pow(volumeGradientVec1.length(), 2) + pow(volumeGradientVec2.length(), 2) + pow(volumeGradientVec3.length(), 2) + pow(volumeGradientVec0.length(), 2);

			float oneOverVertexWeights = 0.25;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1 = volumeConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainVolume1[index]);
			//float deltaLagrangeMultiplierDenominator1 = gradientLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1 = volumeConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainVolume1[index]) + (gama * (volumeGradientVec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = volumeConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainVolume2[index]) + (gama * (volumeGradientVec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			float deltaLagrangeMultiplierNumerator3 = volumeConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrainVolume3[index]) + (gama * (volumeGradientVec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier3 = -deltaLagrangeMultiplierNumerator3 / deltaLagrangeMultiplierDenominator3;

			dynamicSolverData.totalLagrangeMultiplierStrainVolume1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierStrainVolume2[index] += deltaLagrangeMultiplier2;
			dynamicSolverData.totalLagrangeMultiplierStrainVolume3[index] += deltaLagrangeMultiplier3;

			vec1 = vec1 + (volumeGradientVec1 * deltaLagrangeMultiplier1 * oneOverVertexWeights);
			vec2 = vec2 + (volumeGradientVec2 * deltaLagrangeMultiplier2 * oneOverVertexWeights);
			vec3 = vec3 + (volumeGradientVec3 * deltaLagrangeMultiplier3 * oneOverVertexWeights);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//stVenantKirchhoffConstraintTet
void PHYSSolverCPU::stVenantKirchhoffConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//restVolume
			float restVolume = staticSolverData.pRestVolumeList[index];

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec0 = staticSolverData.pCenterofMass[0];
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vertex differences with center of mass for fake tet
			vcsVector diffVec1nVec0 = vec1 - vec0;
			vcsVector diffVec2nVec0 = vec2 - vec0;
			vcsVector diffVec3nVec0 = vec3 - vec0;

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			//materialCoordinate with initial center of mass differences for fake tet
			vcsVector materialCoordinateC1 = staticSolverData.pVertexInitialpositionList[vertexIndex1] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC2 = staticSolverData.pVertexInitialpositionList[vertexIndex2] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC3 = staticSolverData.pVertexInitialpositionList[vertexIndex3] - staticSolverData.initialCenterofMass;

			float determinantMatCoord = (materialCoordinateC1.x * materialCoordinateC2.y * materialCoordinateC3.z) +
										(materialCoordinateC2.x * materialCoordinateC3.y * materialCoordinateC1.z) +
										(materialCoordinateC3.x * materialCoordinateC1.y * materialCoordinateC2.z) -
										(materialCoordinateC3.x * materialCoordinateC2.y * materialCoordinateC1.z) -
										(materialCoordinateC1.x * materialCoordinateC3.y * materialCoordinateC2.z) -
										(materialCoordinateC2.x * materialCoordinateC1.y * materialCoordinateC3.z);
			if (determinantMatCoord == 0.0) determinantMatCoord = 0.000001;

			vcsVector inverseMatCoord1 = vcsVector(materialCoordinateC2.y * materialCoordinateC3.z - materialCoordinateC3.y * materialCoordinateC2.z,
												   materialCoordinateC3.y * materialCoordinateC1.z - materialCoordinateC1.y * materialCoordinateC3.z,
												   materialCoordinateC1.y * materialCoordinateC2.z - materialCoordinateC2.y * materialCoordinateC1.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord2 = vcsVector(materialCoordinateC3.x * materialCoordinateC2.z - materialCoordinateC2.x * materialCoordinateC3.z,
												   materialCoordinateC1.x * materialCoordinateC3.z - materialCoordinateC3.x * materialCoordinateC1.z,
												   materialCoordinateC2.x * materialCoordinateC1.z - materialCoordinateC1.x * materialCoordinateC2.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord3 = vcsVector(materialCoordinateC2.x * materialCoordinateC3.y - materialCoordinateC3.x * materialCoordinateC2.y,
												   materialCoordinateC3.x * materialCoordinateC1.y - materialCoordinateC1.x * materialCoordinateC3.y,
												   materialCoordinateC1.x * materialCoordinateC2.y - materialCoordinateC2.x * materialCoordinateC1.y) * (1.0 / determinantMatCoord);


			//deformationGradient matrix elements 3x3
			float deformationGradientF1X = (diffVec1nVec0.x * inverseMatCoord1.x) + (diffVec2nVec0.x * inverseMatCoord1.y) + (diffVec3nVec0.x * inverseMatCoord1.z);
			float deformationGradientF1Y = (diffVec1nVec0.y * inverseMatCoord1.x) + (diffVec2nVec0.y * inverseMatCoord1.y) + (diffVec3nVec0.y * inverseMatCoord1.z);
			float deformationGradientF1Z = (diffVec1nVec0.z * inverseMatCoord1.x) + (diffVec2nVec0.z * inverseMatCoord1.y) + (diffVec3nVec0.z * inverseMatCoord1.z);

			float deformationGradientF2X = (diffVec1nVec0.x * inverseMatCoord2.x) + (diffVec2nVec0.x * inverseMatCoord2.y) + (diffVec3nVec0.x * inverseMatCoord2.z);
			float deformationGradientF2Y = (diffVec1nVec0.y * inverseMatCoord2.x) + (diffVec2nVec0.y * inverseMatCoord2.y) + (diffVec3nVec0.y * inverseMatCoord2.z);
			float deformationGradientF2Z = (diffVec1nVec0.z * inverseMatCoord2.x) + (diffVec2nVec0.z * inverseMatCoord2.y) + (diffVec3nVec0.z * inverseMatCoord2.z);

			float deformationGradientF3X = (diffVec1nVec0.x * inverseMatCoord3.x) + (diffVec2nVec0.x * inverseMatCoord3.y) + (diffVec3nVec0.x * inverseMatCoord3.z);
			float deformationGradientF3Y = (diffVec1nVec0.y * inverseMatCoord3.x) + (diffVec2nVec0.y * inverseMatCoord3.y) + (diffVec3nVec0.y * inverseMatCoord3.z);
			float deformationGradientF3Z = (diffVec1nVec0.z * inverseMatCoord3.x) + (diffVec2nVec0.z * inverseMatCoord3.y) + (diffVec3nVec0.z * inverseMatCoord3.z);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);
			vcsVector deformationGradientVector3 = vcsVector(deformationGradientF3X, deformationGradientF3Y, deformationGradientF3Z);

			//strainTensor matrix elements 3x3
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);
			float strainTensorS1Z = deformationGradientVector3.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);
			float strainTensorS2Z = deformationGradientVector3.dot(deformationGradientVector2);

			float strainTensorS3X = deformationGradientVector1.dot(deformationGradientVector3);
			float strainTensorS3Y = deformationGradientVector2.dot(deformationGradientVector3);
			float strainTensorS3Z = deformationGradientVector3.dot(deformationGradientVector3);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, strainTensorS1Z);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, strainTensorS2Z);
			vcsVector strainTensor3 = vcsVector(strainTensorS3X, strainTensorS3Y, strainTensorS3Z);

			float strainConstraint11 = 0.5 * (strainTensor1.x - 1.0);
			float strainConstraint22 = 0.5 * (strainTensor2.y - 1.0);
			float strainConstraint33 = 0.5 * (strainTensor3.z - 1.0);
			float strainConstraint12 = 0.5 * strainTensor2.x;
			float strainConstraint13 = 0.5 * strainTensor3.x;
			float strainConstraint23 = 0.5 * strainTensor3.y;
			float strainConstraint21 = 0.5 * strainTensor1.y;
			float strainConstraint31 = 0.5 * strainTensor1.z;
			float strainConstraint32 = 0.5 * strainTensor2.z;

			float shearModulus = dynamicSolverData.youngsModulus / (2 * (1 + dynamicSolverData.poissonRatio));  //mu
			float lameModulus = (dynamicSolverData.youngsModulus * dynamicSolverData.poissonRatio) / ((1 + dynamicSolverData.poissonRatio) * (1 - (2 * dynamicSolverData.poissonRatio))); //lambda
			
			float trace = strainConstraint11 + strainConstraint22 + strainConstraint33;
			float productTrace = (strainConstraint11 * strainConstraint11) + (strainConstraint12 * strainConstraint12) + (strainConstraint13 * strainConstraint13) +
								 (strainConstraint21 * strainConstraint21) + (strainConstraint22 * strainConstraint22) + (strainConstraint23 * strainConstraint23) +
								 (strainConstraint31 * strainConstraint31) + (strainConstraint32 * strainConstraint32) + (strainConstraint33 * strainConstraint33);

			float strainEnergyDensity = (shearModulus * productTrace) + (0.5 * lameModulus * trace * trace);
			float strainEnergyConstraint = restVolume * strainEnergyDensity;

			//if (index == 3378)
				//staticSolverData.relErrorValue = log10(abs((strainEnergyConstraint / dynamicSolverData.youngsModulus)) / abs(restVolume));
			
			//first piola-kirchhoff stress tensor for gradient
			float leftCoef = 2 * shearModulus;
			float rightCoef = lameModulus * trace;
			float prePiolaKirchhoff11 = (leftCoef * strainConstraint11) + rightCoef;
			float prePiolaKirchhoff22 = (leftCoef * strainConstraint22) + rightCoef;
			float prePiolaKirchhoff33 = (leftCoef * strainConstraint33) + rightCoef;
			float prePiolaKirchhoff12 = leftCoef * strainConstraint12;
			float prePiolaKirchhoff13 = leftCoef * strainConstraint13;
			float prePiolaKirchhoff23 = leftCoef * strainConstraint23;
			float prePiolaKirchhoff21 = leftCoef * strainConstraint21;
			float prePiolaKirchhoff31 = leftCoef * strainConstraint31;
			float prePiolaKirchhoff32 = leftCoef * strainConstraint32;

			float piolaKirchhoff1X = (deformationGradientF1X * prePiolaKirchhoff11) + (deformationGradientF2X * prePiolaKirchhoff21) + (deformationGradientF3X * prePiolaKirchhoff31);
			float piolaKirchhoff1Y = (deformationGradientF1Y * prePiolaKirchhoff11) + (deformationGradientF2Y * prePiolaKirchhoff21) + (deformationGradientF3Y * prePiolaKirchhoff31);
			float piolaKirchhoff1Z = (deformationGradientF1Z * prePiolaKirchhoff11) + (deformationGradientF2Z * prePiolaKirchhoff21) + (deformationGradientF3Z * prePiolaKirchhoff31);

			float piolaKirchhoff2X = (deformationGradientF1X * prePiolaKirchhoff12) + (deformationGradientF2X * prePiolaKirchhoff22) + (deformationGradientF3X * prePiolaKirchhoff32);
			float piolaKirchhoff2Y = (deformationGradientF1Y * prePiolaKirchhoff12) + (deformationGradientF2Y * prePiolaKirchhoff22) + (deformationGradientF3Y * prePiolaKirchhoff32);
			float piolaKirchhoff2Z = (deformationGradientF1Z * prePiolaKirchhoff12) + (deformationGradientF2Z * prePiolaKirchhoff22) + (deformationGradientF3Z * prePiolaKirchhoff32);

			float piolaKirchhoff3X = (deformationGradientF1X * prePiolaKirchhoff13) + (deformationGradientF2X * prePiolaKirchhoff23) + (deformationGradientF3X * prePiolaKirchhoff33);
			float piolaKirchhoff3Y = (deformationGradientF1Y * prePiolaKirchhoff13) + (deformationGradientF2Y * prePiolaKirchhoff23) + (deformationGradientF3Y * prePiolaKirchhoff33);
			float piolaKirchhoff3Z = (deformationGradientF1Z * prePiolaKirchhoff13) + (deformationGradientF2Z * prePiolaKirchhoff23) + (deformationGradientF3Z * prePiolaKirchhoff33);

			float stainEnergyGradient1X = (piolaKirchhoff1X * inverseMatCoord1.x) + (piolaKirchhoff2X * inverseMatCoord2.x) + (piolaKirchhoff3X * inverseMatCoord3.x);
			float stainEnergyGradient1Y = (piolaKirchhoff1Y * inverseMatCoord1.x) + (piolaKirchhoff2Y * inverseMatCoord2.x) + (piolaKirchhoff3Y * inverseMatCoord3.x);
			float stainEnergyGradient1Z = (piolaKirchhoff1Z * inverseMatCoord1.x) + (piolaKirchhoff2Z * inverseMatCoord2.x) + (piolaKirchhoff3Z * inverseMatCoord3.x);

			float stainEnergyGradient2X = (piolaKirchhoff1X * inverseMatCoord1.y) + (piolaKirchhoff2X * inverseMatCoord2.y) + (piolaKirchhoff3X * inverseMatCoord3.y);
			float stainEnergyGradient2Y = (piolaKirchhoff1Y * inverseMatCoord1.y) + (piolaKirchhoff2Y * inverseMatCoord2.y) + (piolaKirchhoff3Y * inverseMatCoord3.y);
			float stainEnergyGradient2Z = (piolaKirchhoff1Z * inverseMatCoord1.y) + (piolaKirchhoff2Z * inverseMatCoord2.y) + (piolaKirchhoff3Z * inverseMatCoord3.y);

			float stainEnergyGradient3X = (piolaKirchhoff1X * inverseMatCoord1.z) + (piolaKirchhoff2X * inverseMatCoord2.z) + (piolaKirchhoff3X * inverseMatCoord3.z);
			float stainEnergyGradient3Y = (piolaKirchhoff1Y * inverseMatCoord1.z) + (piolaKirchhoff2Y * inverseMatCoord2.z) + (piolaKirchhoff3Y * inverseMatCoord3.z);
			float stainEnergyGradient3Z = (piolaKirchhoff1Z * inverseMatCoord1.z) + (piolaKirchhoff2Z * inverseMatCoord2.z) + (piolaKirchhoff3Z * inverseMatCoord3.z);

			//gradient vectors
			vcsVector stainEnergyGradientVec1 = vcsVector(stainEnergyGradient1X, stainEnergyGradient1Y, stainEnergyGradient1Z) * restVolume;
			vcsVector stainEnergyGradientVec2 = vcsVector(stainEnergyGradient2X, stainEnergyGradient2Y, stainEnergyGradient2Z) * restVolume;
			vcsVector stainEnergyGradientVec3 = vcsVector(stainEnergyGradient3X, stainEnergyGradient3Y, stainEnergyGradient3Z) * restVolume;
			vcsVector stainEnergyGradientVec0 = (stainEnergyGradientVec1 * (-1.0)) + (stainEnergyGradientVec2 * (-1.0)) + (stainEnergyGradientVec3 * (-1.0));

			//gradientLengthSum
			float gradientLengthSum = pow(stainEnergyGradientVec0.length(), 2) + pow(stainEnergyGradientVec1.length(), 2) + pow(stainEnergyGradientVec2.length(), 2) + pow(stainEnergyGradientVec3.length(), 2);
			if (gradientLengthSum == 0) gradientLengthSum = 0.000001;			

			float oneOverVertexWeights = 0.25;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVK1[index]);
			//float deltaLagrangeMultiplierDenominator1 = gradientLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVK1[index]) + (gama * (stainEnergyGradientVec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVK2[index]) + (gama * (stainEnergyGradientVec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			float deltaLagrangeMultiplierNumerator3 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVK3[index]) + (gama * (stainEnergyGradientVec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier3 = -deltaLagrangeMultiplierNumerator3 / deltaLagrangeMultiplierDenominator3;

			dynamicSolverData.totalLagrangeMultiplierSTVK1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierSTVK2[index] += deltaLagrangeMultiplier2;
			dynamicSolverData.totalLagrangeMultiplierSTVK3[index] += deltaLagrangeMultiplier3;
			
			//Compute new vector positions for stVenantKirchhoffConstraint
			vec1 = vec1 + (stainEnergyGradientVec1 * deltaLagrangeMultiplier1 * oneOverVertexWeights);
			vec2 = vec2 + (stainEnergyGradientVec2 * deltaLagrangeMultiplier2 * oneOverVertexWeights);
			vec3 = vec3 + (stainEnergyGradientVec3 * deltaLagrangeMultiplier3 * oneOverVertexWeights);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//neoHookeanConstraintTet
void PHYSSolverCPU::neoHookeanConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//restVolume
			float restVolume = staticSolverData.pRestVolumeList[index];

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec0 = staticSolverData.pCenterofMass[0];
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vertex differences with center of mass for fake tet
			vcsVector diffVec1nVec0 = vec1 - vec0;
			vcsVector diffVec2nVec0 = vec2 - vec0;
			vcsVector diffVec3nVec0 = vec3 - vec0;

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			//materialCoordinate with initial center of mass differences for fake tet
			vcsVector materialCoordinateC1 = staticSolverData.pVertexInitialpositionList[vertexIndex1] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC2 = staticSolverData.pVertexInitialpositionList[vertexIndex2] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC3 = staticSolverData.pVertexInitialpositionList[vertexIndex3] - staticSolverData.initialCenterofMass;

			float determinantMatCoord = (materialCoordinateC1.x * materialCoordinateC2.y * materialCoordinateC3.z) +
										(materialCoordinateC2.x * materialCoordinateC3.y * materialCoordinateC1.z) +
										(materialCoordinateC3.x * materialCoordinateC1.y * materialCoordinateC2.z) -
										(materialCoordinateC3.x * materialCoordinateC2.y * materialCoordinateC1.z) -
										(materialCoordinateC1.x * materialCoordinateC3.y * materialCoordinateC2.z) -
										(materialCoordinateC2.x * materialCoordinateC1.y * materialCoordinateC3.z);
			if (determinantMatCoord == 0.0) determinantMatCoord = 0.000001;

			vcsVector inverseMatCoord1 = vcsVector(materialCoordinateC2.y * materialCoordinateC3.z - materialCoordinateC3.y * materialCoordinateC2.z,
												   materialCoordinateC3.y * materialCoordinateC1.z - materialCoordinateC1.y * materialCoordinateC3.z,
												   materialCoordinateC1.y * materialCoordinateC2.z - materialCoordinateC2.y * materialCoordinateC1.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord2 = vcsVector(materialCoordinateC3.x * materialCoordinateC2.z - materialCoordinateC2.x * materialCoordinateC3.z,
												   materialCoordinateC1.x * materialCoordinateC3.z - materialCoordinateC3.x * materialCoordinateC1.z,
												   materialCoordinateC2.x * materialCoordinateC1.z - materialCoordinateC1.x * materialCoordinateC2.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord3 = vcsVector(materialCoordinateC2.x * materialCoordinateC3.y - materialCoordinateC3.x * materialCoordinateC2.y,
												   materialCoordinateC3.x * materialCoordinateC1.y - materialCoordinateC1.x * materialCoordinateC3.y,
												   materialCoordinateC1.x * materialCoordinateC2.y - materialCoordinateC2.x * materialCoordinateC1.y) * (1.0 / determinantMatCoord);
			
			//deformationGradient matrix elements 3x3
			float deformationGradientF1X = (diffVec1nVec0.x * inverseMatCoord1.x) + (diffVec2nVec0.x * inverseMatCoord1.y) + (diffVec3nVec0.x * inverseMatCoord1.z);
			float deformationGradientF1Y = (diffVec1nVec0.y * inverseMatCoord1.x) + (diffVec2nVec0.y * inverseMatCoord1.y) + (diffVec3nVec0.y * inverseMatCoord1.z);
			float deformationGradientF1Z = (diffVec1nVec0.z * inverseMatCoord1.x) + (diffVec2nVec0.z * inverseMatCoord1.y) + (diffVec3nVec0.z * inverseMatCoord1.z);

			float deformationGradientF2X = (diffVec1nVec0.x * inverseMatCoord2.x) + (diffVec2nVec0.x * inverseMatCoord2.y) + (diffVec3nVec0.x * inverseMatCoord2.z);
			float deformationGradientF2Y = (diffVec1nVec0.y * inverseMatCoord2.x) + (diffVec2nVec0.y * inverseMatCoord2.y) + (diffVec3nVec0.y * inverseMatCoord2.z);
			float deformationGradientF2Z = (diffVec1nVec0.z * inverseMatCoord2.x) + (diffVec2nVec0.z * inverseMatCoord2.y) + (diffVec3nVec0.z * inverseMatCoord2.z);

			float deformationGradientF3X = (diffVec1nVec0.x * inverseMatCoord3.x) + (diffVec2nVec0.x * inverseMatCoord3.y) + (diffVec3nVec0.x * inverseMatCoord3.z);
			float deformationGradientF3Y = (diffVec1nVec0.y * inverseMatCoord3.x) + (diffVec2nVec0.y * inverseMatCoord3.y) + (diffVec3nVec0.y * inverseMatCoord3.z);
			float deformationGradientF3Z = (diffVec1nVec0.z * inverseMatCoord3.x) + (diffVec2nVec0.z * inverseMatCoord3.y) + (diffVec3nVec0.z * inverseMatCoord3.z);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);
			vcsVector deformationGradientVector3 = vcsVector(deformationGradientF3X, deformationGradientF3Y, deformationGradientF3Z);

			//strainTensor matrix elements 3x3
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);
			float strainTensorS1Z = deformationGradientVector3.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);
			float strainTensorS2Z = deformationGradientVector3.dot(deformationGradientVector2);

			float strainTensorS3X = deformationGradientVector1.dot(deformationGradientVector3);
			float strainTensorS3Y = deformationGradientVector2.dot(deformationGradientVector3);
			float strainTensorS3Z = deformationGradientVector3.dot(deformationGradientVector3);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, strainTensorS1Z);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, strainTensorS2Z);
			vcsVector strainTensor3 = vcsVector(strainTensorS3X, strainTensorS3Y, strainTensorS3Z);

			float shearModulus = dynamicSolverData.youngsModulus / (2 * (1 + dynamicSolverData.poissonRatio));  //mu
			float lameModulus = (dynamicSolverData.youngsModulus * dynamicSolverData.poissonRatio) / ((1 + dynamicSolverData.poissonRatio) * (1 - (2 * dynamicSolverData.poissonRatio))); //lambda
			
			float traceStrainTensor = strainTensor1.x + strainTensor2.y + strainTensor3.z;

			float determinantStrainTensor = (strainTensor1.x * strainTensor2.y * strainTensor3.z) +
											(strainTensor2.x * strainTensor3.y * strainTensor1.z) +
											(strainTensor3.x * strainTensor1.y * strainTensor2.z) -
											(strainTensor3.x * strainTensor2.y * strainTensor1.z) -
											(strainTensor1.x * strainTensor3.y * strainTensor2.z) -
											(strainTensor2.x * strainTensor1.y * strainTensor3.z);
			if (determinantStrainTensor <= 0.0) determinantStrainTensor = 0.000001;
			if (determinantStrainTensor >= 1000000.0) determinantStrainTensor = 1000000.0;

			float strainEnergyDensity = (0.5 * shearModulus * (traceStrainTensor - log(determinantStrainTensor) - 3.0)) + (0.125 * lameModulus * log(determinantStrainTensor) * log(determinantStrainTensor));
			float strainEnergyConstraint = restVolume * strainEnergyDensity;

			//if (index == 3378)
				//staticSolverData.relErrorValue = log10(abs((strainEnergyConstraint / dynamicSolverData.youngsModulus)) / abs(restVolume));

			//inverse of deformationGradient matrix
			float determinantDefGrdnt = (deformationGradientVector1.x * deformationGradientVector2.y * deformationGradientVector3.z) +
										(deformationGradientVector2.x * deformationGradientVector3.y * deformationGradientVector1.z) +
										(deformationGradientVector3.x * deformationGradientVector1.y * deformationGradientVector2.z) -
										(deformationGradientVector3.x * deformationGradientVector2.y * deformationGradientVector1.z) -
										(deformationGradientVector1.x * deformationGradientVector3.y * deformationGradientVector2.z) -
										(deformationGradientVector2.x * deformationGradientVector1.y * deformationGradientVector3.z);
			if (determinantDefGrdnt == 0.0) determinantDefGrdnt = 0.000001;

			vcsVector inverseDefGrdnt1 = vcsVector(deformationGradientVector2.y * deformationGradientVector3.z - deformationGradientVector3.y * deformationGradientVector2.z,
												   deformationGradientVector3.y * deformationGradientVector1.z - deformationGradientVector1.y * deformationGradientVector3.z,
												   deformationGradientVector1.y * deformationGradientVector2.z - deformationGradientVector2.y * deformationGradientVector1.z) * (1.0 / determinantDefGrdnt);

			vcsVector inverseDefGrdnt2 = vcsVector(deformationGradientVector3.x * deformationGradientVector2.z - deformationGradientVector2.x * deformationGradientVector3.z,
												   deformationGradientVector1.x * deformationGradientVector3.z - deformationGradientVector3.x * deformationGradientVector1.z,
												   deformationGradientVector2.x * deformationGradientVector1.z - deformationGradientVector1.x * deformationGradientVector2.z) * (1.0 / determinantDefGrdnt);

			vcsVector inverseDefGrdnt3 = vcsVector(deformationGradientVector2.x * deformationGradientVector3.y - deformationGradientVector3.x * deformationGradientVector2.y,
												   deformationGradientVector3.x * deformationGradientVector1.y - deformationGradientVector1.x * deformationGradientVector3.y,
												   deformationGradientVector1.x * deformationGradientVector2.y - deformationGradientVector2.x * deformationGradientVector1.y) * (1.0 / determinantDefGrdnt);

			//transpose of inverse of deformationGradient matrix
			vcsVector tpInverseDefGrdnt1 = vcsVector(inverseDefGrdnt1.x, inverseDefGrdnt2.x, inverseDefGrdnt3.x);
			vcsVector tpInverseDefGrdnt2 = vcsVector(inverseDefGrdnt1.y, inverseDefGrdnt2.y, inverseDefGrdnt3.y);
			vcsVector tpInverseDefGrdnt3 = vcsVector(inverseDefGrdnt1.z, inverseDefGrdnt2.z, inverseDefGrdnt3.z);

			float logCoef = 0.5 * lameModulus * log(determinantStrainTensor);
						
			//first piola-kirchhoff stress tensor for gradient
			vcsVector piolaKirchhoffVec1 = (deformationGradientVector1 * shearModulus) - (tpInverseDefGrdnt1 * shearModulus) + (tpInverseDefGrdnt1 * logCoef);
			vcsVector piolaKirchhoffVec2 = (deformationGradientVector2 * shearModulus) - (tpInverseDefGrdnt2 * shearModulus) + (tpInverseDefGrdnt2 * logCoef);
			vcsVector piolaKirchhoffVec3 = (deformationGradientVector3 * shearModulus) - (tpInverseDefGrdnt3 * shearModulus) + (tpInverseDefGrdnt3 * logCoef);

			float stainEnergyGradient1X = (piolaKirchhoffVec1.x * inverseMatCoord1.x) + (piolaKirchhoffVec2.x * inverseMatCoord2.x) + (piolaKirchhoffVec3.x * inverseMatCoord3.x);
			float stainEnergyGradient1Y = (piolaKirchhoffVec1.y * inverseMatCoord1.x) + (piolaKirchhoffVec2.y * inverseMatCoord2.x) + (piolaKirchhoffVec3.y * inverseMatCoord3.x);
			float stainEnergyGradient1Z = (piolaKirchhoffVec1.z * inverseMatCoord1.x) + (piolaKirchhoffVec2.z * inverseMatCoord2.x) + (piolaKirchhoffVec3.z * inverseMatCoord3.x);

			float stainEnergyGradient2X = (piolaKirchhoffVec1.x * inverseMatCoord1.y) + (piolaKirchhoffVec2.x * inverseMatCoord2.y) + (piolaKirchhoffVec3.x * inverseMatCoord3.y);
			float stainEnergyGradient2Y = (piolaKirchhoffVec1.y * inverseMatCoord1.y) + (piolaKirchhoffVec2.y * inverseMatCoord2.y) + (piolaKirchhoffVec3.y * inverseMatCoord3.y);
			float stainEnergyGradient2Z = (piolaKirchhoffVec1.z * inverseMatCoord1.y) + (piolaKirchhoffVec2.z * inverseMatCoord2.y) + (piolaKirchhoffVec3.z * inverseMatCoord3.y);

			float stainEnergyGradient3X = (piolaKirchhoffVec1.x * inverseMatCoord1.z) + (piolaKirchhoffVec2.x * inverseMatCoord2.z) + (piolaKirchhoffVec3.x * inverseMatCoord3.z);
			float stainEnergyGradient3Y = (piolaKirchhoffVec1.y * inverseMatCoord1.z) + (piolaKirchhoffVec2.y * inverseMatCoord2.z) + (piolaKirchhoffVec3.y * inverseMatCoord3.z);
			float stainEnergyGradient3Z = (piolaKirchhoffVec1.z * inverseMatCoord1.z) + (piolaKirchhoffVec2.z * inverseMatCoord2.z) + (piolaKirchhoffVec3.z * inverseMatCoord3.z);

			//gradient vectors
			vcsVector stainEnergyGradientVec1 = vcsVector(stainEnergyGradient1X, stainEnergyGradient1Y, stainEnergyGradient1Z) * restVolume;
			vcsVector stainEnergyGradientVec2 = vcsVector(stainEnergyGradient2X, stainEnergyGradient2Y, stainEnergyGradient2Z) * restVolume;
			vcsVector stainEnergyGradientVec3 = vcsVector(stainEnergyGradient3X, stainEnergyGradient3Y, stainEnergyGradient3Z) * restVolume;
			vcsVector stainEnergyGradientVec0 = (stainEnergyGradientVec1 * (-1.0)) + (stainEnergyGradientVec2 * (-1.0)) + (stainEnergyGradientVec3 * (-1.0));

			//gradientLengthSum
			float gradientLengthSum = pow(stainEnergyGradientVec0.length(), 2) + pow(stainEnergyGradientVec1.length(), 2) + pow(stainEnergyGradientVec2.length(), 2) + pow(stainEnergyGradientVec3.length(), 2);
			if (gradientLengthSum == 0) gradientLengthSum = 0.000001;

			float oneOverVertexWeights = 0.25;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierNeoH1[index]);
			//float deltaLagrangeMultiplierDenominator1 = gradientLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierNeoH1[index]) + (gama * (stainEnergyGradientVec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierNeoH2[index]) + (gama * (stainEnergyGradientVec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			float deltaLagrangeMultiplierNumerator3 = strainEnergyConstraint + (alphaHat * dynamicSolverData.totalLagrangeMultiplierNeoH3[index]) + (gama * (stainEnergyGradientVec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3 = ((1.0 + gama) * gradientLengthSum) + alphaHat;
			float deltaLagrangeMultiplier3 = -deltaLagrangeMultiplierNumerator3 / deltaLagrangeMultiplierDenominator3;

			dynamicSolverData.totalLagrangeMultiplierNeoH1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierNeoH2[index] += deltaLagrangeMultiplier2;
			dynamicSolverData.totalLagrangeMultiplierNeoH3[index] += deltaLagrangeMultiplier3;
			
			//Compute new vector positions for neoHookeanConstraint
			vec1 = vec1 + (stainEnergyGradientVec1 * deltaLagrangeMultiplier1 * oneOverVertexWeights);
			vec2 = vec2 + (stainEnergyGradientVec2 * deltaLagrangeMultiplier2 * oneOverVertexWeights);
			vec3 = vec3 + (stainEnergyGradientVec3 * deltaLagrangeMultiplier3 * oneOverVertexWeights);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//hookeSpringConstraint
void PHYSSolverCPU::hookeSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate edges
		for (int index = 0; index < staticSolverData.edgeCount; index++)
		{
			//restLength
			float restLength = staticSolverData.pEdgeRestlengthList[index];

			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;
						
			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pEdgeVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pEdgeVertexIndexList[index].y);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			
			//Compute new vector positions for stretching
			vcsVector delta = vec2 - vec1;
			float deltaLength = delta.length();
			if (deltaLength == 0.0) deltaLength = 0.000001;
			float potentialEnergy = 0.5 * kStiffness * (deltaLength - restLength) * (deltaLength - restLength);

			/*if (index == 472)
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += potentialEnergy;
				staticSolverData.potEneg = potentialEnergy;
			}*/
			
			//gradients
			vcsVector gradientConstraint1 = delta * (-1.0) * (1 - (restLength / deltaLength)) * kStiffness;
			vcsVector gradientConstraint2 = delta * (1 - (restLength / deltaLength)) * kStiffness;
			float constraintLengthSum = pow(gradientConstraint1.length(), 2) + pow(gradientConstraint2.length(), 2);
			if (constraintLengthSum == 0.0) constraintLengthSum = 0.000001;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierHookeSpring1[index]);
			//float deltaLagrangeMultiplierDenominator = constraintLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator / deltaLagrangeMultiplierDenominator;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;

			float deltaLagrangeMultiplierNumerator1 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierHookeSpring1[index]) + (gama * (gradientConstraint1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierHookeSpring2[index]) + (gama * (gradientConstraint2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			dynamicSolverData.totalLagrangeMultiplierHookeSpring1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierHookeSpring2[index] += deltaLagrangeMultiplier2;

			vec1 = vec1 + (gradientConstraint1 * 0.5 * deltaLagrangeMultiplier1);
			vec2 = vec2 + (gradientConstraint2 * 0.5 * deltaLagrangeMultiplier2);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
		}
	}
}


//stvkSpringConstraint
void PHYSSolverCPU::stvkSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate edges
		for (int index = 0; index < staticSolverData.edgeCount; index++)
		{
			//restLength
			float restLength = staticSolverData.pEdgeRestlengthList[index];

			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pEdgeVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pEdgeVertexIndexList[index].y);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];

			//Compute new vector positions for stretching
			vcsVector delta = vec2 - vec1;
			float deltaLength = delta.length();
			if (deltaLength == 0.0) deltaLength = 0.000001;
			float potentialEnergy = 0.5 * kStiffness * ((deltaLength * deltaLength) - (restLength * restLength)) * ((deltaLength * deltaLength) - (restLength * restLength));

			/*if (index == 472) //472   1220
			{
				staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += potentialEnergy;
				//staticSolverData.potEneg = potentialEnergy;
			}*/

			//gradients
			vcsVector gradientConstraint1 = delta * (-1.0) * ((deltaLength * deltaLength) - (restLength * restLength)) * 2.0 * kStiffness;
			vcsVector gradientConstraint2 = delta * ((deltaLength * deltaLength) - (restLength * restLength)) * 2.0 * kStiffness;
			float constraintLengthSum = pow(gradientConstraint1.length(), 2) + pow(gradientConstraint2.length(), 2);
			if (constraintLengthSum == 0.0) constraintLengthSum = 0.000001;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVKSpring1[index]);
			//float deltaLagrangeMultiplierDenominator = constraintLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator / deltaLagrangeMultiplierDenominator;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;

			float deltaLagrangeMultiplierNumerator1 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVKSpring1[index]) + (gama * (gradientConstraint1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVKSpring2[index]) + (gama * (gradientConstraint2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			dynamicSolverData.totalLagrangeMultiplierSTVKSpring1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierSTVKSpring2[index] += deltaLagrangeMultiplier2;

			vec1 = vec1 + (gradientConstraint1 * 0.5 * deltaLagrangeMultiplier1);
			vec2 = vec2 + (gradientConstraint2 * 0.5 * deltaLagrangeMultiplier2);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
		}
	}
}


//morsePotentialConstraint
void PHYSSolverCPU::morsePotentialConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate edges
		for (int index = 0; index < staticSolverData.edgeCount; index++)
		{
			//restLength
			float restLength = staticSolverData.pEdgeRestlengthList[index];

			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pEdgeVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pEdgeVertexIndexList[index].y);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];

			//Compute new vector positions for stretching
			vcsVector delta = vec2 - vec1;
			float deltaLength = delta.length();
			if (deltaLength == 0.0) deltaLength = 0.000001;
			float potentialEnergy = pow((1.0 - exp(deltaLength - restLength)), 2) * kStiffness;

			/*if (index == 472)
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += potentialEnergy;
				staticSolverData.potEneg = potentialEnergy;
			}*/
			
			//gradients
			vcsVector gradientConstraint1 = delta * (-1.0) * ((exp(deltaLength - restLength) * (exp(deltaLength - restLength) - 1.0)) / deltaLength) * 2.0 * kStiffness;
			vcsVector gradientConstraint2 = delta * ((exp(deltaLength - restLength) * (exp(deltaLength - restLength) - 1.0)) / deltaLength) * 2.0 * kStiffness;
			float constraintLengthSum = pow(gradientConstraint1.length(), 2) + pow(gradientConstraint2.length(), 2);
			if (constraintLengthSum == 0.0) constraintLengthSum = 0.000001;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierMorsePot1[index]);
			//float deltaLagrangeMultiplierDenominator = constraintLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator / deltaLagrangeMultiplierDenominator;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;

			float deltaLagrangeMultiplierNumerator1 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierMorsePot1[index]) + (gama * (gradientConstraint1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierMorsePot2[index]) + (gama * (gradientConstraint2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			dynamicSolverData.totalLagrangeMultiplierMorsePot1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierMorsePot2[index] += deltaLagrangeMultiplier2;

			vec1 = vec1 + (gradientConstraint1 * 0.5 * deltaLagrangeMultiplier1);
			vec2 = vec2 + (gradientConstraint2 * 0.5 * deltaLagrangeMultiplier2);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
		}
	}
}


//exponentialHookeSpringConstraint
void PHYSSolverCPU::exponentialHookeSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate edges
		for (int index = 0; index < staticSolverData.edgeCount; index++)
		{
			//restLength
			float restLength = staticSolverData.pEdgeRestlengthList[index];

			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;
						
			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pEdgeVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pEdgeVertexIndexList[index].y);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			
			//Compute new vector positions for stretching
			vcsVector delta = vec2 - vec1;
			float deltaLength = delta.length();
			if (deltaLength == 0.0) deltaLength = 0.000001;
			float hookeanEnergy = 0.5 * (deltaLength - restLength) * (deltaLength - restLength);
			float potentialEnergy = pow((1.0 - exp(hookeanEnergy)), 2) * kStiffness;
			
			/*if (index == 472) //472   1220
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += hookeanEnergy;
				staticSolverData.potEneg = hookeanEnergy;
			}*/
			
			//gradients
			vcsVector hookeanGradientConstraint1 = delta * (-1.0) * (1 - (restLength / deltaLength));
			vcsVector gradientConstraint1 = hookeanGradientConstraint1 * (2 * kStiffness * (1 - exp(hookeanEnergy))) * (-exp(hookeanEnergy));
			vcsVector hookeanGradientConstraint2 = delta * (1 - (restLength / deltaLength));
			vcsVector gradientConstraint2 = hookeanGradientConstraint2 * (2 * kStiffness * (1 - exp(hookeanEnergy))) * (-exp(hookeanEnergy));
			float constraintLengthSum = pow(gradientConstraint1.length(), 2) + pow(gradientConstraint2.length(), 2);
			if (constraintLengthSum == 0.0) constraintLengthSum = 0.000001;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierHookeSpring1[index]);
			//float deltaLagrangeMultiplierDenominator = constraintLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator / deltaLagrangeMultiplierDenominator;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;

			float deltaLagrangeMultiplierNumerator1 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring1[index]) + (gama * (gradientConstraint1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring2[index]) + (gama * (gradientConstraint2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierExponentialHookeSpring2[index] += deltaLagrangeMultiplier2;

			vec1 = vec1 + (gradientConstraint1 * 0.5 * deltaLagrangeMultiplier1);
			vec2 = vec2 + (gradientConstraint2 * 0.5 * deltaLagrangeMultiplier2);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
		}
	}
}


//exponentialStvkSpringConstraint
void PHYSSolverCPU::exponentialStvkSpringConstraint(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate edges
		for (int index = 0; index < staticSolverData.edgeCount; index++)
		{
			//restLength
			float restLength = staticSolverData.pEdgeRestlengthList[index];

			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pEdgeVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pEdgeVertexIndexList[index].y);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];

			//Compute new vector positions for stretching
			vcsVector delta = vec2 - vec1;
			float deltaLength = delta.length();
			if (deltaLength == 0.0) deltaLength = 0.000001;
			float stvkPotentialEnergy = 0.5 * ((deltaLength * deltaLength) - (restLength * restLength)) * ((deltaLength * deltaLength) - (restLength * restLength));
			float potentialEnergy = pow((1.0 - exp(stvkPotentialEnergy)), 2) * kStiffness;
			
			/*if (index == 472) //472  1220
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += potentialEnergy;
				staticSolverData.potEneg = potentialEnergy;
			}*/

			//gradients
			vcsVector stvkGradientConstraint1 = delta * (-1.0) * ((deltaLength * deltaLength) - (restLength * restLength)) * 2.0;
			vcsVector gradientConstraint1 = stvkGradientConstraint1 * (2 * kStiffness * (1 - exp(stvkPotentialEnergy))) * (-exp(stvkPotentialEnergy));
			vcsVector stvkGradientConstraint2 = delta * ((deltaLength * deltaLength) - (restLength * restLength)) * 2.0;
			vcsVector gradientConstraint2 = stvkGradientConstraint2 * (2 * kStiffness * (1 - exp(stvkPotentialEnergy))) * (-exp(stvkPotentialEnergy));
			float constraintLengthSum = pow(gradientConstraint1.length(), 2) + pow(gradientConstraint2.length(), 2);
			if (constraintLengthSum == 0.0) constraintLengthSum = 0.000001;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator = difference + (alphaHat * dynamicSolverData.totalLagrangeMultiplierSTVKSpring1[index]);
			//float deltaLagrangeMultiplierDenominator = constraintLengthSum + alphaHat;
			//deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator / deltaLagrangeMultiplierDenominator;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;

			float deltaLagrangeMultiplierNumerator1 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring1[index]) + (gama * (gradientConstraint1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier1 = -deltaLagrangeMultiplierNumerator1 / deltaLagrangeMultiplierDenominator1;

			float deltaLagrangeMultiplierNumerator2 = potentialEnergy + (alphaHat * dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring2[index]) + (gama * (gradientConstraint2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2 = ((1.0 + gama) * constraintLengthSum) + alphaHat;
			float deltaLagrangeMultiplier2 = -deltaLagrangeMultiplierNumerator2 / deltaLagrangeMultiplierDenominator2;

			dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring1[index] += deltaLagrangeMultiplier1;
			dynamicSolverData.totalLagrangeMultiplierExponentialSTVKSpring2[index] += deltaLagrangeMultiplier2;

			vec1 = vec1 + (gradientConstraint1 * 0.5 * deltaLagrangeMultiplier1);
			vec2 = vec2 + (gradientConstraint2 * 0.5 * deltaLagrangeMultiplier2);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
		}
	}
}


//exponentialGreenStrainConstraintTri
void PHYSSolverCPU::exponentialGreenStrainConstraintTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			vcsVector vecDifference21 = vec2 - vec1;
			vcsVector vecDifference31 = vec3 - vec1;

			//materialCoordinate
			vcsVector materialCoordinateC1 = staticSolverData.pMaterialCoordinateInvC1List[index];
			vcsVector materialCoordinateC2 = staticSolverData.pMaterialCoordinateInvC2List[index];

			//deformationGradient matrix elements 3x2
			float deformationGradientF1X = (vecDifference21.x * materialCoordinateC1.x) + (vecDifference31.x * materialCoordinateC1.y);
			float deformationGradientF1Y = (vecDifference21.y * materialCoordinateC1.x) + (vecDifference31.y * materialCoordinateC1.y);
			float deformationGradientF1Z = (vecDifference21.z * materialCoordinateC1.x) + (vecDifference31.z * materialCoordinateC1.y);

			float deformationGradientF2X = (vecDifference21.x * materialCoordinateC2.x) + (vecDifference31.x * materialCoordinateC2.y);
			float deformationGradientF2Y = (vecDifference21.y * materialCoordinateC2.x) + (vecDifference31.y * materialCoordinateC2.y);
			float deformationGradientF2Z = (vecDifference21.z * materialCoordinateC2.x) + (vecDifference31.z * materialCoordinateC2.y);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);

			//strainTensor matrix elements 2x2
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, 0.0);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, 0.0);

			float strainConstraint11 = strainTensor1.x - 1.0;
			float strainConstraint12 = strainTensor2.x;
			float strainConstraint22 = strainTensor2.y - 1.0;

			float expStrainConstraint11 = pow((1.0 - exp(strainConstraint11)), 2) * kStiffness;
			float expStrainConstraint12 = pow((1.0 - exp(strainConstraint12)), 2) * kStiffness;
			float expStrainConstraint22 = pow((1.0 - exp(strainConstraint22)), 2) * kStiffness;

			/*if (index == 472)
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += difference;
				staticSolverData.potEneg = expStrainConstraint22;
			}*/

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * materialCoordinateC1.x,
													 deformationGradientF1Y * materialCoordinateC1.x,
													 deformationGradientF1Z * materialCoordinateC1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * materialCoordinateC1.y,
													 deformationGradientF1Y * materialCoordinateC1.y,
													 deformationGradientF1Z * materialCoordinateC1.y) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * materialCoordinateC2.x,
													 deformationGradientF2Y * materialCoordinateC2.x,
													 deformationGradientF2Z * materialCoordinateC2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * materialCoordinateC2.y,
													 deformationGradientF2Y * materialCoordinateC2.y,
													 deformationGradientF2Z * materialCoordinateC2.y) * 2.0;

			vcsVector strainGradient12_1 = vcsVector((deformationGradientF2X * materialCoordinateC1.x) + (deformationGradientF1X * materialCoordinateC2.x),
													 (deformationGradientF2Y * materialCoordinateC1.x) + (deformationGradientF1Y * materialCoordinateC2.x),
													 (deformationGradientF2Z * materialCoordinateC1.x) + (deformationGradientF1Z * materialCoordinateC2.x));
			vcsVector strainGradient12_2 = vcsVector((deformationGradientF2X * materialCoordinateC1.y) + (deformationGradientF1X * materialCoordinateC2.y),
													 (deformationGradientF2Y * materialCoordinateC1.y) + (deformationGradientF1Y * materialCoordinateC2.y),
													 (deformationGradientF2Z * materialCoordinateC1.y) + (deformationGradientF1Z * materialCoordinateC2.y));

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec2 = strainGradient11_1;
			vcsVector strainGradient11Vec3 = strainGradient11_2;
			vcsVector strainGradient11Vec1 = (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			vcsVector expStrainGradient11Vec2 = strainGradient11Vec2 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec3 = strainGradient11Vec3 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec1 = strainGradient11Vec1 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec2 = strainGradient12_1;
			vcsVector strainGradient12Vec3 = strainGradient12_2;
			vcsVector strainGradient12Vec1 = (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			vcsVector expStrainGradient12Vec2 = strainGradient12Vec2 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec3 = strainGradient12Vec3 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec1 = strainGradient12Vec1 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec2 = strainGradient22_1;
			vcsVector strainGradient22Vec3 = strainGradient22_2;
			vcsVector strainGradient22Vec1 = (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			vcsVector expStrainGradient22Vec2 = strainGradient22Vec2 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec3 = strainGradient22Vec3 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec1 = strainGradient22Vec1 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(expStrainGradient11Vec1.length(), 2) + pow(expStrainGradient11Vec2.length(), 2) + pow(expStrainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0.0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(expStrainGradient12Vec1.length(), 2) + pow(expStrainGradient12Vec2.length(), 2) + pow(expStrainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0.0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(expStrainGradient22Vec1.length(), 2) + pow(expStrainGradient22Vec2.length(), 2) + pow(expStrainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0.0) gradientLengthSum22 = 0.000001;

			float oneOverVertexWeights = 0.33;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = expStrainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (expStrainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = expStrainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (expStrainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = expStrainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (expStrainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (expStrainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (expStrainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (expStrainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (expStrainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (expStrainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (expStrainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 stretching
			vec1 = vec1 + (expStrainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (expStrainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (expStrainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = expStrainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (expStrainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = expStrainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (expStrainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = expStrainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (expStrainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (expStrainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (expStrainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (expStrainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//exponentialGreenStrainConstraintModifiedTri
void PHYSSolverCPU::exponentialGreenStrainConstraintModifiedTri(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			vcsVector vecDifference21 = vec2 - vec1;
			vcsVector vecDifference31 = vec3 - vec1;

			//materialCoordinate
			vcsVector materialCoordinateC1 = staticSolverData.pMaterialCoordinateInvC1List[index];
			vcsVector materialCoordinateC2 = staticSolverData.pMaterialCoordinateInvC2List[index];

			//deformationGradient matrix elements 3x2
			float deformationGradientF1X = (vecDifference21.x * materialCoordinateC1.x) + (vecDifference31.x * materialCoordinateC1.y);
			float deformationGradientF1Y = (vecDifference21.y * materialCoordinateC1.x) + (vecDifference31.y * materialCoordinateC1.y);
			float deformationGradientF1Z = (vecDifference21.z * materialCoordinateC1.x) + (vecDifference31.z * materialCoordinateC1.y);

			float deformationGradientF2X = (vecDifference21.x * materialCoordinateC2.x) + (vecDifference31.x * materialCoordinateC2.y);
			float deformationGradientF2Y = (vecDifference21.y * materialCoordinateC2.x) + (vecDifference31.y * materialCoordinateC2.y);
			float deformationGradientF2Z = (vecDifference21.z * materialCoordinateC2.x) + (vecDifference31.z * materialCoordinateC2.y);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);

			//strainTensor matrix elements 2x2
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, 0.0);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, 0.0);

			float strainConstraint11 = sqrt(strainTensor1.x) - 1.0;
			float strainConstraint12 = strainTensor2.x / deformationGradientVector1.length() * deformationGradientVector2.length();
			float strainConstraint22 = sqrt(strainTensor2.y) - 1.0;

			float expStrainConstraint11 = pow((1.0 - exp(strainConstraint11)), 2) * kStiffness;
			float expStrainConstraint12 = pow((1.0 - exp(strainConstraint12)), 2) * kStiffness;
			float expStrainConstraint22 = pow((1.0 - exp(strainConstraint22)), 2) * kStiffness;

			float strain12DividedbyCubes = strainTensor2.x / (pow(deformationGradientVector1.length(), 3) * pow(deformationGradientVector2.length(), 3));
			float strain12DividedbyCubesMultDefGrad2 = pow(deformationGradientVector2.length(), 2) * strain12DividedbyCubes;
			float strain12DividedbyCubesMultDefGrad1 = pow(deformationGradientVector1.length(), 2) * strain12DividedbyCubes;
			float oneOverDefGradLengths12 = 1.0 / (deformationGradientVector1.length() * deformationGradientVector2.length());

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * materialCoordinateC1.x,
													 deformationGradientF1Y * materialCoordinateC1.x,
													 deformationGradientF1Z * materialCoordinateC1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * materialCoordinateC1.y,
													 deformationGradientF1Y * materialCoordinateC1.y,
													 deformationGradientF1Z * materialCoordinateC1.y) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * materialCoordinateC2.x,
													 deformationGradientF2Y * materialCoordinateC2.x,
													 deformationGradientF2Z * materialCoordinateC2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * materialCoordinateC2.y,
													 deformationGradientF2Y * materialCoordinateC2.y,
													 deformationGradientF2Z * materialCoordinateC2.y) * 2.0;

			vcsVector modifiedStrainGradient12_1_LeftofMinus = vcsVector((deformationGradientF2X * materialCoordinateC1.x) + (deformationGradientF1X * materialCoordinateC2.x),
																		 (deformationGradientF2Y * materialCoordinateC1.x) + (deformationGradientF1Y * materialCoordinateC2.x),
																		 (deformationGradientF2Z * materialCoordinateC1.x) + (deformationGradientF1Z * materialCoordinateC2.x)) * oneOverDefGradLengths12;
			vcsVector modifiedStrainGradient12_2_LeftofMinus = vcsVector((deformationGradientF2X * materialCoordinateC1.y) + (deformationGradientF1X * materialCoordinateC2.y),
																		 (deformationGradientF2Y * materialCoordinateC1.y) + (deformationGradientF1Y * materialCoordinateC2.y),
																		 (deformationGradientF2Z * materialCoordinateC1.y) + (deformationGradientF1Z * materialCoordinateC2.y)) * oneOverDefGradLengths12;

			vcsVector strainGradient12_1_RightofMinusPart1 = vcsVector(deformationGradientF1X * materialCoordinateC1.x,
																	   deformationGradientF1Y * materialCoordinateC1.x,
																	   deformationGradientF1Z * materialCoordinateC1.x) * strain12DividedbyCubesMultDefGrad2;
			vcsVector strainGradient12_2_RightofMinusPart1 = vcsVector(deformationGradientF1X * materialCoordinateC1.y,
																	   deformationGradientF1Y * materialCoordinateC1.y,
																	   deformationGradientF1Z * materialCoordinateC1.y) * strain12DividedbyCubesMultDefGrad2;

			vcsVector strainGradient12_1_RightofMinusPart2 = vcsVector(deformationGradientF2X * materialCoordinateC2.x,
																	   deformationGradientF2Y * materialCoordinateC2.x,
																	   deformationGradientF2Z * materialCoordinateC2.x) * strain12DividedbyCubesMultDefGrad1;
			vcsVector strainGradient12_2_RightofMinusPart2 = vcsVector(deformationGradientF2X * materialCoordinateC2.y,
																	   deformationGradientF2Y * materialCoordinateC2.y,
																	   deformationGradientF2Z * materialCoordinateC2.y) * strain12DividedbyCubesMultDefGrad1;

			vcsVector modifiedStrainGradient12_1 = modifiedStrainGradient12_1_LeftofMinus - (strainGradient12_1_RightofMinusPart1 + strainGradient12_1_RightofMinusPart2);
			vcsVector modifiedStrainGradient12_2 = modifiedStrainGradient12_2_LeftofMinus - (strainGradient12_2_RightofMinusPart1 + strainGradient12_2_RightofMinusPart2);

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec2 = strainGradient11_1;
			vcsVector strainGradient11Vec3 = strainGradient11_2;
			vcsVector strainGradient11Vec1 = (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			vcsVector expStrainGradient11Vec2 = strainGradient11Vec2 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec3 = strainGradient11Vec3 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec1 = strainGradient11Vec1 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec2 = modifiedStrainGradient12_1;
			vcsVector strainGradient12Vec3 = modifiedStrainGradient12_2;
			vcsVector strainGradient12Vec1 = (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			vcsVector expStrainGradient12Vec2 = strainGradient12Vec2 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec3 = strainGradient12Vec3 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec1 = strainGradient12Vec1 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec2 = strainGradient22_1;
			vcsVector strainGradient22Vec3 = strainGradient22_2;
			vcsVector strainGradient22Vec1 = (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			vcsVector expStrainGradient22Vec2 = strainGradient22Vec2 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec3 = strainGradient22Vec3 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec1 = strainGradient22Vec1 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(expStrainGradient11Vec1.length(), 2) + pow(expStrainGradient11Vec2.length(), 2) + pow(expStrainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0.0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(expStrainGradient12Vec1.length(), 2) + pow(expStrainGradient12Vec2.length(), 2) + pow(expStrainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0.0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(expStrainGradient22Vec1.length(), 2) + pow(expStrainGradient22Vec2.length(), 2) + pow(expStrainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0.0) gradientLengthSum22 = 0.000001;

			float oneOverVertexWeights = 0.33;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = (2.0 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = (2.0 * expStrainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (expStrainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = (2.0 * expStrainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (expStrainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = (2.0 * expStrainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (expStrainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (expStrainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (expStrainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (expStrainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (expStrainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (expStrainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (expStrainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 stretching
			vec1 = vec1 + (expStrainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (expStrainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (expStrainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = (2.0 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = (2.0 * expStrainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (expStrainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = (2.0 * expStrainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (expStrainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = (2.0 * expStrainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (expStrainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (expStrainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (expStrainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (expStrainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//exponentialGreenStrainConstraintTet
void PHYSSolverCPU::exponentialGreenStrainConstraintTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec0 = staticSolverData.pCenterofMass[0];
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vertex differences with center of mass for fake tet
			vcsVector diffVec1nVec0 = vec1 - vec0;
			vcsVector diffVec2nVec0 = vec2 - vec0;
			vcsVector diffVec3nVec0 = vec3 - vec0;

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			//materialCoordinate with initial center of mass differences for fake tet
			vcsVector materialCoordinateC1 = staticSolverData.pVertexInitialpositionList[vertexIndex1] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC2 = staticSolverData.pVertexInitialpositionList[vertexIndex2] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC3 = staticSolverData.pVertexInitialpositionList[vertexIndex3] - staticSolverData.initialCenterofMass;

			float determinantMatCoord = (materialCoordinateC1.x * materialCoordinateC2.y * materialCoordinateC3.z) +
										(materialCoordinateC2.x * materialCoordinateC3.y * materialCoordinateC1.z) +
										(materialCoordinateC3.x * materialCoordinateC1.y * materialCoordinateC2.z) -
										(materialCoordinateC3.x * materialCoordinateC2.y * materialCoordinateC1.z) -
										(materialCoordinateC1.x * materialCoordinateC3.y * materialCoordinateC2.z) -
										(materialCoordinateC2.x * materialCoordinateC1.y * materialCoordinateC3.z);
			if (determinantMatCoord == 0.0) determinantMatCoord = 0.000001;

			vcsVector inverseMatCoord1 = vcsVector(materialCoordinateC2.y * materialCoordinateC3.z - materialCoordinateC3.y * materialCoordinateC2.z,
												   materialCoordinateC3.y * materialCoordinateC1.z - materialCoordinateC1.y * materialCoordinateC3.z,
												   materialCoordinateC1.y * materialCoordinateC2.z - materialCoordinateC2.y * materialCoordinateC1.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord2 = vcsVector(materialCoordinateC3.x * materialCoordinateC2.z - materialCoordinateC2.x * materialCoordinateC3.z,
												   materialCoordinateC1.x * materialCoordinateC3.z - materialCoordinateC3.x * materialCoordinateC1.z,
												   materialCoordinateC2.x * materialCoordinateC1.z - materialCoordinateC1.x * materialCoordinateC2.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord3 = vcsVector(materialCoordinateC2.x * materialCoordinateC3.y - materialCoordinateC3.x * materialCoordinateC2.y,
												   materialCoordinateC3.x * materialCoordinateC1.y - materialCoordinateC1.x * materialCoordinateC3.y,
												   materialCoordinateC1.x * materialCoordinateC2.y - materialCoordinateC2.x * materialCoordinateC1.y) * (1.0 / determinantMatCoord);


			//deformationGradient matrix elements 3x3
			float deformationGradientF1X = (diffVec1nVec0.x * inverseMatCoord1.x) + (diffVec2nVec0.x * inverseMatCoord1.y) + (diffVec3nVec0.x * inverseMatCoord1.z);
			float deformationGradientF1Y = (diffVec1nVec0.y * inverseMatCoord1.x) + (diffVec2nVec0.y * inverseMatCoord1.y) + (diffVec3nVec0.y * inverseMatCoord1.z);
			float deformationGradientF1Z = (diffVec1nVec0.z * inverseMatCoord1.x) + (diffVec2nVec0.z * inverseMatCoord1.y) + (diffVec3nVec0.z * inverseMatCoord1.z);
			
			float deformationGradientF2X = (diffVec1nVec0.x * inverseMatCoord2.x) + (diffVec2nVec0.x * inverseMatCoord2.y) + (diffVec3nVec0.x * inverseMatCoord2.z);
			float deformationGradientF2Y = (diffVec1nVec0.y * inverseMatCoord2.x) + (diffVec2nVec0.y * inverseMatCoord2.y) + (diffVec3nVec0.y * inverseMatCoord2.z);
			float deformationGradientF2Z = (diffVec1nVec0.z * inverseMatCoord2.x) + (diffVec2nVec0.z * inverseMatCoord2.y) + (diffVec3nVec0.z * inverseMatCoord2.z);
			
			float deformationGradientF3X = (diffVec1nVec0.x * inverseMatCoord3.x) + (diffVec2nVec0.x * inverseMatCoord3.y) + (diffVec3nVec0.x * inverseMatCoord3.z);
			float deformationGradientF3Y = (diffVec1nVec0.y * inverseMatCoord3.x) + (diffVec2nVec0.y * inverseMatCoord3.y) + (diffVec3nVec0.y * inverseMatCoord3.z);
			float deformationGradientF3Z = (diffVec1nVec0.z * inverseMatCoord3.x) + (diffVec2nVec0.z * inverseMatCoord3.y) + (diffVec3nVec0.z * inverseMatCoord3.z);
			
			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);
			vcsVector deformationGradientVector3 = vcsVector(deformationGradientF3X, deformationGradientF3Y, deformationGradientF3Z);

			//strainTensor matrix elements 3x3
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);
			float strainTensorS1Z = deformationGradientVector3.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);
			float strainTensorS2Z = deformationGradientVector3.dot(deformationGradientVector2);

			float strainTensorS3X = deformationGradientVector1.dot(deformationGradientVector3);
			float strainTensorS3Y = deformationGradientVector2.dot(deformationGradientVector3);
			float strainTensorS3Z = deformationGradientVector3.dot(deformationGradientVector3);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, strainTensorS1Z);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, strainTensorS2Z);
			vcsVector strainTensor3 = vcsVector(strainTensorS3X, strainTensorS3Y, strainTensorS3Z);

			float strainConstraint11 = strainTensor1.x - 1.0;
			if (strainConstraint11 > 10.0) strainConstraint11 = 9.99999;
			float strainConstraint22 = strainTensor2.y - 1.0;
			if (strainConstraint22 > 10.0) strainConstraint22 = 9.99999;
			float strainConstraint33 = strainTensor3.z - 1.0;
			if (strainConstraint33 > 10.0) strainConstraint33 = 9.99999;
			float strainConstraint12 = strainTensor2.x;
			if (strainConstraint12 > 10.0) strainConstraint12 = 9.99999;
			float strainConstraint13 = strainTensor3.x;
			if (strainConstraint13 > 10.0) strainConstraint13 = 9.99999;
			float strainConstraint23 = strainTensor3.y;
			if (strainConstraint23 > 10.0) strainConstraint23 = 9.99999;
			
			float expStrainConstraint11 = pow((1.0 - exp(strainConstraint11)), 2) * kStiffness;
			float expStrainConstraint22 = pow((1.0 - exp(strainConstraint22)), 2) * kStiffness;
			float expStrainConstraint33 = pow((1.0 - exp(strainConstraint33)), 2) * kStiffness;
			float expStrainConstraint12 = pow((1.0 - exp(strainConstraint12)), 2) * kStiffness;
			float expStrainConstraint13 = pow((1.0 - exp(strainConstraint13)), 2) * kStiffness;
			float expStrainConstraint23 = pow((1.0 - exp(strainConstraint23)), 2) * kStiffness;

			/*if (index == 242)
			{
				//staticSolverData.relErrorValue = log10(abs(deltaLength - restLength) / abs(restLength));
				//staticSolverData.potEneg += difference;
				staticSolverData.potEneg = expStrainConstraint12 + expStrainConstraint13 + expStrainConstraint23;
			}*/

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
													 deformationGradientF1Y * inverseMatCoord1.x,
													 deformationGradientF1Z * inverseMatCoord1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
													 deformationGradientF1Y * inverseMatCoord1.y,
													 deformationGradientF1Z * inverseMatCoord1.y) * 2.0;
			vcsVector strainGradient11_3 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
													 deformationGradientF1Y * inverseMatCoord1.z,
													 deformationGradientF1Z * inverseMatCoord1.z) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
													 deformationGradientF2Y * inverseMatCoord2.x,
													 deformationGradientF2Z * inverseMatCoord2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
													 deformationGradientF2Y * inverseMatCoord2.y,
													 deformationGradientF2Z * inverseMatCoord2.y) * 2.0;
			vcsVector strainGradient22_3 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
													 deformationGradientF2Y * inverseMatCoord2.z,
													 deformationGradientF2Z * inverseMatCoord2.z) * 2.0;

			vcsVector strainGradient33_1 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
													 deformationGradientF3Y * inverseMatCoord3.x,
													 deformationGradientF3Z * inverseMatCoord3.x) * 2.0;
			vcsVector strainGradient33_2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
													 deformationGradientF3Y * inverseMatCoord3.y,
													 deformationGradientF3Z * inverseMatCoord3.y) * 2.0;
			vcsVector strainGradient33_3 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
													 deformationGradientF3Y * inverseMatCoord3.z,
													 deformationGradientF3Z * inverseMatCoord3.z) * 2.0;

			vcsVector strainGradient12_1 = vcsVector(((deformationGradientF2X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord2.x)),
													 ((deformationGradientF2Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord2.x)),
													 ((deformationGradientF2Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord2.x)));
			vcsVector strainGradient12_2 = vcsVector(((deformationGradientF2X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord2.y)),
													 ((deformationGradientF2Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord2.y)),
													 ((deformationGradientF2Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord2.y)));
			vcsVector strainGradient12_3 = vcsVector(((deformationGradientF2X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord2.z)),
													 ((deformationGradientF2Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord2.z)),
													 ((deformationGradientF2Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord2.z)));

			vcsVector strainGradient13_1 = vcsVector(((deformationGradientF3X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord3.x)),
													 ((deformationGradientF3Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord3.x)),
													 ((deformationGradientF3Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord3.x)));
			vcsVector strainGradient13_2 = vcsVector(((deformationGradientF3X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord3.y)),
													 ((deformationGradientF3Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord3.y)),
													 ((deformationGradientF3Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord3.y)));
			vcsVector strainGradient13_3 = vcsVector(((deformationGradientF3X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord3.z)),
													 ((deformationGradientF3Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord3.z)),
													 ((deformationGradientF3Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord3.z)));

			vcsVector strainGradient23_1 = vcsVector(((deformationGradientF3X * inverseMatCoord2.x) + (deformationGradientF2X * inverseMatCoord3.x)),
													 ((deformationGradientF3Y * inverseMatCoord2.x) + (deformationGradientF2Y * inverseMatCoord3.x)),
													 ((deformationGradientF3Z * inverseMatCoord2.x) + (deformationGradientF2Z * inverseMatCoord3.x)));
			vcsVector strainGradient23_2 = vcsVector(((deformationGradientF3X * inverseMatCoord2.y) + (deformationGradientF2X * inverseMatCoord3.y)),
													 ((deformationGradientF3Y * inverseMatCoord2.y) + (deformationGradientF2Y * inverseMatCoord3.y)),
													 ((deformationGradientF3Z * inverseMatCoord2.y) + (deformationGradientF2Z * inverseMatCoord3.y)));
			vcsVector strainGradient23_3 = vcsVector(((deformationGradientF3X * inverseMatCoord2.z) + (deformationGradientF2X * inverseMatCoord3.z)),
													 ((deformationGradientF3Y * inverseMatCoord2.z) + (deformationGradientF2Y * inverseMatCoord3.z)),
													 ((deformationGradientF3Z * inverseMatCoord2.z) + (deformationGradientF2Z * inverseMatCoord3.z)));

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec1 = strainGradient11_1;
			vcsVector strainGradient11Vec2 = strainGradient11_2;
			vcsVector strainGradient11Vec3 = strainGradient11_3;
			vcsVector strainGradient11Vec0 = (strainGradient11Vec1 * (-1.0)) + (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			vcsVector expStrainGradient11Vec1 = strainGradient11Vec1 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec2 = strainGradient11Vec2 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec3 = strainGradient11Vec3 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec0 = strainGradient11Vec0 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec1 = strainGradient22_1;
			vcsVector strainGradient22Vec2 = strainGradient22_2;
			vcsVector strainGradient22Vec3 = strainGradient22_3;
			vcsVector strainGradient22Vec0 = (strainGradient22Vec1 * (-1.0)) + (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			vcsVector expStrainGradient22Vec1 = strainGradient22Vec1 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec2 = strainGradient22Vec2 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec3 = strainGradient22Vec3 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec0 = strainGradient22Vec0 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));

			//strainGradientOverVertices 33 matrix
			vcsVector strainGradient33Vec1 = strainGradient33_1;
			vcsVector strainGradient33Vec2 = strainGradient33_2;
			vcsVector strainGradient33Vec3 = strainGradient33_3;
			vcsVector strainGradient33Vec0 = (strainGradient33Vec1 * (-1.0)) + (strainGradient33Vec2 * (-1.0)) + (strainGradient33Vec3 * (-1.0));

			vcsVector expStrainGradient33Vec1 = strainGradient33Vec1 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));
			vcsVector expStrainGradient33Vec2 = strainGradient33Vec2 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));
			vcsVector expStrainGradient33Vec3 = strainGradient33Vec3 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));
			vcsVector expStrainGradient33Vec0 = strainGradient33Vec0 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec1 = strainGradient12_1;
			vcsVector strainGradient12Vec2 = strainGradient12_2;
			vcsVector strainGradient12Vec3 = strainGradient12_3;
			vcsVector strainGradient12Vec0 = (strainGradient12Vec1 * (-1.0)) + (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			vcsVector expStrainGradient12Vec1 = strainGradient12Vec1 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec2 = strainGradient12Vec2 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec3 = strainGradient12Vec3 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec0 = strainGradient12Vec0 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));

			//strainGradientOverVertices 13 matrix
			vcsVector strainGradient13Vec1 = strainGradient13_1;
			vcsVector strainGradient13Vec2 = strainGradient13_2;
			vcsVector strainGradient13Vec3 = strainGradient13_3;
			vcsVector strainGradient13Vec0 = (strainGradient13Vec1 * (-1.0)) + (strainGradient13Vec2 * (-1.0)) + (strainGradient13Vec3 * (-1.0));

			vcsVector expStrainGradient13Vec1 = strainGradient13Vec1 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));
			vcsVector expStrainGradient13Vec2 = strainGradient13Vec2 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));
			vcsVector expStrainGradient13Vec3 = strainGradient13Vec3 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));
			vcsVector expStrainGradient13Vec0 = strainGradient13Vec0 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));

			//strainGradientOverVertices 23 matrix
			vcsVector strainGradient23Vec1 = strainGradient23_1;
			vcsVector strainGradient23Vec2 = strainGradient23_2;
			vcsVector strainGradient23Vec3 = strainGradient23_3;
			vcsVector strainGradient23Vec0 = (strainGradient23Vec1 * (-1.0)) + (strainGradient23Vec2 * (-1.0)) + (strainGradient23Vec3 * (-1.0));

			vcsVector expStrainGradient23Vec1 = strainGradient23Vec1 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));
			vcsVector expStrainGradient23Vec2 = strainGradient23Vec2 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));
			vcsVector expStrainGradient23Vec3 = strainGradient23Vec3 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));
			vcsVector expStrainGradient23Vec0 = strainGradient23Vec0 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(expStrainGradient11Vec0.length(), 2) + pow(expStrainGradient11Vec1.length(), 2) + pow(expStrainGradient11Vec2.length(), 2) + pow(expStrainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(expStrainGradient22Vec0.length(), 2) + pow(expStrainGradient22Vec1.length(), 2) + pow(expStrainGradient22Vec2.length(), 2) + pow(expStrainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0) gradientLengthSum22 = 0.000001;

			//gradientLengthSum33
			float gradientLengthSum33 = pow(expStrainGradient33Vec0.length(), 2) + pow(expStrainGradient33Vec1.length(), 2) + pow(expStrainGradient33Vec2.length(), 2) + pow(expStrainGradient33Vec3.length(), 2);
			if (gradientLengthSum33 == 0) gradientLengthSum33 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(expStrainGradient12Vec0.length(), 2) + pow(expStrainGradient12Vec1.length(), 2) + pow(expStrainGradient12Vec2.length(), 2) + pow(expStrainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum13
			float gradientLengthSum13 = pow(expStrainGradient13Vec0.length(), 2) + pow(expStrainGradient13Vec1.length(), 2) + pow(expStrainGradient13Vec2.length(), 2) + pow(expStrainGradient13Vec3.length(), 2);
			if (gradientLengthSum13 == 0) gradientLengthSum13 = 0.000001;

			//gradientLengthSum23
			float gradientLengthSum23 = pow(expStrainGradient23Vec0.length(), 2) + pow(expStrainGradient23Vec1.length(), 2) + pow(expStrainGradient23Vec2.length(), 2) + pow(expStrainGradient23Vec3.length(), 2);
			if (gradientLengthSum23 == 0) gradientLengthSum23 = 0.000001;

			float oneOverVertexWeights = 0.25;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = strainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = expStrainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (expStrainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = expStrainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (expStrainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = expStrainConstraint11 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (expStrainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (expStrainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (expStrainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (expStrainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = strainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = expStrainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (expStrainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = expStrainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (expStrainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = expStrainConstraint22 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (expStrainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (expStrainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (expStrainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (expStrainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_33 = strainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]);
			//float deltaLagrangeMultiplierDenominator1_33 = gradientLengthSum33 + alphaHat;
			//deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator1_33 = expStrainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]) + (gama * (expStrainGradient33Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator2_33 = expStrainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_33[index]) + (gama * (expStrainGradient33Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier2_33 = -deltaLagrangeMultiplierNumerator2_33 / deltaLagrangeMultiplierDenominator2_33;

			float deltaLagrangeMultiplierNumerator3_33 = expStrainConstraint33 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_33[index]) + (gama * (expStrainGradient33Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier3_33 = -deltaLagrangeMultiplierNumerator3_33 / deltaLagrangeMultiplierDenominator3_33;

			dynamicSolverData.totalLagrangeMultiplierStrain1_33[index] += deltaLagrangeMultiplier1_33;
			dynamicSolverData.totalLagrangeMultiplierStrain2_33[index] += deltaLagrangeMultiplier2_33;
			dynamicSolverData.totalLagrangeMultiplierStrain3_33[index] += deltaLagrangeMultiplier3_33;

			//Compute new vector positions for strainTensor33 stretching
			vec1 = vec1 + (expStrainGradient33Vec1 * deltaLagrangeMultiplier1_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec2 = vec2 + (expStrainGradient33Vec2 * deltaLagrangeMultiplier2_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec3 = vec3 + (expStrainGradient33Vec3 * deltaLagrangeMultiplier3_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (expStrainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (expStrainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (expStrainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 shearing
			vec1 = vec1 + (expStrainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (expStrainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (expStrainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]);
			//float deltaLagrangeMultiplierDenominator1_13 = gradientLengthSum13 + alphaHat;
			//deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator1_13 = expStrainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]) + (gama * (expStrainGradient13Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator2_13 = expStrainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_13[index]) + (gama * (expStrainGradient13Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier2_13 = -deltaLagrangeMultiplierNumerator2_13 / deltaLagrangeMultiplierDenominator2_13;

			float deltaLagrangeMultiplierNumerator3_13 = expStrainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_13[index]) + (gama * (expStrainGradient13Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier3_13 = -deltaLagrangeMultiplierNumerator3_13 / deltaLagrangeMultiplierDenominator3_13;

			dynamicSolverData.totalLagrangeMultiplierStrain1_13[index] += deltaLagrangeMultiplier1_13;
			dynamicSolverData.totalLagrangeMultiplierStrain2_13[index] += deltaLagrangeMultiplier2_13;
			dynamicSolverData.totalLagrangeMultiplierStrain3_13[index] += deltaLagrangeMultiplier3_13;

			//Compute new vector positions for strainTensor13 shearing
			vec1 = vec1 + (expStrainGradient13Vec1 * deltaLagrangeMultiplier1_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec2 = vec2 + (expStrainGradient13Vec2 * deltaLagrangeMultiplier2_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec3 = vec3 + (expStrainGradient13Vec3 * deltaLagrangeMultiplier3_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]);
			//float deltaLagrangeMultiplierDenominator1_23 = gradientLengthSum23 + alphaHat;
			//deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator1_23 = expStrainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]) + (gama * (expStrainGradient23Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator2_23 = expStrainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_23[index]) + (gama * (expStrainGradient23Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier2_23 = -deltaLagrangeMultiplierNumerator2_23 / deltaLagrangeMultiplierDenominator2_23;

			float deltaLagrangeMultiplierNumerator3_23 = expStrainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_23[index]) + (gama * (expStrainGradient23Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier3_23 = -deltaLagrangeMultiplierNumerator3_23 / deltaLagrangeMultiplierDenominator3_23;

			dynamicSolverData.totalLagrangeMultiplierStrain1_23[index] += deltaLagrangeMultiplier1_23;
			dynamicSolverData.totalLagrangeMultiplierStrain2_23[index] += deltaLagrangeMultiplier2_23;
			dynamicSolverData.totalLagrangeMultiplierStrain3_23[index] += deltaLagrangeMultiplier3_23;

			//Compute new vector positions for strainTensor23 shearing
			vec1 = vec1 + (expStrainGradient23Vec1 * deltaLagrangeMultiplier1_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec2 = vec2 + (expStrainGradient23Vec2 * deltaLagrangeMultiplier2_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec3 = vec3 + (expStrainGradient23Vec3 * deltaLagrangeMultiplier3_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//exponentialGreenStrainConstraintModifiedTet
void PHYSSolverCPU::exponentialGreenStrainConstraintModifiedTet(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{
	omp_set_num_threads(NUMBER_OF_THREADS);
	#pragma omp parallel num_threads(omp_get_max_threads()) if(dynamicSolverData.omp)
	{
		#pragma omp for schedule(static) ordered
		//iterate faces
		for (int index = 0; index < staticSolverData.faceCount; index++)
		{
			//kStiffness
			float kStiffness = dynamicSolverData.springStiffness;

			//vertexIndices
			int vertexIndex1 = int(staticSolverData.pFaceVertexIndexList[index].x);
			int vertexIndex2 = int(staticSolverData.pFaceVertexIndexList[index].y);
			int vertexIndex3 = int(staticSolverData.pFaceVertexIndexList[index].z);

			//vcsVector for indices
			vcsVector vec0 = staticSolverData.pCenterofMass[0];
			vcsVector vec1 = staticSolverData.pVertexPositionList[vertexIndex1];
			vcsVector vec2 = staticSolverData.pVertexPositionList[vertexIndex2];
			vcsVector vec3 = staticSolverData.pVertexPositionList[vertexIndex3];

			//vertex differences with center of mass for fake tet
			vcsVector diffVec1nVec0 = vec1 - vec0;
			vcsVector diffVec2nVec0 = vec2 - vec0;
			vcsVector diffVec3nVec0 = vec3 - vec0;

			//vcsVectorOld for indices
			vcsVector vecOld1 = staticSolverData.pVertexOldpositionList[vertexIndex1];
			vcsVector vecOld2 = staticSolverData.pVertexOldpositionList[vertexIndex2];
			vcsVector vecOld3 = staticSolverData.pVertexOldpositionList[vertexIndex3];

			//materialCoordinate with initial center of mass differences for fake tet
			vcsVector materialCoordinateC1 = staticSolverData.pVertexInitialpositionList[vertexIndex1] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC2 = staticSolverData.pVertexInitialpositionList[vertexIndex2] - staticSolverData.initialCenterofMass;
			vcsVector materialCoordinateC3 = staticSolverData.pVertexInitialpositionList[vertexIndex3] - staticSolverData.initialCenterofMass;

			float determinantMatCoord = (materialCoordinateC1.x * materialCoordinateC2.y * materialCoordinateC3.z) +
										(materialCoordinateC2.x * materialCoordinateC3.y * materialCoordinateC1.z) +
										(materialCoordinateC3.x * materialCoordinateC1.y * materialCoordinateC2.z) -
										(materialCoordinateC3.x * materialCoordinateC2.y * materialCoordinateC1.z) -
										(materialCoordinateC1.x * materialCoordinateC3.y * materialCoordinateC2.z) -
										(materialCoordinateC2.x * materialCoordinateC1.y * materialCoordinateC3.z);
			if (determinantMatCoord == 0.0) determinantMatCoord = 0.000001;


			vcsVector inverseMatCoord1 = vcsVector(materialCoordinateC2.y * materialCoordinateC3.z - materialCoordinateC3.y * materialCoordinateC2.z,
												   materialCoordinateC3.y * materialCoordinateC1.z - materialCoordinateC1.y * materialCoordinateC3.z,
												   materialCoordinateC1.y * materialCoordinateC2.z - materialCoordinateC2.y * materialCoordinateC1.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord2 = vcsVector(materialCoordinateC3.x * materialCoordinateC2.z - materialCoordinateC2.x * materialCoordinateC3.z,
												   materialCoordinateC1.x * materialCoordinateC3.z - materialCoordinateC3.x * materialCoordinateC1.z,
												   materialCoordinateC2.x * materialCoordinateC1.z - materialCoordinateC1.x * materialCoordinateC2.z) * (1.0 / determinantMatCoord);

			vcsVector inverseMatCoord3 = vcsVector(materialCoordinateC2.x * materialCoordinateC3.y - materialCoordinateC3.x * materialCoordinateC2.y,
												   materialCoordinateC3.x * materialCoordinateC1.y - materialCoordinateC1.x * materialCoordinateC3.y,
												   materialCoordinateC1.x * materialCoordinateC2.y - materialCoordinateC2.x * materialCoordinateC1.y) * (1.0 / determinantMatCoord);


			//deformationGradient matrix elements 3x3
			float deformationGradientF1X = (diffVec1nVec0.x * inverseMatCoord1.x) + (diffVec2nVec0.x * inverseMatCoord1.y) + (diffVec3nVec0.x * inverseMatCoord1.z);
			float deformationGradientF1Y = (diffVec1nVec0.y * inverseMatCoord1.x) + (diffVec2nVec0.y * inverseMatCoord1.y) + (diffVec3nVec0.y * inverseMatCoord1.z);
			float deformationGradientF1Z = (diffVec1nVec0.z * inverseMatCoord1.x) + (diffVec2nVec0.z * inverseMatCoord1.y) + (diffVec3nVec0.z * inverseMatCoord1.z);

			float deformationGradientF2X = (diffVec1nVec0.x * inverseMatCoord2.x) + (diffVec2nVec0.x * inverseMatCoord2.y) + (diffVec3nVec0.x * inverseMatCoord2.z);
			float deformationGradientF2Y = (diffVec1nVec0.y * inverseMatCoord2.x) + (diffVec2nVec0.y * inverseMatCoord2.y) + (diffVec3nVec0.y * inverseMatCoord2.z);
			float deformationGradientF2Z = (diffVec1nVec0.z * inverseMatCoord2.x) + (diffVec2nVec0.z * inverseMatCoord2.y) + (diffVec3nVec0.z * inverseMatCoord2.z);

			float deformationGradientF3X = (diffVec1nVec0.x * inverseMatCoord3.x) + (diffVec2nVec0.x * inverseMatCoord3.y) + (diffVec3nVec0.x * inverseMatCoord3.z);
			float deformationGradientF3Y = (diffVec1nVec0.y * inverseMatCoord3.x) + (diffVec2nVec0.y * inverseMatCoord3.y) + (diffVec3nVec0.y * inverseMatCoord3.z);
			float deformationGradientF3Z = (diffVec1nVec0.z * inverseMatCoord3.x) + (diffVec2nVec0.z * inverseMatCoord3.y) + (diffVec3nVec0.z * inverseMatCoord3.z);

			//deformationGradient matrix
			vcsVector deformationGradientVector1 = vcsVector(deformationGradientF1X, deformationGradientF1Y, deformationGradientF1Z);
			vcsVector deformationGradientVector2 = vcsVector(deformationGradientF2X, deformationGradientF2Y, deformationGradientF2Z);
			vcsVector deformationGradientVector3 = vcsVector(deformationGradientF3X, deformationGradientF3Y, deformationGradientF3Z);

			//strainTensor matrix elements 3x3
			float strainTensorS1X = deformationGradientVector1.dot(deformationGradientVector1);
			float strainTensorS1Y = deformationGradientVector2.dot(deformationGradientVector1);
			float strainTensorS1Z = deformationGradientVector3.dot(deformationGradientVector1);

			float strainTensorS2X = deformationGradientVector1.dot(deformationGradientVector2);
			float strainTensorS2Y = deformationGradientVector2.dot(deformationGradientVector2);
			float strainTensorS2Z = deformationGradientVector3.dot(deformationGradientVector2);

			float strainTensorS3X = deformationGradientVector1.dot(deformationGradientVector3);
			float strainTensorS3Y = deformationGradientVector2.dot(deformationGradientVector3);
			float strainTensorS3Z = deformationGradientVector3.dot(deformationGradientVector3);

			//strainTensor matrix
			vcsVector strainTensor1 = vcsVector(strainTensorS1X, strainTensorS1Y, strainTensorS1Z);
			vcsVector strainTensor2 = vcsVector(strainTensorS2X, strainTensorS2Y, strainTensorS2Z);
			vcsVector strainTensor3 = vcsVector(strainTensorS3X, strainTensorS3Y, strainTensorS3Z);

			float strainConstraint11 = sqrt(strainTensor1.x) - 1.0;
			if (strainConstraint11 > 10.0) strainConstraint11 = 9.99999;
			float strainConstraint22 = sqrt(strainTensor2.y) - 1.0;
			if (strainConstraint22 > 10.0) strainConstraint22 = 9.99999;
			float strainConstraint33 = sqrt(strainTensor3.z) - 1.0;
			if (strainConstraint33 > 10.0) strainConstraint33 = 9.99999;
			float strainConstraint12 = strainTensor2.x / (deformationGradientVector1.length() * deformationGradientVector2.length());
			if (strainConstraint12 > 10.0) strainConstraint12 = 9.99999;
			float strainConstraint13 = strainTensor3.x / (deformationGradientVector1.length() * deformationGradientVector3.length());
			if (strainConstraint13 > 10.0) strainConstraint13 = 9.99999;
			float strainConstraint23 = strainTensor3.y / (deformationGradientVector2.length() * deformationGradientVector3.length());
			if (strainConstraint23 > 10.0) strainConstraint23 = 9.99999;

			float expStrainConstraint11 = pow((1.0 - exp(strainConstraint11)), 2) * kStiffness;
			float expStrainConstraint22 = pow((1.0 - exp(strainConstraint22)), 2) * kStiffness;
			float expStrainConstraint33 = pow((1.0 - exp(strainConstraint33)), 2) * kStiffness;
			float expStrainConstraint12 = pow((1.0 - exp(strainConstraint12)), 2) * kStiffness;
			float expStrainConstraint13 = pow((1.0 - exp(strainConstraint13)), 2) * kStiffness;
			float expStrainConstraint23 = pow((1.0 - exp(strainConstraint23)), 2) * kStiffness;

			float squareofDefGrad1 = pow(deformationGradientVector1.length(), 2);
			if (squareofDefGrad1 > 10.0) squareofDefGrad1 = 10.0;
			float squareofDefGrad2 = pow(deformationGradientVector2.length(), 2);
			if (squareofDefGrad2 > 10.0) squareofDefGrad2 = 10.0;
			float squareofDefGrad3 = pow(deformationGradientVector3.length(), 2);
			if (squareofDefGrad3 > 10.0) squareofDefGrad3 = 10.0;

			float cubeofDefGrad1 = pow(deformationGradientVector1.length(), 3);
			if (cubeofDefGrad1 > 20.0) cubeofDefGrad1 = 20.0;
			float cubeofDefGrad2 = pow(deformationGradientVector2.length(), 3);
			if (cubeofDefGrad2 > 20.0) cubeofDefGrad2 = 20.0;
			float cubeofDefGrad3 = pow(deformationGradientVector3.length(), 3);
			if (cubeofDefGrad3 > 20.0) cubeofDefGrad3 = 20.0;

			float strain12DividedbyCubes = strainTensor2.x / (cubeofDefGrad1 * cubeofDefGrad2);
			float oneOverDefGradLengths12 = 1.0 / (deformationGradientVector1.length() * deformationGradientVector2.length());

			float strain13DividedbyCubes = strainTensor3.x / (cubeofDefGrad1 * cubeofDefGrad3);
			float oneOverDefGradLengths13 = 1.0 / (deformationGradientVector1.length() * deformationGradientVector3.length());

			float strain23DividedbyCubes = strainTensor3.y / (cubeofDefGrad2 * cubeofDefGrad3);
			float oneOverDefGradLengths23 = 1.0 / (deformationGradientVector2.length() * deformationGradientVector3.length());

			//strainGradient matrix elements
			vcsVector strainGradient11_1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
													 deformationGradientF1Y * inverseMatCoord1.x,
													 deformationGradientF1Z * inverseMatCoord1.x) * 2.0;
			vcsVector strainGradient11_2 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
													 deformationGradientF1Y * inverseMatCoord1.y,
													 deformationGradientF1Z * inverseMatCoord1.y) * 2.0;
			vcsVector strainGradient11_3 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
													 deformationGradientF1Y * inverseMatCoord1.z,
													 deformationGradientF1Z * inverseMatCoord1.z) * 2.0;

			vcsVector strainGradient22_1 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
													 deformationGradientF2Y * inverseMatCoord2.x,
													 deformationGradientF2Z * inverseMatCoord2.x) * 2.0;
			vcsVector strainGradient22_2 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
													 deformationGradientF2Y * inverseMatCoord2.y,
													 deformationGradientF2Z * inverseMatCoord2.y) * 2.0;
			vcsVector strainGradient22_3 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
													 deformationGradientF2Y * inverseMatCoord2.z,
													 deformationGradientF2Z * inverseMatCoord2.z) * 2.0;

			vcsVector strainGradient33_1 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
													 deformationGradientF3Y * inverseMatCoord3.x,
													 deformationGradientF3Z * inverseMatCoord3.x) * 2.0;
			vcsVector strainGradient33_2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
													 deformationGradientF3Y * inverseMatCoord3.y,
													 deformationGradientF3Z * inverseMatCoord3.y) * 2.0;
			vcsVector strainGradient33_3 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
													 deformationGradientF3Y * inverseMatCoord3.z,
													 deformationGradientF3Z * inverseMatCoord3.z) * 2.0;

			vcsVector modifiedStrainGradient12_1_LeftofMinus = vcsVector((deformationGradientF2X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord2.x),
																		 (deformationGradientF2Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord2.x),
																		 (deformationGradientF2Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord2.x));
			vcsVector modifiedStrainGradient12_2_LeftofMinus = vcsVector((deformationGradientF2X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord2.y),
																		 (deformationGradientF2Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord2.y),
																		 (deformationGradientF2Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord2.y));
			vcsVector modifiedStrainGradient12_3_LeftofMinus = vcsVector((deformationGradientF2X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord2.z),
																		 (deformationGradientF2Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord2.z),
																		 (deformationGradientF2Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord2.z));

			vcsVector strainGradient12_1_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
																	   deformationGradientF1Y * inverseMatCoord1.x,
																	   deformationGradientF1Z * inverseMatCoord1.x);
			vcsVector strainGradient12_2_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
																	   deformationGradientF1Y * inverseMatCoord1.y,
																	   deformationGradientF1Z * inverseMatCoord1.y);
			vcsVector strainGradient12_3_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
																	   deformationGradientF1Y * inverseMatCoord1.z,
																	   deformationGradientF1Z * inverseMatCoord1.z);

			vcsVector strainGradient12_1_RightofMinusPart2 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
																	   deformationGradientF2Y * inverseMatCoord2.x,
																	   deformationGradientF2Z * inverseMatCoord2.x);
			vcsVector strainGradient12_2_RightofMinusPart2 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
																	   deformationGradientF2Y * inverseMatCoord2.y,
																	   deformationGradientF2Z * inverseMatCoord2.y);
			vcsVector strainGradient12_3_RightofMinusPart2 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
																	   deformationGradientF2Y * inverseMatCoord2.z,
																	   deformationGradientF2Z * inverseMatCoord2.z);

			vcsVector modifiedStrainGradient12_1_RightofMinus = (strainGradient12_1_RightofMinusPart1 * squareofDefGrad2) + (strainGradient12_1_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient12_2_RightofMinus = (strainGradient12_2_RightofMinusPart1 * squareofDefGrad2) + (strainGradient12_2_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient12_3_RightofMinus = (strainGradient12_3_RightofMinusPart1 * squareofDefGrad2) + (strainGradient12_3_RightofMinusPart2 * squareofDefGrad1);

			vcsVector modifiedStrainGradient12_1 = (modifiedStrainGradient12_1_LeftofMinus * oneOverDefGradLengths12) - (modifiedStrainGradient12_1_RightofMinus * strain12DividedbyCubes);
			vcsVector modifiedStrainGradient12_2 = (modifiedStrainGradient12_2_LeftofMinus * oneOverDefGradLengths12) - (modifiedStrainGradient12_2_RightofMinus * strain12DividedbyCubes);
			vcsVector modifiedStrainGradient12_3 = (modifiedStrainGradient12_3_LeftofMinus * oneOverDefGradLengths12) - (modifiedStrainGradient12_3_RightofMinus * strain12DividedbyCubes);

			vcsVector modifiedStrainGradient13_1_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord1.x) + (deformationGradientF1X * inverseMatCoord3.x),
																		 (deformationGradientF3Y * inverseMatCoord1.x) + (deformationGradientF1Y * inverseMatCoord3.x),
																		 (deformationGradientF3Z * inverseMatCoord1.x) + (deformationGradientF1Z * inverseMatCoord3.x));
			vcsVector modifiedStrainGradient13_2_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord1.y) + (deformationGradientF1X * inverseMatCoord3.y),
																		 (deformationGradientF3Y * inverseMatCoord1.y) + (deformationGradientF1Y * inverseMatCoord3.y),
																		 (deformationGradientF3Z * inverseMatCoord1.y) + (deformationGradientF1Z * inverseMatCoord3.y));
			vcsVector modifiedStrainGradient13_3_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord1.z) + (deformationGradientF1X * inverseMatCoord3.z),
																		 (deformationGradientF3Y * inverseMatCoord1.z) + (deformationGradientF1Y * inverseMatCoord3.z),
																		 (deformationGradientF3Z * inverseMatCoord1.z) + (deformationGradientF1Z * inverseMatCoord3.z));

			vcsVector strainGradient13_1_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.x,
																	   deformationGradientF1Y * inverseMatCoord1.x,
																	   deformationGradientF1Z * inverseMatCoord1.x);
			vcsVector strainGradient13_2_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.y,
																	   deformationGradientF1Y * inverseMatCoord1.y,
																	   deformationGradientF1Z * inverseMatCoord1.y);
			vcsVector strainGradient13_3_RightofMinusPart1 = vcsVector(deformationGradientF1X * inverseMatCoord1.z,
																	   deformationGradientF1Y * inverseMatCoord1.z,
																	   deformationGradientF1Z * inverseMatCoord1.z);

			vcsVector strainGradient13_1_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
																	   deformationGradientF3Y * inverseMatCoord3.x,
																	   deformationGradientF3Z * inverseMatCoord3.x);
			vcsVector strainGradient13_2_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
																	   deformationGradientF3Y * inverseMatCoord3.y,
																	   deformationGradientF3Z * inverseMatCoord3.y);
			vcsVector strainGradient13_3_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
																	   deformationGradientF3Y * inverseMatCoord3.z,
																	   deformationGradientF3Z * inverseMatCoord3.z);

			vcsVector modifiedStrainGradient13_1_RightofMinus = (strainGradient13_1_RightofMinusPart1 * squareofDefGrad3) + (strainGradient13_1_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient13_2_RightofMinus = (strainGradient13_2_RightofMinusPart1 * squareofDefGrad3) + (strainGradient13_2_RightofMinusPart2 * squareofDefGrad1);
			vcsVector modifiedStrainGradient13_3_RightofMinus = (strainGradient13_3_RightofMinusPart1 * squareofDefGrad3) + (strainGradient13_3_RightofMinusPart2 * squareofDefGrad1);

			vcsVector modifiedStrainGradient13_1 = (modifiedStrainGradient13_1_LeftofMinus * oneOverDefGradLengths13) - (modifiedStrainGradient13_1_RightofMinus * strain13DividedbyCubes);
			vcsVector modifiedStrainGradient13_2 = (modifiedStrainGradient13_2_LeftofMinus * oneOverDefGradLengths13) - (modifiedStrainGradient13_2_RightofMinus * strain13DividedbyCubes);
			vcsVector modifiedStrainGradient13_3 = (modifiedStrainGradient13_3_LeftofMinus * oneOverDefGradLengths13) - (modifiedStrainGradient13_3_RightofMinus * strain13DividedbyCubes);

			vcsVector modifiedStrainGradient23_1_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord2.x) + (deformationGradientF2X * inverseMatCoord3.x),
																		 (deformationGradientF3Y * inverseMatCoord2.x) + (deformationGradientF2Y * inverseMatCoord3.x),
																		 (deformationGradientF3Z * inverseMatCoord2.x) + (deformationGradientF2Z * inverseMatCoord3.x));
			vcsVector modifiedStrainGradient23_2_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord2.y) + (deformationGradientF2X * inverseMatCoord3.y),
																		 (deformationGradientF3Y * inverseMatCoord2.y) + (deformationGradientF2Y * inverseMatCoord3.y),
																		 (deformationGradientF3Z * inverseMatCoord2.y) + (deformationGradientF2Z * inverseMatCoord3.y));
			vcsVector modifiedStrainGradient23_3_LeftofMinus = vcsVector((deformationGradientF3X * inverseMatCoord2.z) + (deformationGradientF2X * inverseMatCoord3.z),
																		 (deformationGradientF3Y * inverseMatCoord2.z) + (deformationGradientF2Y * inverseMatCoord3.z),
																		 (deformationGradientF3Z * inverseMatCoord2.z) + (deformationGradientF2Z * inverseMatCoord3.z));

			vcsVector strainGradient23_1_RightofMinusPart1 = vcsVector(deformationGradientF2X * inverseMatCoord2.x,
																	   deformationGradientF2Y * inverseMatCoord2.x,
																	   deformationGradientF2Z * inverseMatCoord2.x);
			vcsVector strainGradient23_2_RightofMinusPart1 = vcsVector(deformationGradientF2X * inverseMatCoord2.y,
																	   deformationGradientF2Y * inverseMatCoord2.y,
																	   deformationGradientF2Z * inverseMatCoord2.y);
			vcsVector strainGradient23_3_RightofMinusPart1 = vcsVector(deformationGradientF2X * inverseMatCoord2.z,
																	   deformationGradientF2Y * inverseMatCoord2.z,
																	   deformationGradientF2Z * inverseMatCoord2.z);

			vcsVector strainGradient23_1_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.x,
																	   deformationGradientF3Y * inverseMatCoord3.x,
																	   deformationGradientF3Z * inverseMatCoord3.x);
			vcsVector strainGradient23_2_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.y,
																	   deformationGradientF3Y * inverseMatCoord3.y,
																	   deformationGradientF3Z * inverseMatCoord3.y);
			vcsVector strainGradient23_3_RightofMinusPart2 = vcsVector(deformationGradientF3X * inverseMatCoord3.z,
																	   deformationGradientF3Y * inverseMatCoord3.z,
																	   deformationGradientF3Z * inverseMatCoord3.z);

			vcsVector modifiedStrainGradient23_1_RightofMinus = (strainGradient23_1_RightofMinusPart1 * squareofDefGrad3) + (strainGradient23_1_RightofMinusPart2 * squareofDefGrad2);
			vcsVector modifiedStrainGradient23_2_RightofMinus = (strainGradient23_2_RightofMinusPart1 * squareofDefGrad3) + (strainGradient23_2_RightofMinusPart2 * squareofDefGrad2);
			vcsVector modifiedStrainGradient23_3_RightofMinus = (strainGradient23_3_RightofMinusPart1 * squareofDefGrad3) + (strainGradient23_3_RightofMinusPart2 * squareofDefGrad2);

			vcsVector modifiedStrainGradient23_1 = (modifiedStrainGradient23_1_LeftofMinus * oneOverDefGradLengths23) - (modifiedStrainGradient23_1_RightofMinus * strain23DividedbyCubes);
			vcsVector modifiedStrainGradient23_2 = (modifiedStrainGradient23_2_LeftofMinus * oneOverDefGradLengths23) - (modifiedStrainGradient23_2_RightofMinus * strain23DividedbyCubes);
			vcsVector modifiedStrainGradient23_3 = (modifiedStrainGradient23_3_LeftofMinus * oneOverDefGradLengths23) - (modifiedStrainGradient23_3_RightofMinus * strain23DividedbyCubes);

			//strainGradientOverVertices 11 matrix
			vcsVector strainGradient11Vec1 = strainGradient11_1;
			vcsVector strainGradient11Vec2 = strainGradient11_2;
			vcsVector strainGradient11Vec3 = strainGradient11_3;
			vcsVector strainGradient11Vec0 = (strainGradient11Vec1 * (-1.0)) + (strainGradient11Vec2 * (-1.0)) + (strainGradient11Vec3 * (-1.0));

			vcsVector expStrainGradient11Vec1 = strainGradient11Vec1 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec2 = strainGradient11Vec2 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec3 = strainGradient11Vec3 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));
			vcsVector expStrainGradient11Vec0 = strainGradient11Vec0 * (2 * kStiffness * (1 - exp(strainConstraint11))) * (-exp(strainConstraint11));

			//strainGradientOverVertices 22 matrix
			vcsVector strainGradient22Vec1 = strainGradient22_1;
			vcsVector strainGradient22Vec2 = strainGradient22_2;
			vcsVector strainGradient22Vec3 = strainGradient22_3;
			vcsVector strainGradient22Vec0 = (strainGradient22Vec1 * (-1.0)) + (strainGradient22Vec2 * (-1.0)) + (strainGradient22Vec3 * (-1.0));

			vcsVector expStrainGradient22Vec1 = strainGradient22Vec1 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec2 = strainGradient22Vec2 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec3 = strainGradient22Vec3 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));
			vcsVector expStrainGradient22Vec0 = strainGradient22Vec0 * (2 * kStiffness * (1 - exp(strainConstraint22))) * (-exp(strainConstraint22));

			//strainGradientOverVertices 33 matrix
			vcsVector strainGradient33Vec1 = strainGradient33_1;
			vcsVector strainGradient33Vec2 = strainGradient33_2;
			vcsVector strainGradient33Vec3 = strainGradient33_3;
			vcsVector strainGradient33Vec0 = (strainGradient33Vec1 * (-1.0)) + (strainGradient33Vec2 * (-1.0)) + (strainGradient33Vec3 * (-1.0));

			vcsVector expStrainGradient33Vec1 = strainGradient33Vec1 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));
			vcsVector expStrainGradient33Vec2 = strainGradient33Vec2 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));
			vcsVector expStrainGradient33Vec3 = strainGradient33Vec3 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));
			vcsVector expStrainGradient33Vec0 = strainGradient33Vec0 * (2 * kStiffness * (1 - exp(strainConstraint33))) * (-exp(strainConstraint33));

			//strainGradientOverVertices 12 matrix
			vcsVector strainGradient12Vec1 = modifiedStrainGradient12_1;
			vcsVector strainGradient12Vec2 = modifiedStrainGradient12_2;
			vcsVector strainGradient12Vec3 = modifiedStrainGradient12_3;
			vcsVector strainGradient12Vec0 = (strainGradient12Vec1 * (-1.0)) + (strainGradient12Vec2 * (-1.0)) + (strainGradient12Vec3 * (-1.0));

			vcsVector expStrainGradient12Vec1 = strainGradient12Vec1 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec2 = strainGradient12Vec2 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec3 = strainGradient12Vec3 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));
			vcsVector expStrainGradient12Vec0 = strainGradient12Vec0 * (2 * kStiffness * (1 - exp(strainConstraint12))) * (-exp(strainConstraint12));

			//strainGradientOverVertices 13 matrix
			vcsVector strainGradient13Vec1 = modifiedStrainGradient13_1;
			vcsVector strainGradient13Vec2 = modifiedStrainGradient13_2;
			vcsVector strainGradient13Vec3 = modifiedStrainGradient13_3;
			vcsVector strainGradient13Vec0 = (strainGradient13Vec1 * (-1.0)) + (strainGradient13Vec2 * (-1.0)) + (strainGradient13Vec3 * (-1.0));

			vcsVector expStrainGradient13Vec1 = strainGradient13Vec1 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));
			vcsVector expStrainGradient13Vec2 = strainGradient13Vec2 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));
			vcsVector expStrainGradient13Vec3 = strainGradient13Vec3 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));
			vcsVector expStrainGradient13Vec0 = strainGradient13Vec0 * (2 * kStiffness * (1 - exp(strainConstraint13))) * (-exp(strainConstraint13));

			//strainGradientOverVertices 23 matrix
			vcsVector strainGradient23Vec1 = modifiedStrainGradient23_1;
			vcsVector strainGradient23Vec2 = modifiedStrainGradient23_2;
			vcsVector strainGradient23Vec3 = modifiedStrainGradient23_3;
			vcsVector strainGradient23Vec0 = (strainGradient23Vec1 * (-1.0)) + (strainGradient23Vec2 * (-1.0)) + (strainGradient23Vec3 * (-1.0));

			vcsVector expStrainGradient23Vec1 = strainGradient23Vec1 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));
			vcsVector expStrainGradient23Vec2 = strainGradient23Vec2 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));
			vcsVector expStrainGradient23Vec3 = strainGradient23Vec3 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));
			vcsVector expStrainGradient23Vec0 = strainGradient23Vec0 * (2 * kStiffness * (1 - exp(strainConstraint23))) * (-exp(strainConstraint23));

			//gradientLengthSum11
			float gradientLengthSum11 = pow(expStrainGradient11Vec0.length(), 2) + pow(expStrainGradient11Vec1.length(), 2) + pow(expStrainGradient11Vec2.length(), 2) + pow(expStrainGradient11Vec3.length(), 2);
			if (gradientLengthSum11 == 0) gradientLengthSum11 = 0.000001;

			//gradientLengthSum22
			float gradientLengthSum22 = pow(expStrainGradient22Vec0.length(), 2) + pow(expStrainGradient22Vec1.length(), 2) + pow(expStrainGradient22Vec2.length(), 2) + pow(expStrainGradient22Vec3.length(), 2);
			if (gradientLengthSum22 == 0) gradientLengthSum22 = 0.000001;

			//gradientLengthSum33
			float gradientLengthSum33 = pow(expStrainGradient33Vec0.length(), 2) + pow(expStrainGradient33Vec1.length(), 2) + pow(expStrainGradient33Vec2.length(), 2) + pow(expStrainGradient33Vec3.length(), 2);
			if (gradientLengthSum33 == 0) gradientLengthSum33 = 0.000001;

			//gradientLengthSum12
			float gradientLengthSum12 = pow(expStrainGradient12Vec0.length(), 2) + pow(expStrainGradient12Vec1.length(), 2) + pow(expStrainGradient12Vec2.length(), 2) + pow(expStrainGradient12Vec3.length(), 2);
			if (gradientLengthSum12 == 0) gradientLengthSum12 = 0.000001;

			//gradientLengthSum13
			float gradientLengthSum13 = pow(expStrainGradient13Vec0.length(), 2) + pow(expStrainGradient13Vec1.length(), 2) + pow(expStrainGradient13Vec2.length(), 2) + pow(expStrainGradient13Vec3.length(), 2);
			if (gradientLengthSum13 == 0) gradientLengthSum13 = 0.000001;

			//gradientLengthSum23
			float gradientLengthSum23 = pow(expStrainGradient23Vec0.length(), 2) + pow(expStrainGradient23Vec1.length(), 2) + pow(expStrainGradient23Vec2.length(), 2) + pow(expStrainGradient23Vec3.length(), 2);
			if (gradientLengthSum23 == 0) gradientLengthSum23 = 0.000001;

			float oneOverVertexWeights = 0.25;

			float alphaCompliance = dynamicSolverData.alphaCompliance;

			float alphaHat = alphaCompliance / (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_11 = (2 * strainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]);
			//float deltaLagrangeMultiplierDenominator1_11 = gradientLengthSum11 + alphaHat;
			//deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			//with damping
			float betaHat = dynamicSolverData.drag * (dynamicSolverData.deltaTime * dynamicSolverData.deltaTime);
			float gama = (alphaHat * betaHat) / dynamicSolverData.deltaTime;
			vcsVector velocity1 = vec1 - vecOld1;
			vcsVector velocity2 = vec2 - vecOld2;
			vcsVector velocity3 = vec3 - vecOld3;

			float deltaLagrangeMultiplierNumerator1_11 = (2 * expStrainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_11[index]) + (gama * (expStrainGradient11Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier1_11 = -deltaLagrangeMultiplierNumerator1_11 / deltaLagrangeMultiplierDenominator1_11;

			float deltaLagrangeMultiplierNumerator2_11 = (2 * expStrainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_11[index]) + (gama * (expStrainGradient11Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier2_11 = -deltaLagrangeMultiplierNumerator2_11 / deltaLagrangeMultiplierDenominator2_11;

			float deltaLagrangeMultiplierNumerator3_11 = (2 * expStrainConstraint11 * sqrt(strainTensor1.x)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_11[index]) + (gama * (expStrainGradient11Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_11 = ((1.0 + gama) * gradientLengthSum11) + alphaHat;
			float deltaLagrangeMultiplier3_11 = -deltaLagrangeMultiplierNumerator3_11 / deltaLagrangeMultiplierDenominator3_11;

			dynamicSolverData.totalLagrangeMultiplierStrain1_11[index] += deltaLagrangeMultiplier1_11;
			dynamicSolverData.totalLagrangeMultiplierStrain2_11[index] += deltaLagrangeMultiplier2_11;
			dynamicSolverData.totalLagrangeMultiplierStrain3_11[index] += deltaLagrangeMultiplier3_11;
			
			//Compute new vector positions for strainTensor11 stretching
			vec1 = vec1 + (expStrainGradient11Vec1 * deltaLagrangeMultiplier1_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec2 = vec2 + (expStrainGradient11Vec2 * deltaLagrangeMultiplier2_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);
			vec3 = vec3 + (expStrainGradient11Vec3 * deltaLagrangeMultiplier3_11 * oneOverVertexWeights * dynamicSolverData.strainStiffness11);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_22 = (2 * strainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]);
			//float deltaLagrangeMultiplierDenominator1_22 = gradientLengthSum22 + alphaHat;
			//deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator1_22 = (2 * expStrainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_22[index]) + (gama * (expStrainGradient22Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier1_22 = -deltaLagrangeMultiplierNumerator1_22 / deltaLagrangeMultiplierDenominator1_22;

			float deltaLagrangeMultiplierNumerator2_22 = (2 * expStrainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_22[index]) + (gama * (expStrainGradient22Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier2_22 = -deltaLagrangeMultiplierNumerator2_22 / deltaLagrangeMultiplierDenominator2_22;

			float deltaLagrangeMultiplierNumerator3_22 = (2 * expStrainConstraint22 * sqrt(strainTensor2.y)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_22[index]) + (gama * (expStrainGradient22Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_22 = ((1.0 + gama) * gradientLengthSum22) + alphaHat;
			float deltaLagrangeMultiplier3_22 = -deltaLagrangeMultiplierNumerator3_22 / deltaLagrangeMultiplierDenominator3_22;

			dynamicSolverData.totalLagrangeMultiplierStrain1_22[index] += deltaLagrangeMultiplier1_22;
			dynamicSolverData.totalLagrangeMultiplierStrain2_22[index] += deltaLagrangeMultiplier2_22;
			dynamicSolverData.totalLagrangeMultiplierStrain3_22[index] += deltaLagrangeMultiplier3_22;

			//Compute new vector positions for strainTensor22 stretching
			vec1 = vec1 + (expStrainGradient22Vec1 * deltaLagrangeMultiplier1_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec2 = vec2 + (expStrainGradient22Vec2 * deltaLagrangeMultiplier2_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);
			vec3 = vec3 + (expStrainGradient22Vec3 * deltaLagrangeMultiplier3_22 * oneOverVertexWeights * dynamicSolverData.strainStiffness22);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_33 = (2 * strainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]);
			//float deltaLagrangeMultiplierDenominator1_33 = gradientLengthSum33 + alphaHat;
			//deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator1_33 = (2 * expStrainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_33[index]) + (gama * (expStrainGradient33Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier1_33 = -deltaLagrangeMultiplierNumerator1_33 / deltaLagrangeMultiplierDenominator1_33;

			float deltaLagrangeMultiplierNumerator2_33 = (2 * expStrainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_33[index]) + (gama * (expStrainGradient33Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier2_33 = -deltaLagrangeMultiplierNumerator2_33 / deltaLagrangeMultiplierDenominator2_33;

			float deltaLagrangeMultiplierNumerator3_33 = (2 * expStrainConstraint33 * sqrt(strainTensor3.z)) + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_33[index]) + (gama * (expStrainGradient33Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_33 = ((1.0 + gama) * gradientLengthSum33) + alphaHat;
			float deltaLagrangeMultiplier3_33 = -deltaLagrangeMultiplierNumerator3_33 / deltaLagrangeMultiplierDenominator3_33;

			dynamicSolverData.totalLagrangeMultiplierStrain1_33[index] += deltaLagrangeMultiplier1_33;
			dynamicSolverData.totalLagrangeMultiplierStrain2_33[index] += deltaLagrangeMultiplier2_33;
			dynamicSolverData.totalLagrangeMultiplierStrain3_33[index] += deltaLagrangeMultiplier3_33;

			//Compute new vector positions for strainTensor33 stretching
			vec1 = vec1 + (expStrainGradient33Vec1 * deltaLagrangeMultiplier1_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec2 = vec2 + (expStrainGradient33Vec2 * deltaLagrangeMultiplier2_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);
			vec3 = vec3 + (expStrainGradient33Vec3 * deltaLagrangeMultiplier3_33 * oneOverVertexWeights * dynamicSolverData.strainStiffness33);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_12 = strainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]);
			//float deltaLagrangeMultiplierDenominator1_12 = gradientLengthSum12 + alphaHat;
			//deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator1_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_12[index]) + (gama * (expStrainGradient12Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier1_12 = -deltaLagrangeMultiplierNumerator1_12 / deltaLagrangeMultiplierDenominator1_12;

			float deltaLagrangeMultiplierNumerator2_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_12[index]) + (gama * (expStrainGradient12Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier2_12 = -deltaLagrangeMultiplierNumerator2_12 / deltaLagrangeMultiplierDenominator2_12;

			float deltaLagrangeMultiplierNumerator3_12 = expStrainConstraint12 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_12[index]) + (gama * (expStrainGradient12Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_12 = ((1.0 + gama) * gradientLengthSum12) + alphaHat;
			float deltaLagrangeMultiplier3_12 = -deltaLagrangeMultiplierNumerator3_12 / deltaLagrangeMultiplierDenominator3_12;

			dynamicSolverData.totalLagrangeMultiplierStrain1_12[index] += deltaLagrangeMultiplier1_12;
			dynamicSolverData.totalLagrangeMultiplierStrain2_12[index] += deltaLagrangeMultiplier2_12;
			dynamicSolverData.totalLagrangeMultiplierStrain3_12[index] += deltaLagrangeMultiplier3_12;

			//Compute new vector positions for strainTensor12 shearing
			vec1 = vec1 + (expStrainGradient12Vec1 * deltaLagrangeMultiplier1_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec2 = vec2 + (expStrainGradient12Vec2 * deltaLagrangeMultiplier2_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);
			vec3 = vec3 + (expStrainGradient12Vec3 * deltaLagrangeMultiplier3_12 * oneOverVertexWeights * dynamicSolverData.strainStiffness12);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_13 = strainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]);
			//float deltaLagrangeMultiplierDenominator1_13 = gradientLengthSum13 + alphaHat;
			//deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator1_13 = expStrainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_13[index]) + (gama * (expStrainGradient13Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier1_13 = -deltaLagrangeMultiplierNumerator1_13 / deltaLagrangeMultiplierDenominator1_13;

			float deltaLagrangeMultiplierNumerator2_13 = expStrainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_13[index]) + (gama * (expStrainGradient13Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier2_13 = -deltaLagrangeMultiplierNumerator2_13 / deltaLagrangeMultiplierDenominator2_13;

			float deltaLagrangeMultiplierNumerator3_13 = expStrainConstraint13 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_13[index]) + (gama * (expStrainGradient13Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_13 = ((1.0 + gama) * gradientLengthSum13) + alphaHat;
			float deltaLagrangeMultiplier3_13 = -deltaLagrangeMultiplierNumerator3_13 / deltaLagrangeMultiplierDenominator3_13;

			dynamicSolverData.totalLagrangeMultiplierStrain1_13[index] += deltaLagrangeMultiplier1_13;
			dynamicSolverData.totalLagrangeMultiplierStrain2_13[index] += deltaLagrangeMultiplier2_13;
			dynamicSolverData.totalLagrangeMultiplierStrain3_13[index] += deltaLagrangeMultiplier3_13;

			//Compute new vector positions for strainTensor13 shearing
			vec1 = vec1 + (expStrainGradient13Vec1 * deltaLagrangeMultiplier1_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec2 = vec2 + (expStrainGradient13Vec2 * deltaLagrangeMultiplier2_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);
			vec3 = vec3 + (expStrainGradient13Vec3 * deltaLagrangeMultiplier3_13 * oneOverVertexWeights * dynamicSolverData.strainStiffness13);

			//without damping
			//float deltaLagrangeMultiplierNumerator1_23 = strainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]);
			//float deltaLagrangeMultiplierDenominator1_23 = gradientLengthSum23 + alphaHat;
			//deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator1_23 = expStrainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain1_23[index]) + (gama * (expStrainGradient23Vec1.dot(velocity1)));
			float deltaLagrangeMultiplierDenominator1_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier1_23 = -deltaLagrangeMultiplierNumerator1_23 / deltaLagrangeMultiplierDenominator1_23;

			float deltaLagrangeMultiplierNumerator2_23 = expStrainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain2_23[index]) + (gama * (expStrainGradient23Vec2.dot(velocity2)));
			float deltaLagrangeMultiplierDenominator2_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier2_23 = -deltaLagrangeMultiplierNumerator2_23 / deltaLagrangeMultiplierDenominator2_23;

			float deltaLagrangeMultiplierNumerator3_23 = expStrainConstraint23 + (alphaHat * dynamicSolverData.totalLagrangeMultiplierStrain3_23[index]) + (gama * (expStrainGradient23Vec3.dot(velocity3)));
			float deltaLagrangeMultiplierDenominator3_23 = ((1.0 + gama) * gradientLengthSum23) + alphaHat;
			float deltaLagrangeMultiplier3_23 = -deltaLagrangeMultiplierNumerator3_23 / deltaLagrangeMultiplierDenominator3_23;

			dynamicSolverData.totalLagrangeMultiplierStrain1_23[index] += deltaLagrangeMultiplier1_23;
			dynamicSolverData.totalLagrangeMultiplierStrain2_23[index] += deltaLagrangeMultiplier2_23;
			dynamicSolverData.totalLagrangeMultiplierStrain3_23[index] += deltaLagrangeMultiplier3_23;

			//Compute new vector positions for strainTensor23 shearing
			vec1 = vec1 + (expStrainGradient23Vec1 * deltaLagrangeMultiplier1_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec2 = vec2 + (expStrainGradient23Vec2 * deltaLagrangeMultiplier2_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);
			vec3 = vec3 + (expStrainGradient23Vec3 * deltaLagrangeMultiplier3_23 * oneOverVertexWeights * dynamicSolverData.strainStiffness23);

			//set vertexPositionList
			staticSolverData.pVertexPositionList[vertexIndex1] = vec1;
			staticSolverData.pVertexPositionList[vertexIndex2] = vec2;
			staticSolverData.pVertexPositionList[vertexIndex3] = vec3;
		}
	}
}


//positionConstraint
void PHYSSolverCPU::positionConstraints(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
{	
	//check if posCons 
	if(dynamicSolverData.positionConstraintCount)
	{
		//iterate posCons
		for(int index = 0; index < dynamicSolverData.positionConstraintCount; index++)
		{
			//check if posCon active
			if(dynamicSolverData.pPositionConstraintActiveList[index])
			{
				//check if vertexIndex in vertexCount
				if(dynamicSolverData.pPositionConstraintVertexIndexList[index] < staticSolverData.vertexCount)
				{
					//set vertexPositionList
					staticSolverData.pVertexPositionList[dynamicSolverData.pPositionConstraintVertexIndexList[index]] = dynamicSolverData.pPositionConstraintCoordinateList[index];
				}
			}
		}
	}
}