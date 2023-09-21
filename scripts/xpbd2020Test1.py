

#Test Fixture for xpbd

#procedure
import maya.cmds as cmds
import pymel.core as pm

pm.loadPlugin( 'xpbd2020.mll' )

#create outputGeo
#----------------------------------
#outputGeo
obj = pm.ls('mainModel')
outputGeoTrans = obj[0]
outputGeoShape = outputGeoTrans.getShape()
pm.select(cl = True)

#create PHYS solver node
#----------------------------------
PHYSSolver = pm.createNode('PHYSSolver')
PHYSSolver.repetitions.set(200)
pm.select(cl = True)

#set PHYSSolver gravity
#----------------------------------
gravityPerSec = -9.8
PHYSSolver.gravity.set(0,gravityPerSec,0)

#set PHYSSolver wind direction
#----------------------------------
PHYSSolver.windDirection.set(0,0,0)

#make connections
#----------------------------------
#time
timeNode = pm.PyNode('time1')
pm.select(cl = True)
timeNode.outTime >> PHYSSolver.currentFrame

#outputGeo
outputGeoShape.outMesh >> PHYSSolver.inputGeo

PHYSSolver.outputGeo >> outputGeoShape.inMesh


#create positionConstraintLocators
#----------------------------------
positionConstraintLocatorIndex0Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex0')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex0' )
positionConstraintLocatorIndex0Shape = positionConstraintLocatorIndex0Trans.getShape()
#vPos0 = cmds.pointPosition('mainModelShape.vtx[930]',w=True) 
vPos0 = cmds.pointPosition('mainModelShape.vtx[420]',w=True)
positionConstraintLocatorIndex0Trans.translate.set(vPos0[0], vPos0[1], vPos0[2])
pm.select(cl = True)

positionConstraintLocatorIndex1Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex1')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex1' )
positionConstraintLocatorIndex1Shape = positionConstraintLocatorIndex1Trans.getShape()
#vPos1 = cmds.pointPosition('mainModelShape.vtx[960]',w=True) 
vPos1 = cmds.pointPosition('mainModelShape.vtx[440]',w=True)
positionConstraintLocatorIndex1Trans.translate.set(vPos1[0], vPos1[1], vPos1[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
#PHYSSolver.positionConstraint[0].positionConstraintVertexIndex.set(930) 
PHYSSolver.positionConstraint[0].positionConstraintVertexIndex.set(420)
positionConstraintLocatorIndex0Shape.worldPosition >> PHYSSolver.positionConstraint[0].positionConstraintCoordinate
pm.select(cl = True)
#PHYSSolver.positionConstraint[1].positionConstraintVertexIndex.set(960) 
PHYSSolver.positionConstraint[1].positionConstraintVertexIndex.set(440)
positionConstraintLocatorIndex1Shape.worldPosition >> PHYSSolver.positionConstraint[1].positionConstraintCoordinate
pm.select(cl = True)

#set PHYSSolver active attr
#----------------------------------
PHYSSolver.positionConstraint[0].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[1].positionConstraintActive.set(1)


#adjust timeline
#----------------------------------
pm.playbackOptions(ast = 1, aet = 800, min = 1, max = 800)

#select solver
#----------------------------------
pm.select(cl = True)
pm.select(PHYSSolver, r = True)