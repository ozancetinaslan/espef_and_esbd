

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
gravityPerSec = 0.0
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

#locator 0
#----------------------------------
positionConstraintLocatorIndex0Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex0')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex0' )
positionConstraintLocatorIndex0Shape = positionConstraintLocatorIndex0Trans.getShape()
vPos0 = cmds.pointPosition('mainModelShape.vtx[0]',w=True)
positionConstraintLocatorIndex0Trans.translate.set(vPos0[0], vPos0[1], vPos0[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[0].positionConstraintVertexIndex.set(0)
positionConstraintLocatorIndex0Shape.worldPosition >> PHYSSolver.positionConstraint[0].positionConstraintCoordinate
pm.select(cl = True)

#locator 1
#----------------------------------
positionConstraintLocatorIndex1Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex1')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex1' )
positionConstraintLocatorIndex1Shape = positionConstraintLocatorIndex1Trans.getShape()
vPos1 = cmds.pointPosition('mainModelShape.vtx[1]',w=True)
positionConstraintLocatorIndex1Trans.translate.set(vPos1[0], vPos1[1], vPos1[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[1].positionConstraintVertexIndex.set(1)
positionConstraintLocatorIndex1Shape.worldPosition >> PHYSSolver.positionConstraint[1].positionConstraintCoordinate
pm.select(cl = True)

#locator 2
#----------------------------------
positionConstraintLocatorIndex2Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex2')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex2' )
positionConstraintLocatorIndex2Shape = positionConstraintLocatorIndex2Trans.getShape()
vPos2 = cmds.pointPosition('mainModelShape.vtx[2]',w=True)
positionConstraintLocatorIndex2Trans.translate.set(vPos2[0], vPos2[1], vPos2[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[2].positionConstraintVertexIndex.set(2)
positionConstraintLocatorIndex2Shape.worldPosition >> PHYSSolver.positionConstraint[2].positionConstraintCoordinate
pm.select(cl = True)

#locator 3
#----------------------------------
positionConstraintLocatorIndex3Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex3')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex3' )
positionConstraintLocatorIndex3Shape = positionConstraintLocatorIndex3Trans.getShape()
vPos3 = cmds.pointPosition('mainModelShape.vtx[3]',w=True)
positionConstraintLocatorIndex3Trans.translate.set(vPos3[0], vPos3[1], vPos3[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[3].positionConstraintVertexIndex.set(3)
positionConstraintLocatorIndex3Shape.worldPosition >> PHYSSolver.positionConstraint[3].positionConstraintCoordinate
pm.select(cl = True)

#locator 4
#----------------------------------
positionConstraintLocatorIndex4Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex4')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex4' )
positionConstraintLocatorIndex4Shape = positionConstraintLocatorIndex4Trans.getShape()
vPos4 = cmds.pointPosition('mainModelShape.vtx[4]',w=True)
positionConstraintLocatorIndex4Trans.translate.set(vPos4[0], vPos4[1], vPos4[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[4].positionConstraintVertexIndex.set(4)
positionConstraintLocatorIndex4Shape.worldPosition >> PHYSSolver.positionConstraint[4].positionConstraintCoordinate
pm.select(cl = True)

#locator 5
#----------------------------------
positionConstraintLocatorIndex5Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex5')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex5' )
positionConstraintLocatorIndex5Shape = positionConstraintLocatorIndex5Trans.getShape()
vPos5 = cmds.pointPosition('mainModelShape.vtx[5]',w=True)
positionConstraintLocatorIndex5Trans.translate.set(vPos5[0], vPos5[1], vPos5[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[5].positionConstraintVertexIndex.set(5)
positionConstraintLocatorIndex5Shape.worldPosition >> PHYSSolver.positionConstraint[5].positionConstraintCoordinate
pm.select(cl = True)

#locator 6
#----------------------------------
positionConstraintLocatorIndex6Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex6')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex6' )
positionConstraintLocatorIndex6Shape = positionConstraintLocatorIndex6Trans.getShape()
vPos6 = cmds.pointPosition('mainModelShape.vtx[52]',w=True)
positionConstraintLocatorIndex6Trans.translate.set(vPos6[0], vPos6[1], vPos6[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[6].positionConstraintVertexIndex.set(52)
positionConstraintLocatorIndex6Shape.worldPosition >> PHYSSolver.positionConstraint[6].positionConstraintCoordinate
pm.select(cl = True)

#locator 7
#----------------------------------
positionConstraintLocatorIndex7Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex7')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex7' )
positionConstraintLocatorIndex7Shape = positionConstraintLocatorIndex7Trans.getShape()
vPos7 = cmds.pointPosition('mainModelShape.vtx[53]',w=True)
positionConstraintLocatorIndex7Trans.translate.set(vPos7[0], vPos7[1], vPos7[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[7].positionConstraintVertexIndex.set(53)
positionConstraintLocatorIndex7Shape.worldPosition >> PHYSSolver.positionConstraint[7].positionConstraintCoordinate
pm.select(cl = True)

#locator 8
#----------------------------------
positionConstraintLocatorIndex8Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex8')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex8' )
positionConstraintLocatorIndex8Shape = positionConstraintLocatorIndex8Trans.getShape()
vPos8 = cmds.pointPosition('mainModelShape.vtx[54]',w=True)
positionConstraintLocatorIndex8Trans.translate.set(vPos8[0], vPos8[1], vPos8[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[8].positionConstraintVertexIndex.set(54)
positionConstraintLocatorIndex8Shape.worldPosition >> PHYSSolver.positionConstraint[8].positionConstraintCoordinate
pm.select(cl = True)

#locator 9
#----------------------------------
positionConstraintLocatorIndex9Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex9')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex9' )
positionConstraintLocatorIndex9Shape = positionConstraintLocatorIndex9Trans.getShape()
vPos9 = cmds.pointPosition('mainModelShape.vtx[55]',w=True)
positionConstraintLocatorIndex9Trans.translate.set(vPos9[0], vPos9[1], vPos9[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[9].positionConstraintVertexIndex.set(55)
positionConstraintLocatorIndex9Shape.worldPosition >> PHYSSolver.positionConstraint[9].positionConstraintCoordinate
pm.select(cl = True)

#locator 10
#----------------------------------
positionConstraintLocatorIndex10Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex10')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex10' )
positionConstraintLocatorIndex10Shape = positionConstraintLocatorIndex10Trans.getShape()
vPos10 = cmds.pointPosition('mainModelShape.vtx[56]',w=True)
positionConstraintLocatorIndex10Trans.translate.set(vPos10[0], vPos10[1], vPos10[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[10].positionConstraintVertexIndex.set(56)
positionConstraintLocatorIndex10Shape.worldPosition >> PHYSSolver.positionConstraint[10].positionConstraintCoordinate
pm.select(cl = True)

#locator 11
#----------------------------------
positionConstraintLocatorIndex11Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex11')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex11' )
positionConstraintLocatorIndex11Shape = positionConstraintLocatorIndex11Trans.getShape()
vPos11 = cmds.pointPosition('mainModelShape.vtx[57]',w=True)
positionConstraintLocatorIndex11Trans.translate.set(vPos11[0], vPos11[1], vPos11[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[11].positionConstraintVertexIndex.set(57)
positionConstraintLocatorIndex11Shape.worldPosition >> PHYSSolver.positionConstraint[11].positionConstraintCoordinate
pm.select(cl = True)

#locator 12
#----------------------------------
positionConstraintLocatorIndex12Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex12')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex12' )
positionConstraintLocatorIndex12Shape = positionConstraintLocatorIndex12Trans.getShape()
vPos12 = cmds.pointPosition('mainModelShape.vtx[104]',w=True)
positionConstraintLocatorIndex12Trans.translate.set(vPos12[0], vPos12[1], vPos12[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[12].positionConstraintVertexIndex.set(104)
positionConstraintLocatorIndex12Shape.worldPosition >> PHYSSolver.positionConstraint[12].positionConstraintCoordinate
pm.select(cl = True)

#locator 13
#----------------------------------
positionConstraintLocatorIndex13Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex13')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex13' )
positionConstraintLocatorIndex13Shape = positionConstraintLocatorIndex13Trans.getShape()
vPos13 = cmds.pointPosition('mainModelShape.vtx[105]',w=True)
positionConstraintLocatorIndex13Trans.translate.set(vPos13[0], vPos13[1], vPos13[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[13].positionConstraintVertexIndex.set(105)
positionConstraintLocatorIndex13Shape.worldPosition >> PHYSSolver.positionConstraint[13].positionConstraintCoordinate
pm.select(cl = True)

#locator 14
#----------------------------------
positionConstraintLocatorIndex14Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex14')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex14' )
positionConstraintLocatorIndex14Shape = positionConstraintLocatorIndex14Trans.getShape()
vPos14 = cmds.pointPosition('mainModelShape.vtx[106]',w=True)
positionConstraintLocatorIndex14Trans.translate.set(vPos14[0], vPos14[1], vPos14[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[14].positionConstraintVertexIndex.set(106)
positionConstraintLocatorIndex14Shape.worldPosition >> PHYSSolver.positionConstraint[14].positionConstraintCoordinate
pm.select(cl = True)

#locator 15
#----------------------------------
positionConstraintLocatorIndex15Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex15')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex15' )
positionConstraintLocatorIndex15Shape = positionConstraintLocatorIndex15Trans.getShape()
vPos15 = cmds.pointPosition('mainModelShape.vtx[107]',w=True)
positionConstraintLocatorIndex15Trans.translate.set(vPos15[0], vPos15[1], vPos15[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[15].positionConstraintVertexIndex.set(107)
positionConstraintLocatorIndex15Shape.worldPosition >> PHYSSolver.positionConstraint[15].positionConstraintCoordinate
pm.select(cl = True)

#locator 16
#----------------------------------
positionConstraintLocatorIndex16Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex16')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex16' )
positionConstraintLocatorIndex16Shape = positionConstraintLocatorIndex16Trans.getShape()
vPos16 = cmds.pointPosition('mainModelShape.vtx[108]',w=True)
positionConstraintLocatorIndex16Trans.translate.set(vPos16[0], vPos16[1], vPos16[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[16].positionConstraintVertexIndex.set(108)
positionConstraintLocatorIndex16Shape.worldPosition >> PHYSSolver.positionConstraint[16].positionConstraintCoordinate
pm.select(cl = True)

#locator 17
#----------------------------------
positionConstraintLocatorIndex17Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex17')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex17' )
positionConstraintLocatorIndex17Shape = positionConstraintLocatorIndex17Trans.getShape()
vPos17 = cmds.pointPosition('mainModelShape.vtx[109]',w=True)
positionConstraintLocatorIndex17Trans.translate.set(vPos17[0], vPos17[1], vPos17[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[17].positionConstraintVertexIndex.set(109)
positionConstraintLocatorIndex17Shape.worldPosition >> PHYSSolver.positionConstraint[17].positionConstraintCoordinate
pm.select(cl = True)

#locator 18
#----------------------------------
positionConstraintLocatorIndex18Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex18')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex18' )
positionConstraintLocatorIndex18Shape = positionConstraintLocatorIndex18Trans.getShape()
vPos18 = cmds.pointPosition('mainModelShape.vtx[156]',w=True)
positionConstraintLocatorIndex18Trans.translate.set(vPos18[0], vPos18[1], vPos18[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[18].positionConstraintVertexIndex.set(156)
positionConstraintLocatorIndex18Shape.worldPosition >> PHYSSolver.positionConstraint[18].positionConstraintCoordinate
pm.select(cl = True)

#locator 19
#----------------------------------
positionConstraintLocatorIndex19Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex19')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex19' )
positionConstraintLocatorIndex19Shape = positionConstraintLocatorIndex19Trans.getShape()
vPos19 = cmds.pointPosition('mainModelShape.vtx[157]',w=True)
positionConstraintLocatorIndex19Trans.translate.set(vPos19[0], vPos19[1], vPos19[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[19].positionConstraintVertexIndex.set(157)
positionConstraintLocatorIndex19Shape.worldPosition >> PHYSSolver.positionConstraint[19].positionConstraintCoordinate
pm.select(cl = True)

#locator 20
#----------------------------------
positionConstraintLocatorIndex20Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex20')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex20' )
positionConstraintLocatorIndex20Shape = positionConstraintLocatorIndex20Trans.getShape()
vPos20 = cmds.pointPosition('mainModelShape.vtx[158]',w=True)
positionConstraintLocatorIndex20Trans.translate.set(vPos20[0], vPos20[1], vPos20[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[20].positionConstraintVertexIndex.set(158)
positionConstraintLocatorIndex20Shape.worldPosition >> PHYSSolver.positionConstraint[20].positionConstraintCoordinate
pm.select(cl = True)

#locator 21
#----------------------------------
positionConstraintLocatorIndex21Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex21')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex21' )
positionConstraintLocatorIndex21Shape = positionConstraintLocatorIndex21Trans.getShape()
vPos21 = cmds.pointPosition('mainModelShape.vtx[159]',w=True)
positionConstraintLocatorIndex21Trans.translate.set(vPos21[0], vPos21[1], vPos21[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[21].positionConstraintVertexIndex.set(159)
positionConstraintLocatorIndex21Shape.worldPosition >> PHYSSolver.positionConstraint[21].positionConstraintCoordinate
pm.select(cl = True)

#locator 22
#----------------------------------
positionConstraintLocatorIndex22Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex22')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex22' )
positionConstraintLocatorIndex22Shape = positionConstraintLocatorIndex22Trans.getShape()
vPos22 = cmds.pointPosition('mainModelShape.vtx[160]',w=True)
positionConstraintLocatorIndex22Trans.translate.set(vPos22[0], vPos22[1], vPos22[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[22].positionConstraintVertexIndex.set(160)
positionConstraintLocatorIndex22Shape.worldPosition >> PHYSSolver.positionConstraint[22].positionConstraintCoordinate
pm.select(cl = True)

#locator 23
#----------------------------------
positionConstraintLocatorIndex23Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex23')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex23' )
positionConstraintLocatorIndex23Shape = positionConstraintLocatorIndex23Trans.getShape()
vPos23 = cmds.pointPosition('mainModelShape.vtx[161]',w=True)
positionConstraintLocatorIndex23Trans.translate.set(vPos23[0], vPos23[1], vPos23[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[23].positionConstraintVertexIndex.set(161)
positionConstraintLocatorIndex23Shape.worldPosition >> PHYSSolver.positionConstraint[23].positionConstraintCoordinate
pm.select(cl = True)

#locator 24
#----------------------------------
positionConstraintLocatorIndex24Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex24')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex24' )
positionConstraintLocatorIndex24Shape = positionConstraintLocatorIndex24Trans.getShape()
vPos24 = cmds.pointPosition('mainModelShape.vtx[144]',w=True)
positionConstraintLocatorIndex24Trans.translate.set(vPos24[0], vPos24[1], vPos24[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[24].positionConstraintVertexIndex.set(144)
positionConstraintLocatorIndex24Shape.worldPosition >> PHYSSolver.positionConstraint[24].positionConstraintCoordinate
pm.select(cl = True)

#locator 25
#----------------------------------
positionConstraintLocatorIndex25Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex25')
cmds.scale( 0.0001, 0.0001, 0.0001, 'positionConstraintLocatorIndex25' )
positionConstraintLocatorIndex25Shape = positionConstraintLocatorIndex25Trans.getShape()
vPos25 = cmds.pointPosition('mainModelShape.vtx[146]',w=True)
positionConstraintLocatorIndex25Trans.translate.set(vPos25[0], vPos25[1], vPos25[2])
pm.select(cl = True)

#connect locators and set index attrs on PHYSSolver
#----------------------------------
PHYSSolver.positionConstraint[25].positionConstraintVertexIndex.set(146)
positionConstraintLocatorIndex25Shape.worldPosition >> PHYSSolver.positionConstraint[25].positionConstraintCoordinate
pm.select(cl = True)

#set PHYSSolver active attr
#----------------------------------
PHYSSolver.positionConstraint[0].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[1].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[2].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[3].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[4].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[5].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[6].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[7].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[8].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[9].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[10].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[11].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[12].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[13].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[14].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[15].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[16].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[17].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[18].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[19].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[20].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[21].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[22].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[23].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[24].positionConstraintActive.set(1)
PHYSSolver.positionConstraint[25].positionConstraintActive.set(1)

#collision objects
#----------------------------------
CollisionCon = ['pSphere1']

objCol = pm.ls(CollisionCon)

#colision body 0
#----------------------------------
collisionSphereTrans0 = objCol[0]
collisionSphereShape0 = collisionSphereTrans0.getShape()

#connect body 0 as collision object
#----------------------------------
PHYSSolver.collisionConstraint[0].collisionConstraintActive.set(1)
PHYSSolver.collisionConstraint[0].collisionConstraintType.set(0)
collisionSphereShape0.outMesh >>PHYSSolver.collisionConstraint[0].collisionConstraintGeo
collisionSphereShape0.parentMatrix >> PHYSSolver.collisionConstraint[0].collisionConstraintGeoMatrix


#adjust timeline
#----------------------------------
pm.playbackOptions(ast = 1, aet = 200, min = 1, max = 200)

#select solver
#----------------------------------
pm.select(cl = True)
pm.select(PHYSSolver, r = True)