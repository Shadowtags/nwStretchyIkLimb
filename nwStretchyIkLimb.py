# Enables a quick and easy toggable stretchy solution for a selected joint chain.
# Select root of chain and execute script
#
# import nwCreateStretchyIkLimb as stretchy
# reload(stretchy)
# stretchy.createStretchyLimb()

import pymel.core as pm

def createStretchyLimb():
    sel = []
    control = ""
    originalJoints = []
    measureJoints = []
    distanceNodes = []

    # get selected nodes
    sel = pm.ls(sl = True)

    if len(sel) < 2:
        pm.warning("Not enough node were selected. The script needs the root joint and the Ik control to be selected.")

    else:
        # make sure selection order is correct
        if not pm.objectType(sel[0], isType = "joint"):
            tmp = sel[0]
            sel[1] = sel[0]
            sel[0] = tmp

        # store original joint chain
        originalJoints = pm.ls(sel[0], dag = True)

        if len(originalJoints) > 3:
            pm.warning("Script does not support joint chains larger than three(3) joints. Cancelling operation.")

        else:
            # duplicate joint chain, used to measure original length
            dupl = pm.duplicate(sel[0])[0]
            control = sel[1]
            measureLocator = pm.spaceLocator()
            pm.addAttr(attributeType = "float", keyable = True, longName = "distance", niceName = "Distance")

            measureJoints = pm.ls(dupl, dag = True)

            # rename duplicate joint chain
            for i, obj in enumerate(measureJoints):
                if i == 0:
                    pm.rename(obj, "msr_%s" %sel[0])
                else:
                    pm.rename(obj, "msr_%s" %obj.partition("|")[2])

            # place and rename locator
            pm.xform(measureLocator, ws = True, translation = pm.xform(measureJoints[len(measureJoints) - 1], query = True, worldSpace = True, translation = True))
            pm.rename(measureLocator, "%s_stretch_loc" %measureJoints[len(measureJoints) - 1])

            # setup nodes measuring joint chain
            for i in range(len(measureJoints) - 1):
                distanceNodes.append(distanceBetweenNodes(measureJoints[i], measureJoints[i+1]))

            distanceNodes.append(distanceBetweenNodes(measureJoints[0], measureLocator))

            addDist = pm.shadingNode("addDoubleLinear", asUtility = True, name = "%s_To_%s_add" %(measureJoints[0], measureJoints[len(measureJoints) - 1]))
            pm.connectAttr("%s.distance" %distanceNodes[0], "%s.input1" %addDist)
            pm.connectAttr("%s.distance" %distanceNodes[1], "%s.input2" %addDist)

            # connect arm distance to locator
            pm.connectAttr("%s.distance" %distanceNodes[2], "%s.distance" %measureLocator)

            # create utility nodes for final stretching
            normalizeNode = pm.shadingNode("multiplyDivide", asUtility = True, name = "%s_normalize" %measureJoints[0])
            stretchNode = pm.shadingNode("multiplyDivide", asUtility = True, name = "%s_stretch" %measureJoints[0])
            stretchLimitNode = pm.shadingNode("clamp", asUtility = True, name = "%s_limits" %measureJoints[0])

            # normalize length
            pm.connectAttr("%s.distance" %distanceNodes[2], "%s.input1X" %normalizeNode)
            pm.connectAttr("%s.output" %addDist, "%s.input2X" %normalizeNode)
            pm.setAttr("%s.operation" %normalizeNode, 2)

            # setup stretch limit
            pm.connectAttr("%s.outputX" %normalizeNode, "%s.inputR" %stretchLimitNode)
            pm.setAttr("%s.minR" %stretchLimitNode, 1)
            pm.setAttr("%s.maxR" %stretchLimitNode, 3)

            # setup stretch node
            pm.connectAttr("%s.outputR" %stretchLimitNode, "%s.input1X" %stretchNode)
            pm.connectAttr("%s.outputR" %stretchLimitNode, "%s.input1Y" %stretchNode)
            pm.connectAttr("%s.outputX" %stretchNode, "%s.translateX" %originalJoints[1])
            pm.connectAttr("%s.outputY" %stretchNode, "%s.translateX" %originalJoints[2])
            pm.setAttr("%s.input2X" %stretchNode, pm.xform(measureJoints[1], query = True, translation = True)[0])
            pm.setAttr("%s.input2Y" %stretchNode, pm.xform(measureJoints[2], query = True, translation = True)[0])

            # constrain locator to animation control and setup toggable stretch
            pm.pointConstraint(control, measureLocator, maintainOffset = True)
            pm.setKeyframe(measureLocator, attribute = ["translateX", "translateY", "translateZ"])

            pm.select(control, replace = True)
            pm.addAttr(attributeType = "float", keyable = True, longName = "stretch", niceName = "Stretch", defaultValue = 1.0, minValue = 0.0, maxValue = 1.0)

            pm.connectAttr("%s.stretch" %control, "%s.blendPoint1" %measureLocator)

            # create Ik solver
            ikHandle = pm.ikHandle(solver = "ikRPsolver", startJoint = originalJoints[0], endEffector = originalJoints[2])[0]
            pm.rename(ikHandle, "Ik_%s_handle" %originalJoints[0].partition("_")[2])

            pm.parent(ikHandle, control)

            # clean up scene
            for node in [ikHandle, measureLocator, measureJoints[0]]:
                pm.setAttr("%s.visibility" %node, 0)

            pm.select(control, replace = True)



def distanceBetweenNodes(node1, node2):
    distNode = pm.shadingNode("distanceBetween", asUtility = True, name = "%s_To_%s_dist" %(node1.partition("_")[2], node2.partition("_")[2]))

    pm.connectAttr("%s.worldMatrix" %node1, "%s.inMatrix1" %distNode)
    pm.connectAttr("%s.worldMatrix" %node2, "%s.inMatrix2" %distNode)
    pm.connectAttr("%s.rotatePivotTranslate" %node1, "%s.point1" %distNode)
    pm.connectAttr("%s.rotatePivotTranslate" %node2, "%s.point2" %distNode)

    return distNode
