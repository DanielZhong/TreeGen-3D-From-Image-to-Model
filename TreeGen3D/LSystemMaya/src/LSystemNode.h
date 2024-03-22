#pragma once
#include <maya/MPxNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MTime.h>

#include "cylinder.h"
#include "LSystem.h"

class LSystemNode : public MPxNode
{
public:
    LSystemNode() {}
    virtual ~LSystemNode() {}

    static void* creator();
    static MStatus initialize();

    virtual MStatus compute(const MPlug& plug, MDataBlock& data);
    MObject createMesh(const MTime& time, const double& stepSize, const double& defaultAngle, const MString& grammar, MObject& data, MStatus& stat);
public:
    static MTypeId id;
    static MObject defaultAngle;
    static MObject stepSize;
    static MObject grammarFile;
    static MObject time;
    static MObject outputMesh;
};