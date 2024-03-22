#define MNoVersionString
#define MNoPluginEntry

#include "LSystemNode.h"
#include "cylinder.h"
#include "LSystem.h"

#include <maya/MFnPlugin.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MTime.h>

#define McheckErr(stat,msg) if (MS::kSuccess != stat) { return MS::kFailure; }

MStatus returnStatus;

MTypeId LSystemNode::id(0x0);
MObject LSystemNode::time;
MObject LSystemNode::stepSize;
MObject LSystemNode::defaultAngle;
MObject LSystemNode::grammarFile;
MObject LSystemNode::outputMesh;

void* LSystemNode::creator() {
    return new LSystemNode();
}

MStatus LSystemNode::initialize() {
    MFnNumericAttribute numAttr;
    MFnTypedAttribute typedAttr;
    MFnUnitAttribute unitAttr;

    time = unitAttr.create("time", "tm", MFnUnitAttribute::kTime, 0.0);
    unitAttr.setKeyable(true); unitAttr.setStorable(true); unitAttr.setReadable(true); unitAttr.setWritable(true);

    defaultAngle = numAttr.create("defaultAngle", "da", MFnNumericData::kDouble, 0.0);
    numAttr.setKeyable(true); numAttr.setStorable(true); numAttr.setReadable(true); numAttr.setWritable(true);

    stepSize = numAttr.create("stepSize", "ss", MFnNumericData::kDouble, 0.0);
    numAttr.setKeyable(true); numAttr.setStorable(true); numAttr.setReadable(true); numAttr.setWritable(true);

    grammarFile = typedAttr.create("grammarFile", "g", MFnData::kString);
    typedAttr.setKeyable(true); typedAttr.setStorable(true); typedAttr.setReadable(true); typedAttr.setWritable(true);

    outputMesh = typedAttr.create("outputMesh", "out", MFnData::kMesh);
    typedAttr.setKeyable(false); typedAttr.setStorable(false); typedAttr.setReadable(true); typedAttr.setWritable(false);

    addAttribute(time); addAttribute(defaultAngle); addAttribute(stepSize); addAttribute(grammarFile); addAttribute(outputMesh);
    attributeAffects(time, outputMesh); attributeAffects(stepSize, outputMesh); attributeAffects(defaultAngle, outputMesh);  attributeAffects(grammarFile, outputMesh);

    return MS::kSuccess;
}

MStatus LSystemNode::compute(const MPlug & plug, MDataBlock & data) {
    if (plug != outputMesh) {
        return MS::kUnknownParameter;
    }

    MStatus status;

    // Retrieve the time attribute value
    MDataHandle timeDataHandle = data.inputValue(time, &status);
    McheckErr(status, "Time data handle fail\n");
    MTime timeValue = timeDataHandle.asTime();

    // Retrieve the default angle attribute value
    MDataHandle angleDataHandle = data.inputValue(defaultAngle, &status);
    McheckErr(status, "Default angle data handle fail\n");
    double defaultAngleValue = angleDataHandle.asDouble();

    // Retrieve the step size attribute value
    MDataHandle stepSizeDataHandle = data.inputValue(stepSize, &status);
    McheckErr(status, "Step size data handle fail\n");
    double stepSizeValue = stepSizeDataHandle.asDouble();

    // Retrieve the grammar file attribute value
    MDataHandle grammarFileDataHandle = data.inputValue(grammarFile, &status);
    McheckErr(status, "Grammar file data handle fail\n");
    MString grammarValue = grammarFileDataHandle.asString();

    // Proceed with computation only if all inputs are retrieved successfully
    MDataHandle outputHandle = data.outputValue(outputMesh, &status);
    McheckErr(status, "Creating new mesh fail\n");

    MFnMeshData dataCreator;
    MObject newOutputData = dataCreator.create(&status);
    McheckErr(status, "Creating new output Data fail");

    createMesh(timeValue, stepSizeValue, defaultAngleValue, grammarValue, newOutputData, status);
    McheckErr(status, "Creating new mesh fail");

    outputHandle.setMObject(newOutputData);
    data.setClean(plug);

    return MS::kSuccess;
}

MObject LSystemNode::createMesh(const MTime & time, const double& stepSize, const double& defaultAngle, const MString & grammarFileDir, MObject& outputData, MStatus& status) {
    // Init
    int numOfIterations = (int)time.as(MTime::kFilm);
    MObject pluginMObj = MFnPlugin::findPlugin("LSystem");
    MFnPlugin plugin(pluginMObj);
    MString grammarPath = plugin.loadPath() + "/" + grammarFileDir;
    LSystem sys;
    
    sys.loadProgram(grammarPath.asChar());
    sys.setDefaultStep(stepSize);
    sys.setDefaultAngle(defaultAngle);

    // Process the prework of drawing
    std::vector<LSystem::Branch> branches_vec;

    for (int i = 0; i < numOfIterations; i++)
    {
        sys.process(i, branches_vec);
    }

    MPointArray mpoints;
    MIntArray mfaceCounts, mfaceConnects;

    // Draw the final
    for (const auto& branch : branches_vec) {
        MPoint start(branch.first[0], branch.first[2], branch.first[1]);
        MPoint end(branch.second[0], branch.second[2], branch.second[1]);
        CylinderMesh cylinder(start, end);
        cylinder.appendToMesh(mpoints, mfaceCounts, mfaceConnects);
    }

    // Return the mesh
    MFnMesh mesh;
    return mesh.create(mpoints.length(), mfaceCounts.length(), mpoints, mfaceCounts, mfaceConnects, outputData, &status);
}