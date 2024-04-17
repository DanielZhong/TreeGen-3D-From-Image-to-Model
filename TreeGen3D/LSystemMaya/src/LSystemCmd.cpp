#include "LSystemCmd.h"
#include "LSystem.h"
#include <maya/MGlobal.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MPointArray.h>
#include <maya/MDagModifier.h>
#include <maya/MFnCircleSweepManip.h> 
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <list>
#include "ImportImageCMD.h"

LSystemCmd::LSystemCmd() : MPxCommand()
{
}

LSystemCmd::~LSystemCmd() 
{
}

MSyntax LSystemCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag("-ss", "-stepsize", MSyntax::kDouble);
	syntax.addFlag("-da", "-defaultAngle", MSyntax::kDouble);
	syntax.addFlag("-g", "-grammar", MSyntax::kString);
	syntax.addFlag("-ni", "-numIterations", MSyntax::kLong);
	return syntax;
}

MStatus LSystemCmd::doIt( const MArgList& args )
{
	MStatus status;
	double stepSize = 0.0;
	double defaultAngle = 0.0;
	int numIterations = 0;
	MString grammar = "";
	MArgDatabase argData(syntax(), args);

	if (argData.isFlagSet("-ss"))
		argData.getFlagArgument("-ss", 0, stepSize);

	if (argData.isFlagSet("-da"))
		argData.getFlagArgument("-da", 0, defaultAngle);

	if (argData.isFlagSet("-g"))
		argData.getFlagArgument("-g", 0, grammar);

	if (argData.isFlagSet("-ni"))
		argData.getFlagArgument("-ni", 0, numIterations);

	ImportImageCmd imageCmd;
	LSystem system;
	std::vector<LSystem::Branch> branches_vec;

	system.setDefaultAngle(defaultAngle);
	system.setDefaultStep(stepSize); 
	system.loadProgramFromString(grammar.asChar());
	// system.loadProgramFromString(imageCmd.SpawnedGrammar.asChar());
	// MGlobal::displayInfo(imageCmd.SpawnedGrammar);

	// interations for L-System
	for (int i = 0; i < numIterations; i++)
	{
		system.process(i, branches_vec);
	}

	// Use a single stringstream and string for command construction
	std::stringstream sstream;

	// Draw the branches from the final iteration
	for (auto& branch : branches_vec) {
        // custom shape: curve
        MPoint startPoint(branch.first[0], branch.first[2], branch.first[1]);
        MPoint endPoint(branch.second[0], branch.second[2], branch.second[1]);

        MPointArray curvePoints;
        curvePoints.append(startPoint);
        curvePoints.append(endPoint);

        MDoubleArray knotSequences;
        knotSequences.append(0.0);
        knotSequences.append(1.0);

        MDagModifier dagModifier;
        MObject curveTransformObj = dagModifier.createNode("nurbsCurve");
        dagModifier.doIt();

        MFnNurbsCurve curveFn;
        curveFn.setObject(curveTransformObj);
        curveFn.create(curvePoints, knotSequences, 1, MFnNurbsCurve::kOpen, false, false, MObject::kNullObj, &status);

        if (!status) {
            status.perror("Failed to create curve for branch");
            return status;
        }
    }

	// Execute the MEL commands
	MGlobal::executeCommand(sstream.str().c_str());
	return MStatus::kSuccess;
}

