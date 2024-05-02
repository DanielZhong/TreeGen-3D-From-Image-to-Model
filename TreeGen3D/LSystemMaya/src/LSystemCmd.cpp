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
	syntax.addFlag("-sc", "-scalar", MSyntax::kDouble);
	syntax.addFlag("-apb", "-anglePerturbation", MSyntax::kDouble);
	syntax.addFlag("-spb", "-stepSizePerturbation", MSyntax::kDouble);
	return syntax;
}

MStatus LSystemCmd::doIt( const MArgList& args )
{
	MStatus status;
	double stepSize = 0.0;
	double defaultAngle = 0.0;
	int numIterations = 0;
	double scalar = 1.;
	double anglePerturbation = 0;
	double stepSizePerturbation = 0;
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

	if (argData.isFlagSet("-sc"))
		argData.getFlagArgument("-sc", 0, scalar);

	if (argData.isFlagSet("-apb"))
		argData.getFlagArgument("-apb", 0, anglePerturbation);

	if (argData.isFlagSet("-spb"))
		argData.getFlagArgument("-spb", 0, stepSizePerturbation);

	ImportImageCmd imageCmd;
	LSystem system;
	std::vector<LSystem::Branch> branches_vec;

	system.setDefaultAngle(defaultAngle);
	system.setDefaultStep(stepSize);
	system.setDefaultScalar(scalar);
	system.setDefaultAPB(anglePerturbation);
	system.setDefaultSSPB(stepSizePerturbation);
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

	MDagModifier dagModifier;
	MObject lSystemGroup = dagModifier.createNode("transform", MObject::kNullObj, &status);
	if (!status) {
		status.perror("Failed to create LSystem group transform");
		return status;
	}
	dagModifier.renameNode(lSystemGroup, "LSystem");
	dagModifier.doIt();

	// Draw the branches from the final iteration
	for (auto& branch : branches_vec) {
		MPoint startPoint(branch.first[0], branch.first[2], branch.first[1]);
		MPoint endPoint(branch.second[0], branch.second[2], branch.second[1]);

		MPointArray curvePoints;
		curvePoints.append(startPoint);
		curvePoints.append(endPoint);

		MDoubleArray knotSequences;
		knotSequences.append(0.0);
		knotSequences.append(1.0);

		MDagModifier curveDagModifier;
		MObject curveTransformObj = curveDagModifier.createNode("transform", lSystemGroup, &status);  // Creating a transform node under LSystem
		if (!status) {
			status.perror("Failed to create transform for curve");
			return status;
		}
		curveDagModifier.renameNode(curveTransformObj, "curveTransform");
		curveDagModifier.doIt();

		MFnNurbsCurve curveFn;
		MObject curveShapeObj = curveFn.create(curvePoints, knotSequences, 1, MFnNurbsCurve::kOpen, false, false, curveTransformObj, &status);  // Parent the curve shape to the transform
		if (!status) {
			status.perror("Failed to create curve for branch");
			return status;
		}
	}

	// Execute the MEL commands
	MGlobal::executeCommand(sstream.str().c_str());
	return MStatus::kSuccess;
}

