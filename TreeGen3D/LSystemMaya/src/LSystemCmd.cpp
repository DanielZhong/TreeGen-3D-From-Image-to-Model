#include "LSystemCmd.h"
#include "LSystem.h"
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MGlobal.h>
#include <list>
#include <sstream>

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

	LSystem system;
	std::vector<LSystem::Branch> branches_vec;

	system.setDefaultAngle(defaultAngle);
	system.setDefaultStep(stepSize);
	system.loadProgramFromString(grammar.asChar());

	// interations for L-System
	for (int i = 0; i < numIterations; i++)
	{
		system.process(i, branches_vec);
	}

	// Use a single stringstream and string for command construction
	std::stringstream sstream;

	// Draw the branches from the final iteration
	for (const auto& branch : branches_vec) {
		vec3 start = branch.first;
		vec3 end = branch.second;

		// Construct the curve creation command
		sstream << "curve -d 1 -p " << start[0] << " " << start[2] << " " << start[1]
			<< " -p " << end[0] << " " << end[2] << " " << end[1] << " -k 0 -k 1;";

		// Construct the extrusion commands
		sstream << "circle -r 0.2 -name nurbsCircle1; "
			<< "select -r nurbsCircle1 curve1; "
			<< "extrude -ch true -rn false -po 1 -et 2 -ucp 1 -fpt 1 -upn 1 -rotation 0 -scale 1 -rsp 1 nurbsCircle1 curve1;";
	}

	// Execute the MEL commands
	MGlobal::executeCommand(sstream.str().c_str());
	return MStatus::kSuccess;
}

