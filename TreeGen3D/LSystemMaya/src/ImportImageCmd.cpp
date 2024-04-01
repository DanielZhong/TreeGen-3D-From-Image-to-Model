//#include "LSystemCmd.h"
#include "ImportImageCmd.h"
#include <maya/MGlobal.h>
#include <fstream>
#include <sstream>
#include <string>

ImportImageCmd::ImportImageCmd() {}

ImportImageCmd::~ImportImageCmd() {}

void* ImportImageCmd::creator() {
    return new ImportImageCmd();
}

MSyntax ImportImageCmd::newSyntax() {
    MSyntax syntax;
    // Define the command's syntax here. For example:
    syntax.addFlag("-p", "-path", MSyntax::kString);
    return syntax;
}

MStatus ImportImageCmd::doIt(const MArgList& args) {
    MGlobal::displayInfo("Succesfuly Called ImportImageCmd");
    MString filepath;
    for (unsigned int i = 0; i < args.length(); i++) {
        if (args.asString(i) == "-p" || args.asString(i) == "-path") {
            filepath = args.asString(++i);
            break;
        }
    }

    if (filepath.length() == 0) {
        MGlobal::displayError("No file path provided.");
        return MS::kFailure;
    }

    // Print the filepath to verify it's being passed correctly
    MString infoMsg = "Filepath provided: ";
    infoMsg += filepath;
    MGlobal::displayInfo(infoMsg);

    // Call the helper function with the file path
    return parseBoundingBoxData(filepath.asChar());
    return MS::kSuccess;
}

MStatus ImportImageCmd::parseBoundingBoxData(const std::string& filepath) {
    // Assuming the bbox file has the same base name but with ".txt" extension
    std::string bboxFilePath = filepath;
    size_t lastdot = bboxFilePath.find_last_of(".");
    if (lastdot != std::string::npos) bboxFilePath.replace(lastdot, bboxFilePath.length() - lastdot, ".txt");

    std::ifstream inFile(bboxFilePath);
    if (!inFile) {
        MGlobal::displayError("Could not open the bounding box file.");
        return MS::kFailure;
    }

    std::string line;
    while (std::getline(inFile, line)) {
        std::istringstream iss(line);
        std::string class_name;
        float score, x_c, y_c, w, h, angle;
        if (iss >> class_name >> score >> x_c >> y_c >> w >> h >> angle) {
            // Process each bounding box
            MGlobal::displayInfo(MString("Processed bbox for class: ") + class_name.c_str());
            // Here, add your logic to handle each bounding box, e.g., creating annotations in Maya
        }
    }

    inFile.close();
    return MS::kSuccess;
}