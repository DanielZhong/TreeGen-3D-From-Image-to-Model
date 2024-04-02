//#include "LSystemCmd.h"
#include "ImportImageCmd.h"
#include "tree_structure.h"
#include <maya/MGlobal.h>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <maya/MVector.h>
#include <cmath>

inline double angleFromY_LC(R2Vector dir) // anticlockwise is positive
{
    dir.Normalize();
    float fAngle = acos(dir.Y());
    /*if (dir.X() < 0)
    {
    fAngle = 2 * M_PI - fAngle;
    }
    return -fAngle;*/

    if (dir.X() > 0)
    {
        fAngle = -fAngle;
    }
    return fAngle;
}

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

inline int get_class_id(const std::string& name) {
    static const std::map<std::string, int> class_ids = {
        {"b60", 0},
        {"b1", 1},
        {"b2", 2},
        {"b3", 3},
        {"b4", 4},
        {"b5", 5},
        {"b6", 6},
        {"b7", 7},
        {"b8", 8}
    };

    auto it = class_ids.find(name);
    if (it != class_ids.end()) {
        return it->second;
    }
    else {
        return -1;
    }
}

inline MVector my_rotate(const MVector& s, double angleD) {
    double angle = angleD * M_PI / 180.0;
    double xv = s.x * cos(angle) + s.y * sin(angle);
    double yv = -s.x * sin(angle) + s.y * cos(angle);
    MVector t(xv, yv, 0.0);
    return t;
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
            Bounding_box_parse bbx;
            bbx.center_position = R2Vector(x_c, y_c);
            bbx.label_id = get_class_id(class_name);

            // Adjust width, height, and angle based on orientation
            if (w > h) {
                bbx.width = h;
                bbx.height = w;
                bbx.angleFromY = -angle + 90;  // Adjusting angle based on orientation
            }
            else {
                bbx.width = w;
                bbx.height = h;
                bbx.angleFromY = -angle;  // No additional rotation needed
            }

            MVector center(x_c, y_c, 0.0);
            MVector ltc(-bbx.width * 0.5, -bbx.height * 0.5, 0.0);
            MVector rtc(bbx.width * 0.5, -bbx.height * 0.5, 0.0);
            MVector rbc(bbx.width * 0.5, bbx.height * 0.5, 0.0);
            MVector lbc(-bbx.width * 0.5, bbx.height * 0.5, 0.0);
            MVector dir(0.0, 1.0, 0.0);

            ltc = my_rotate(ltc, angle);
            rtc = my_rotate(rtc, angle);
            rbc = my_rotate(rbc, angle);
            lbc = my_rotate(lbc, angle);
            dir = my_rotate(dir, angle);

            bbx.l_t_corner = R2Vector(ltc[0], ltc[1]);
            bbx.r_t_corner = R2Vector(rtc[0], rtc[1]);
            bbx.r_b_corner = R2Vector(rbc[0], rbc[1]);
            bbx.l_b_corner = R2Vector(lbc[0], lbc[1]);
            bbx.direction = R2Vector(dir[0], dir[1]);
            bbx.direction_opp = R2Vector(-dir[0], -dir[1]);

            bbx.center_position_LC = R2Vector(bbx.center_position.X(), -bbx.center_position.Y());
            bbx.l_t_corner_LC = R2Vector(bbx.l_t_corner.X(), -bbx.l_t_corner.Y());
            bbx.r_t_corner_LC = R2Vector(bbx.r_t_corner.X(), -bbx.r_t_corner.Y());
            bbx.r_b_corner_LC = R2Vector(bbx.r_b_corner.X(), -bbx.r_b_corner.Y());
            bbx.l_b_corner_LC = R2Vector(bbx.l_b_corner.X(), -bbx.l_b_corner.Y());
            bbx.direction_LC = R2Vector(bbx.direction.X(), -bbx.direction.Y());
            bbx.direction_LC_opp = R2Vector(bbx.direction_opp.X(), -bbx.direction_opp.Y());
            //bbx.angleFromY_LC = std::min(0.0, std::abs(RAD2DEG(angleFromY_LC(bbx.direction_LC))));
            bbx.angleFromY_LC = RAD2DEG(angleFromY_LC(bbx.direction_LC));
            bbx.angleFromY_LC_opp = RAD2DEG(angleFromY_LC(bbx.direction_LC_opp));

            // Process each bounding box
            MGlobal::displayInfo(MString("Processed bbox for class: ") + class_name.c_str());
        }
    }

    inFile.close();
    return MS::kSuccess;
}