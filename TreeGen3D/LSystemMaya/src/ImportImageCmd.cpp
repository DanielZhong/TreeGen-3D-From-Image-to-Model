//#include "LSystemCmd.h"
#include "ImportImageCmd.h"
#include <maya/MGlobal.h>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <maya/MVector.h>
#include <cmath>
#include <queue>
#include <algorithm>
#include <iostream>
#include <iomanip>

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Helper Variable, 不知道为什么不让存到private里面，调取的时候找不到，只能放这里了。。。
std::vector<Bounding_box_parse> m_bbx_parse;
Nary_TreeNode* rootNode;
std::string m_save_image_name;
double m_min_len;
int m_root_id;


// Helper Functions
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

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Constructor & Deconstructor & Creator & Input Syntax
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

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Plugin Main Logic
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
    m_save_image_name = filepath;
    m_save_image_name.replace(m_save_image_name.end() - 4, m_save_image_name.end(), "_bbx.jpg");

    m_bbx_parse.clear();
    m_min_len = 1000;

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

            m_min_len = std::min(m_min_len, bbx.height);

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

            m_bbx_parse.push_back(bbx);

            // Process each bounding box
            MGlobal::displayInfo(MString("Processed bbox for class: ") + class_name.c_str());
        }
    }

    inFile.close();
    return MS::kSuccess;
}

//void ImportImageCmd::processBoundingBox(Bounding_box_parse& bbx) {
//    // Process and manipulate bounding box as necessary
//    // This could involve setting up relationships, computing additional properties, etc.
//}
//
//void ImportImageCmd::buildNaryTree() {
//    std::vector<Nary_TreeNode*> nodes_list;
//    std::vector<bool> visited;
//    std::vector<int> parents;
//    //build tree nodes
//    for (int i = 0; i < m_bbx_parse.size(); i++) {
//        Bounding_box_parse cur_bbx = m_bbx_parse[i];
//        Nary_TreeNode* cur_node = CreateNaryTreeNode(m_bbx_parse[i]);
//        cur_node->bbx_index = i + 1;
//        nodes_list.push_back(cur_node);
//        visited.push_back(false);
//    }
//    //consturct the tree
//    Nary_TreeNode* root_node = nodes_list[m_root_id];
//    root_node->bbx.angleFromParent = root_node->bbx.angleFromY_LC;
//    visited[m_root_id] = true;
//
//    queue<Nary_TreeNode*> qt;
//    qt.push(root_node);
//    typedef boost::property_map<UndirectedGraph, boost::vertex_index_t>::type IndexMap;
//    IndexMap index = get(boost::vertex_index, m_graph);
//
//    while (!qt.empty()) {
//        Nary_TreeNode* cur_node = qt.front();
//        qt.pop();
//        if (cur_node) {
//            UndirectedGraph::adjacency_iterator vit, vend;
//            std::vector<Nary_TreeNode*> children_list;
//            for (std::tie(vit, vend) = boost::adjacent_vertices(cur_node->bbx_index - 1, m_graph); vit != vend; ++vit) {
//                int adj_idx = index[*vit];
//                if (!visited[adj_idx]) {
//                    double p1 = nodes_list[adj_idx]->bbx.angleFromY_LC - cur_node->bbx.angleFromY_LC;
//                    double p2 = nodes_list[adj_idx]->bbx.angleFromY_LC_opp - cur_node->bbx.angleFromY_LC;
//                    nodes_list[adj_idx]->bbx.angleFromParent = abs(p1) < abs(p2) ? p1 : p2;
//                    children_list.push_back(nodes_list[adj_idx]);
//                    visited[adj_idx] = true;
//                }
//            }
//
//            std::sort(children_list.begin(), children_list.end(), Nary_GreaterSort);
//            int main_branch_idx = -1; // we find which child should be the main branch
//            double min_abs_anbgle = 10000;
//            for (int i = 0; i < children_list.size(); i++) {
//                if (abs(children_list[i]->bbx.angleFromParent) < m_mianBranchAngle_thrs) {
//                    if (abs(children_list[i]->bbx.angleFromParent) < min_abs_anbgle) {
//                        min_abs_anbgle = abs(children_list[i]->bbx.angleFromParent);
//                        main_branch_idx = i;
//                    }
//                }
//            }
//
//            for (int i = 0; i < children_list.size(); i++) {
//                if (i == main_branch_idx) {
//                    children_list[i]->main_branch = true;
//                    children_list[i]->turn_indicator = "0";
//                }
//                else if (children_list[i]->bbx.angleFromParent > 0) {
//                    children_list[i]->turn_indicator = "1";
//                }
//                else if (children_list[i]->bbx.angleFromParent < 0) {
//                    children_list[i]->turn_indicator = "-1";
//                }
//                ConnectTreeNodes(cur_node, children_list[i]);
//                qt.push(cur_node->children[i]);
//            }
//        }
//    }
//}