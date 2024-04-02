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
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <vector>
#include <limits>

//#include <boost/graph/breadth_first_search.hpp>
//#include <boost/graph/dijkstra_shortest_paths.hpp>
//#include <boost/property_map/property_map.hpp>
//#include <boost/graph/kruskal_min_spanning_tree.hpp>

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Helper Variable, 不知道为什么不让存到private里面，调取的时候找不到，只能放这里了。。。
std::vector<Bounding_box_parse> m_bbx_parse;
Nary_TreeNode* rootNode = NULL; // Initialize pointers to NULL for safety
std::string m_save_image_name;
double m_min_len;
int m_root_id;
UndirectedGraph m_graph;

bool m_show_tree = true;
bool m_show_tree_render = false;
bool m_render = false;
int m_image_id = 0;
bool m_show_image = false;
bool m_showSampledPoints = false;
double m_line_thickness = 2.0;
double m_rotate_angle = 0.0;
double scale_factor_ = 14.0; // Ensure consistency in naming convention (e.g., m_scaleFactor)
double m_scale = 1.0;
double m_center_x = 0.0;
double m_center_y = 0.0;
int m_bbxID = -1;
double m_weight_thrs = 1.0;
double m_centerDis_thrs = 1.0; // Value retained as specified
double m_cornerDis_thrs = 0.1;
double m_mianBranchAngle_thrs = 5.0; // Value retained as specified, but consider correcting the spelling to "m_mainBranchAngle_thrs"
std::string m_optAlgorithm_ = "Our";


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

bool inline Nary_GreaterSort(Nary_TreeNode* a, Nary_TreeNode* b) {
    return (a->bbx.angleFromParent > b->bbx.angleFromParent);
}

void Nary_print_tree(Nary_TreeNode* root, int spaces) {
    int loop;
    if (root != NULL) {
        MString spaceStr;

        for (loop = 1; loop <= spaces; loop++) {
            spaceStr += " ";
        }

        MGlobal::displayInfo(spaceStr + root->bbx_index);
    }

    int main_branch_idx = -1;
    for (unsigned int i = 0; i < root->children.size(); i++) {
        if (root->children[i]->main_branch) {
            main_branch_idx = i;
            continue;
        }
        Nary_print_tree(root->children[i], spaces + 4);
    }
    if (main_branch_idx != -1) {
        Nary_print_tree(root->children[main_branch_idx], spaces + 4);
    }
}

struct pairwise_bbx {
    int i, j;
    double weight;
    pairwise_bbx(int x, int y, double dis) : weight(dis) {
        i = (x < y) ? x : y;
        j = (x < y) ? y : x;
    }
    bool operator==(const pairwise_bbx b) const {
        return (i == b.i && j == b.j);
    }

    bool operator <(const pairwise_bbx& b) const {
        bool flag = (i < b.i) || (!(i < b.i) && (j < b.j));
        return flag;
    }

    bool operator () (const pairwise_bbx& b) const {
        return (i == b.i && j == b.j);
    }
};

bool check_bbox_intersect(Bounding_box_parse b1, Bounding_box_parse b2) {
    bool is_intersect = false;

    // first, do a very fast check to remove most of the cases
    double distance = (b1.center_position_LC - b2.center_position_LC).Length();
    if (distance > 2.3 * b1.height && distance > 2.3 * b2.height) {
        return false;
    }

    //second, hadnle the tangency case
    t_line b1_lines[4] = { t_line(b1.l_t_corner_LC, b1.r_t_corner_LC), t_line{b1.r_t_corner_LC, b1.r_b_corner_LC},
                           t_line(b1.r_b_corner_LC, b1.l_b_corner_LC), t_line(b1.l_b_corner_LC, b1.l_t_corner_LC) };
    t_line b2_lines[4] = { t_line(b2.l_t_corner_LC, b2.r_t_corner_LC), t_line{b2.r_t_corner_LC, b2.r_b_corner_LC },
                           t_line(b2.r_b_corner_LC, b2.l_b_corner_LC), t_line(b2.l_b_corner_LC, b2.l_t_corner_LC) };
    for (int i = 0; i < 4; i++) {
        t_line e1 = b1_lines[i];
        for (int j = 0; j < 4; j++) {
            t_line e2 = b2_lines[j];
            if (((e1.p_start - e2.p_start).Length() < 0.15 * b1.width && (e1.p_end - e2.p_end).Length() < 0.15 * b1.width) ||
                ((e1.p_start - e2.p_end).Length() < 0.15 * b1.width && (e1.p_end - e2.p_start).Length() < 0.15 * b1.width)) {
                return true;
            }
            R2Vector mid1 = (e1.p_start + e1.p_end) * 0.5;
            R2Vector mid2 = (e2.p_start + e2.p_end) * 0.5;
            if ((mid1 - mid2).Length() < 0.15 * b1.width) {
                return true;
            }
        }
    }

    // Convert Bounding_box_parse corners to MPoint vectors for polygon representation
    std::vector<MPoint> polygonA = {
        MPoint(b1.l_t_corner_LC.X(), b1.l_t_corner_LC.Y(), 0.0),
        MPoint(b1.r_t_corner_LC.X(), b1.r_t_corner_LC.Y(), 0.0),
        MPoint(b1.r_b_corner_LC.X(), b1.r_b_corner_LC.Y(), 0.0),
        MPoint(b1.l_b_corner_LC.X(), b1.l_b_corner_LC.Y(), 0.0)
    };

    std::vector<MPoint> polygonB = {
        MPoint(b2.l_t_corner_LC.X(), b2.l_t_corner_LC.Y(), 0.0),
        MPoint(b2.r_t_corner_LC.X(), b2.r_t_corner_LC.Y(), 0.0),
        MPoint(b2.r_b_corner_LC.X(), b2.r_b_corner_LC.Y(), 0.0),
        MPoint(b2.l_b_corner_LC.X(), b2.l_b_corner_LC.Y(), 0.0)
    };

    // Separating Axis Theorem (SAT) for polygon intersection
    for (int polyi = 0; polyi < 2; ++polyi) {
        const auto& polygon = polyi == 0 ? polygonA : polygonB;

        for (size_t i1 = 0; i1 < polygon.size(); ++i1) {
            size_t i2 = (i1 + 1) % polygon.size();

            MVector normal(polygon[i2].y - polygon[i1].y, polygon[i1].x - polygon[i2].x, 0.0);

            double minA = std::numeric_limits<double>::max();
            double maxA = -std::numeric_limits<double>::max();
            for (const auto& point : polygonA) {
                double projected = (point.x * normal.x + point.y * normal.y);
                minA = std::min(minA, projected);
                maxA = std::max(maxA, projected);
            }

            double minB = std::numeric_limits<double>::max();
            double maxB = -std::numeric_limits<double>::max();
            for (const auto& point : polygonB) {
                double projected = (point.x * normal.x + point.y * normal.y);
                minB = std::min(minB, projected);
                maxB = std::max(maxB, projected);
            }

            if (maxA < minB || maxB < minA)
                return false; // No overlap found on this axis
        }
    }

    return true; // Overlap found on all axes, polygons intersect
}

double compute_relative_distance(Bounding_box_parse b1, Bounding_box_parse b2) {

    double pair_weight = 100;
    double max_height = (b1.height > b2.height) ? b1.height : b2.height;
    // first, do a very fast check to remove most of the cases
    double cc_distance = (b1.center_position_LC - b2.center_position_LC).Length();
    if (cc_distance > m_centerDis_thrs * max_height) {
        //if (cc_distance > (1.0 * (b1.height + b2.height))){
        return pair_weight;
    }

    //second, do another check by using bbx corners to remove some cases
    bool intersect = check_bbox_intersect(b1, b2);
    if (!intersect) {
        t_line b1_lines[4] = { t_line(b1.l_t_corner_LC, b1.r_t_corner_LC), t_line{ b1.r_t_corner_LC, b1.r_b_corner_LC },
            t_line(b1.r_b_corner_LC, b1.l_b_corner_LC), t_line(b1.l_b_corner_LC, b1.l_t_corner_LC) };
        t_line b2_lines[4] = { t_line(b2.l_t_corner_LC, b2.r_t_corner_LC), t_line{ b2.r_t_corner_LC, b2.r_b_corner_LC },
            t_line(b2.r_b_corner_LC, b2.l_b_corner_LC), t_line(b2.l_b_corner_LC, b2.l_t_corner_LC) };

        double min_dis = 10000;
        for (int i = 0; i < 4; i++) {
            t_line e1 = b1_lines[i];
            for (int j = 0; j < 4; j++) {
                t_line e2 = b2_lines[j];
                min_dis = std::min(min_dis, (e1.p_start - e2.p_start).Length());
                min_dis = std::min(min_dis, (e1.p_end - e2.p_end).Length());
                min_dis = std::min(min_dis, (e1.p_start - e2.p_end).Length());
                min_dis = std::min(min_dis, (e1.p_end - e2.p_start).Length());

                R2Vector mid1 = (e1.p_start + e1.p_end) * 0.5;
                R2Vector mid2 = (e2.p_start + e2.p_end) * 0.5;
                min_dis = std::min(min_dis, (mid1 - mid2).Length());
            }
        }
        if (min_dis > m_cornerDis_thrs * max_height) {
            return pair_weight;
        }
    }

    ////third, if the angle between two boxes are very large, we remove it
    //double dot_value = b1.direction_LC.Dot(b2.direction_LC);
    //double cos_value = dot_value / (b1.direction_LC.Length() * b2.direction_LC.Length());
    //double angle = RAD2DEG(std::acos(cos_value));
    //if (angle > 90.0){
    //	return pair_weight;
    //}

    // we compute the relative distance 
    R2Vector b2_to_b1 = (b1.center_position_LC - b2.center_position_LC);
    //b2_to_b1.Normalize();
    R2Vector b1_to_b2 = (b2.center_position_LC - b1.center_position_LC);
    //b1_to_b2.Normalize();
    R2Vector projection_vec_on_b1 = b2_to_b1;
    R2Vector projection_vec_on_b2 = b1_to_b2;
    projection_vec_on_b1.Project(b1.direction_LC);
    projection_vec_on_b2.Project(b2.direction_LC);
    double projection_dis_on_b1 = projection_vec_on_b1.Length();
    double projection_dis_on_b2 = projection_vec_on_b2.Length();

    pair_weight = std::abs((projection_dis_on_b1 + projection_dis_on_b2) - (b1.height + b2.height)) / (0.5 * (b1.height + b2.height));

    return pair_weight;
}

template <typename T>
MString toMString(const T& value) {
    std::stringstream ss;
    ss << value;
    return MString(ss.str().c_str());
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


    parseBoundingBoxData(filepath.asChar());
    buildGraph();
    //buildNaryTree();

    /*return parseBoundingBoxData(filepath.asChar());*/
    return MS::kSuccess;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Main Functions
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

            if (w > h) {
                bbx.width = h;
                bbx.height = w;
                bbx.angleFromY = -angle + 90;
                bbx.angleFromY_opp = -(90 + angle);
            }
            else {
                bbx.width = w;
                bbx.height = h;
                bbx.angleFromY = -angle - 180;
                bbx.angleFromY_opp = -angle;
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
            //MGlobal::displayInfo(MString("Processed bbox for class: ") + class_name.c_str());
        }
    }
    inFile.close();

    // Test
    /*for (size_t i = 0; i < m_bbx_parse.size(); ++i) {
        const auto& bbx = m_bbx_parse[i];
        MGlobal::displayInfo(MString("BBX ") + std::to_string(i + 1).c_str() + ": Center (" + std::to_string(bbx.center_position.X()).c_str() + ", " + std::to_string(bbx.center_position.Y()).c_str() + "), Width: " + std::to_string(bbx.width).c_str() + ", Height: " + std::to_string(bbx.height).c_str() + ", Angle: " + std::to_string(bbx.angleFromY).c_str());
    }*/
    return MS::kSuccess;
}

//build the graph structure
void ImportImageCmd::buildGraph() {
    m_graph.clear();

    std::vector<int> nodes_list;
    std::vector<bool> visited;
    m_root_id = 0;
    double min_y = 100000;
    //build tree nodes
    std::vector<pairwise_bbx> adjenct_bbx;
    std::cout << "number of BBX: " << m_bbx_parse.size() << std::endl;
    for (int i = 0; i < m_bbx_parse.size(); i++) {
        Bounding_box_parse cur_bbx = m_bbx_parse[i];

        //std::cout << "i: " << i << std::endl;
        for (int j = 0; j < m_bbx_parse.size(); j++) {
            if (i == j) continue;
            //std::cout << "j: " << j << std::endl;
            bool intersect = check_bbox_intersect(m_bbx_parse[i], m_bbx_parse[j]);
            double pair_weight = compute_relative_distance(m_bbx_parse[i], m_bbx_parse[j]);

            //if (intersect){
            //	//boost::add_edge(i, j, 0.0, m_graph);
            //	pairwise_bbx pb(i, j, 0.0);
            //	if (std::find(adjenct_bbx.begin(), adjenct_bbx.end(), pb) == adjenct_bbx.end()){
            //		adjenct_bbx.push_back(pb);
            //	}
            //	std::cout << "True: " << pair_weight << std::endl;
            //}
            //else{
            //	std::cout << "False: " << pair_weight << std::endl;
            //}

            if (pair_weight < m_weight_thrs) {
                //boost::add_edge(i, j, 0.0, m_graph);
                pairwise_bbx pb(i, j, pair_weight);
                if (std::find(adjenct_bbx.begin(), adjenct_bbx.end(), pb) == adjenct_bbx.end()) {
                    adjenct_bbx.push_back(pb);
                }
                //std::cout << "True: " << intersect << std::endl;
            }
            else {
                //std::cout << "False: " << intersect << std::endl;
            }

        }
        if (m_bbx_parse[i].center_position_LC.Y() < min_y) {
            min_y = m_bbx_parse[i].center_position_LC.Y();
            m_root_id = i;
        }
        nodes_list.push_back(i);
        visited.push_back(false);
    }

    for (int i = 0; i < adjenct_bbx.size(); i++) {
        pairwise_bbx cur_pb = adjenct_bbx[i];
        boost::add_edge(cur_pb.i, cur_pb.j, cur_pb.weight, m_graph);
    }

    std::vector<std::vector<vertex_t>> cycles = udgcd::FindCycles<UndirectedGraph, vertex_t>(m_graph);
    udgcd::PrintPaths(std::cout, cycles);


    MGlobal::displayInfo(MString("Build graph done!"));

    // print and test
    auto ei = edges(m_graph); // Assuming this function exists and returns a pair of iterators
    MGlobal::displayInfo("Number of edges = " + toMString(num_edges(m_graph)));
    MGlobal::displayInfo("Edge list:");

    for (auto it = ei.first; it != ei.second; ++it) {
        // Assuming *it gives you an edge descriptor that you can somehow convert to string or identifier
        MGlobal::displayInfo(toMString(*it));
    }

    // Adjacency iteration for a vertex, example vertex id '2' is used here
    UndirectedGraph::adjacency_iterator vit, vend;
    std::tie(vit, vend) = boost::adjacent_vertices(2, m_graph); // Assuming Boost-like functions
    MGlobal::displayInfo("Adjacent vertices to vertex 2:");
    for (; vit != vend; ++vit) {
        // Assuming *vit gives you a vertex descriptor that you can convert to string or identifier
        MGlobal::displayInfo(toMString(*vit));
    }

    //build_NaryTree();
}

void ImportImageCmd::processBoundingBox(Bounding_box_parse& bbx) {
    // Process and manipulate bounding box as necessary
    // This could involve setting up relationships, computing additional properties, etc.
}

void ImportImageCmd::buildNaryTree() {
    std::vector<Nary_TreeNode*> nodes_list;
    std::vector<bool> visited;
    std::vector<int> parents;
    //build tree nodes
    for (int i = 0; i < m_bbx_parse.size(); i++) {
        Bounding_box_parse cur_bbx = m_bbx_parse[i];
        Nary_TreeNode* cur_node = CreateNaryTreeNode(m_bbx_parse[i]);
        cur_node->bbx_index = i + 1;
        nodes_list.push_back(cur_node);
        visited.push_back(false);
    }
    //consturct the tree
    Nary_TreeNode* root_node = nodes_list[m_root_id];
    root_node->bbx.angleFromParent = root_node->bbx.angleFromY_LC;
    visited[m_root_id] = true;

    queue<Nary_TreeNode*> qt;
    qt.push(root_node);
    typedef boost::property_map<UndirectedGraph, boost::vertex_index_t>::type IndexMap;
    IndexMap index = get(boost::vertex_index, m_graph);

    while (!qt.empty()) {
        Nary_TreeNode* cur_node = qt.front();
        qt.pop();
        if (cur_node) {
            UndirectedGraph::adjacency_iterator vit, vend;
            std::vector<Nary_TreeNode*> children_list;
            for (std::tie(vit, vend) = boost::adjacent_vertices(cur_node->bbx_index - 1, m_graph); vit != vend; ++vit) {
                int adj_idx = index[*vit];
                if (!visited[adj_idx]) {
                    double p1 = nodes_list[adj_idx]->bbx.angleFromY_LC - cur_node->bbx.angleFromY_LC;
                    double p2 = nodes_list[adj_idx]->bbx.angleFromY_LC_opp - cur_node->bbx.angleFromY_LC;
                    nodes_list[adj_idx]->bbx.angleFromParent = abs(p1) < abs(p2) ? p1 : p2;
                    children_list.push_back(nodes_list[adj_idx]);
                    visited[adj_idx] = true;
                }
            }

            std::sort(children_list.begin(), children_list.end(), Nary_GreaterSort);
            int main_branch_idx = -1; // we find which child should be the main branch
            double min_abs_anbgle = 10000;
            for (int i = 0; i < children_list.size(); i++) {
                if (abs(children_list[i]->bbx.angleFromParent) < m_mianBranchAngle_thrs) {
                    if (abs(children_list[i]->bbx.angleFromParent) < min_abs_anbgle) {
                        min_abs_anbgle = abs(children_list[i]->bbx.angleFromParent);
                        main_branch_idx = i;
                    }
                }
            }

            for (int i = 0; i < children_list.size(); i++) {
                if (i == main_branch_idx) {
                    children_list[i]->main_branch = true;
                    children_list[i]->turn_indicator = "0";
                }
                else if (children_list[i]->bbx.angleFromParent > 0) {
                    children_list[i]->turn_indicator = "1";
                }
                else if (children_list[i]->bbx.angleFromParent < 0) {
                    children_list[i]->turn_indicator = "-1";
                }
                ConnectTreeNodes(cur_node, children_list[i]);
                qt.push(cur_node->children[i]);
            }
        }
    }
    MGlobal::displayInfo(MString("Build tree done! The initial tree: "));
    Nary_print_tree(root_node, 0);
}