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

#define Debug_parseBoundingBoxData 0
#define Debug_buildGraph 1
#define Debug_removeCycles 1
#define Debug_extractMinimalSpanningTree 1
#define Debug_buildNaryTree 1

struct ruleSuccessor {
    string successor;
    vector<double> paramters;
    ruleSuccessor() :successor("") {
    }
    ruleSuccessor(string s) :successor(s) {
    }
};
typedef unordered_map<string, vector<ruleSuccessor> > newAssociativeArray;

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Helper Variable
// TODO:: 不知道为什么不让存到private里面，调取的时候找不到，只能放这里了。。。
std::vector<Bounding_box_parse> m_bbx_parse;
Nary_TreeNode* rootNode = NULL; // Initialize pointers to NULL for safety
std::string m_save_image_name;
double m_min_len;
int m_root_id;
UndirectedGraph m_graph;
int m_tree_node_size_;
double m_average_branch_angle_;
int m_tree_node_size_angle_;
newAssociativeArray m_rules;

std::set<Nary_TreeNode*> m_selected_repetitions_nary_nodes;
std::set<string> m_selected_repetitions;

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

//void Nary_compute_strahler_number(Nary_TreeNode* root) {
//    std::vector<std::vector<Nary_TreeNode*> > matrix;
//    if (root == NULL)
//    {
//        //return matrix;
//        return;
//    }
//
//    stack<vector<Nary_TreeNode*> > sv;
//    vector<Nary_TreeNode*> temp;
//    temp.push_back(root);
//    sv.push(temp);
//
//    vector<Nary_TreeNode*> path;
//    path.push_back(root);
//
//    int count = 1;
//    while (!path.empty())
//    {
//        for (int i = 0; i < path[0]->children.size(); i++) {
//            if (path[0]->children[i] != NULL) {
//                path.push_back(path[0]->children[i]);
//            }
//        }
//        path.erase(path.begin());
//        count--;
//        if (count == 0)
//        {
//            /* vector<TreeNode*> tmp;
//            vector<TreeNode *>::iterator it = path.begin();
//            for(; it != path.end(); ++it)
//            {
//            tmp.push_back((*it));
//            }*/
//            sv.push(path);
//            count = path.size();
//        }
//    }
//
//    while (!sv.empty())
//    {
//        if (sv.top().size() > 0)
//        {
//            matrix.push_back(sv.top());
//        }
//        sv.pop();
//    }
//
//    for (int i = 0; i < matrix.size(); i++) {
//        for (int j = 0; j < matrix[i].size(); j++) {
//            Nary_TreeNode* cur_node = matrix[i][j];
//            if (cur_node->children.size() == 0) {
//                cur_node->strahler_number = 1;
//                continue;
//            }
//            int max_strahler_number = cur_node->children[0]->strahler_number;
//            bool same_order = true;
//            for (int k = 1; k < cur_node->children.size(); k++) {
//                if (cur_node->children[k]->strahler_number != max_strahler_number) {
//                    max_strahler_number = std::max(cur_node->children[k]->strahler_number, max_strahler_number);
//                    same_order = false;
//                }
//            }
//            if ((same_order && cur_node->children.size() > 1) || cur_node->children.size() > 2) {
//                cur_node->strahler_number = max_strahler_number + 1;
//            }
//            else {
//                cur_node->strahler_number = max_strahler_number;
//            }
//        }
//    }
//}
//
//string Nary_write_grammar_forPaper(Nary_TreeNode* root, bool with_paras) {
//    string str = "";
//    if (root == NULL)
//        return str;
//
//    std::ostringstream streamObj1, streamObj2, streamObj3;
//
//    streamObj3 << std::fixed; // Set Fixed -Point Notation
//    streamObj3 << std::setprecision(1); // Set precision to 2 digits
//    streamObj3 << abs(root->bbx.angleFromParent); //Add double to stream
//    string turn_angle = streamObj3.str(); // Get string from output string stream
//
//    streamObj1 << std::fixed; // Set Fixed -Point Notation
//    streamObj1 << std::setprecision(1); // Set precision to 2 digits
//    double scaler = root->bbx.height / m_min_len;
//    streamObj1 << scaler; //Add double to stream
//    string turn_scaler = streamObj1.str(); // Get string from output string stream
//
//    streamObj2 << std::fixed; // Set Fixed -Point Notation
//    streamObj2 << std::setprecision(1); // Set precision to 2 digits
//    streamObj2 << abs(root->strahler_number); //Add double to stream
//    string strahler_number = streamObj2.str(); // Get string from output string stream
//
//    if (abs(root->bbx.angleFromParent) < m_mianBranchAngle_thrs) {
//        if (with_paras) {
//            string prefix = "F(" + turn_scaler + ")";
//            str += prefix;
//        }
//        else {
//            str += "F";
//        }
//
//        int main_branch_idx = -1;
//        for (int i = 0; i < root->children.size(); i++) {
//            if (root->children[i]->main_branch) {
//                main_branch_idx = i;
//                continue;
//            }
//            str += Nary_write_grammar_forPaper(root->children[i], with_paras);
//        }
//        if (main_branch_idx != -1) {
//            str += Nary_write_grammar_forPaper(root->children[main_branch_idx], with_paras);
//        }
//    }
//    else if (root->bbx.angleFromParent > 0) {
//        if (with_paras) {
//            //str += "[+(" + std::to_string(root->bbx.angleFromParent) + ")F";
//            string prefix1 = "F(" + turn_scaler + ")";
//            string prfix = "[+(" + turn_angle + ")" + prefix1;
//            str += prfix;
//        }
//        else {
//            str += "[+F";
//        }
//
//        int main_branch_idx = -1;
//        for (int i = 0; i < root->children.size(); i++) {
//            if (root->children[i]->main_branch) {
//                main_branch_idx = i;
//                continue;
//            }
//            str += Nary_write_grammar_forPaper(root->children[i], with_paras);
//        }
//        if (main_branch_idx != -1) {
//            str += Nary_write_grammar_forPaper(root->children[main_branch_idx], with_paras);
//        }
//        str += "]";
//        //std::cout << "[+F]";
//    }
//    else if (root->bbx.angleFromParent < 0) {
//        //std::cout << "[-(" << -root->bbx.angleFromParent << ")F]";
//        //str += "[-(" + std::to_string(root->bbx.angleFromParent) + ")F";
//        //str += "[-F";
//        if (with_paras) {
//            string prefix1 = "F(" + turn_scaler + ")";
//            string prfix = "[-(" + turn_angle + ")" + prefix1;
//            str += prfix;
//        }
//        else {
//            str += "[-F";
//        }
//
//        int main_branch_idx = -1;
//        for (int i = 0; i < root->children.size(); i++) {
//            if (root->children[i]->main_branch) {
//                main_branch_idx = i;
//                continue;
//            }
//            str += Nary_write_grammar_forPaper(root->children[i], with_paras);
//        }
//        if (main_branch_idx != -1) {
//            str += Nary_write_grammar_forPaper(root->children[main_branch_idx], with_paras);
//        }
//        str += "]";
//        //std::cout << "[-F]";
//    }
//    return str;
//}
//
//string Nary_write_grammar(Nary_TreeNode* root, bool with_paras) {
//    string str = "";
//    if (root == NULL)
//        return str;
//
//    std::ostringstream streamObj1, streamObj2, streamObj3;
//
//    streamObj3 << std::fixed; // Set Fixed -Point Notation
//    streamObj3 << std::setprecision(1); // Set precision to 2 digits
//    streamObj3 << abs(root->bbx.angleFromParent); //Add double to stream
//    string turn_angle = streamObj3.str(); // Get string from output string stream
//
//    streamObj1 << std::fixed; // Set Fixed -Point Notation
//    streamObj1 << std::setprecision(1); // Set precision to 2 digits
//    double scaler = root->bbx.height / m_min_len;
//    streamObj1 << scaler; //Add double to stream
//    string turn_scaler = streamObj1.str(); // Get string from output string stream
//
//    streamObj2 << std::fixed; // Set Fixed -Point Notation
//    streamObj2 << std::setprecision(1); // Set precision to 2 digits
//    streamObj2 << abs(root->strahler_number); //Add double to stream
//    string strahler_number = streamObj2.str(); // Get string from output string stream
//
//    if (abs(root->bbx.angleFromParent) < m_mianBranchAngle_thrs) {
//        if (with_paras) {
//            //string prefix = "<*(" + turn_scaler + ")F>";
//            string prefix = "<*(" + turn_scaler + ")&(" + strahler_number + ")F>";
//            str += prefix;
//        }
//        else {
//            str += "F";
//        }
//
//        int main_branch_idx = -1;
//        for (int i = 0; i < root->children.size(); i++) {
//            if (root->children[i]->main_branch) {
//                main_branch_idx = i;
//                continue;
//            }
//            str += Nary_write_grammar(root->children[i], with_paras);
//        }
//        if (main_branch_idx != -1) {
//            str += Nary_write_grammar(root->children[main_branch_idx], with_paras);
//        }
//    }
//    else if (root->bbx.angleFromParent > 0) {
//        if (with_paras) {
//            //str += "[+(" + std::to_string(root->bbx.angleFromParent) + ")F";
//            string prefix1 = "<*(" + turn_scaler + ")&(" + strahler_number + ")F>";
//            string prfix = "[+(" + turn_angle + ")" + prefix1;
//            str += prfix;
//        }
//        else {
//            str += "[+F";
//        }
//
//        int main_branch_idx = -1;
//        for (int i = 0; i < root->children.size(); i++) {
//            if (root->children[i]->main_branch) {
//                main_branch_idx = i;
//                continue;
//            }
//            str += Nary_write_grammar(root->children[i], with_paras);
//        }
//        if (main_branch_idx != -1) {
//            str += Nary_write_grammar(root->children[main_branch_idx], with_paras);
//        }
//        str += "]";
//        //std::cout << "[+F]";
//    }
//    else if (root->bbx.angleFromParent < 0) {
//        //std::cout << "[-(" << -root->bbx.angleFromParent << ")F]";
//        //str += "[-(" + std::to_string(root->bbx.angleFromParent) + ")F";
//        //str += "[-F";
//        if (with_paras) {
//            string prefix1 = "<*(" + turn_scaler + ")&(" + strahler_number + ")F>";
//            string prfix = "[-(" + turn_angle + ")" + prefix1;
//            str += prfix;
//        }
//        else {
//            str += "[-F";
//        }
//
//        int main_branch_idx = -1;
//        for (int i = 0; i < root->children.size(); i++) {
//            if (root->children[i]->main_branch) {
//                main_branch_idx = i;
//                continue;
//            }
//            str += Nary_write_grammar(root->children[i], with_paras);
//        }
//        if (main_branch_idx != -1) {
//            str += Nary_write_grammar(root->children[main_branch_idx], with_paras);
//        }
//        str += "]";
//        //std::cout << "[-F]";
//    }
//    return str;
//}
//
//double Nary_get_scalar_para(Nary_TreeNode* root) {
//    double scalar;
//
//    if (root != NULL) {
//        if (root->children.size() > 0) {
//            double max_scale = -1000;
//            for (int i = 0; i < root->children.size(); i++) {
//                double s = root->bbx.height / root->children[i]->bbx.height;
//                max_scale = std::max(max_scale, s);
//                if (!root->children[i]->main_branch) {
//                    m_average_branch_angle_ += std::abs(root->children[i]->bbx.angleFromParent);
//                    m_tree_node_size_angle_++;
//                }
//            }
//            scalar = std::max(max_scale, 1.0);
//            m_tree_node_size_++;
//        }
//        else {
//            return 0.0;
//        }
//    }
//    else {
//        return 0.0;
//    }
//
//    for (int i = 0; i < root->children.size(); i++) {
//        scalar += Nary_get_scalar_para(root->children[i]);
//    }
//    return scalar;
//}
//
//struct Nary_repetition_node {
//    int oocur_time;
//    int last_groups_numer;
//    int group_node_size;
//    std::vector<Nary_TreeNode*> parent_node;
//    //std::unordered_set<TreeNode *> parent_node;
//    Nary_repetition_node() : oocur_time(0), last_groups_numer(0) {}
//};
//
//string Nary_find_repetitions(Nary_TreeNode* node, unordered_map<string, Nary_repetition_node>& m)
//{
//    if (!node)
//        return "";
//
//    string str = "(";
//    //str += node->turn_indicator;
//    int main_branch_idx = -1;
//    for (int i = 0; i < node->children.size(); i++) {
//        if (node->children[i]->main_branch) {
//            main_branch_idx = i;
//            continue;
//        }
//        if (node->cluster_level == node->children[i]->cluster_level) {
//            str += node->children[i]->turn_indicator;
//            str += Nary_find_repetitions(node->children[i], m);
//        }
//
//    }
//    if (main_branch_idx != -1 && node->cluster_level == node->children[main_branch_idx]->cluster_level) {
//        str += node->children[main_branch_idx]->turn_indicator;
//        str += Nary_find_repetitions(node->children[main_branch_idx], m);
//    }
//    //str += to_string(node->data);
//    str += ")";
//
//    // Subtree already present (Note that we use 
//    // unordered_map instead of unordered_set 
//    // because we want to print multiple duplicates 
//    // only once, consider example of 4 in above 
//    // subtree, it should be printed only once. 
//    //if (m[str].oocur_time >= 1 && str.length()>3)
//    //	cout << node->bbx_index << " ";
//
//    std::set<Nary_TreeNode*>::iterator siter;
//    siter = m_selected_repetitions_nary_nodes.find(node);
//    if (siter == m_selected_repetitions_nary_nodes.end()) {
//        m[str].oocur_time++;
//        m[str].parent_node.push_back(node);
//    }
//
//    /*std::pair<std::unordered_set<TreeNode *>::iterator, bool> ret;
//    ret = m[str].parent_node.insert(node);
//    if (ret.second){
//    m[str].oocur_time++;
//    }*/
//
//    return str;
//}
//
//string Nary_find_repetitions(Nary_TreeNode* node, unordered_map<string, Nary_repetition_node>& m)
//{
//    if (!node)
//        return "";
//
//    string str = "(";
//    //str += node->turn_indicator;
//    int main_branch_idx = -1;
//    for (int i = 0; i < node->children.size(); i++) {
//        if (node->children[i]->main_branch) {
//            main_branch_idx = i;
//            continue;
//        }
//        if (node->cluster_level == node->children[i]->cluster_level) {
//            str += node->children[i]->turn_indicator;
//            str += Nary_find_repetitions(node->children[i], m);
//        }
//
//    }
//    if (main_branch_idx != -1 && node->cluster_level == node->children[main_branch_idx]->cluster_level) {
//        str += node->children[main_branch_idx]->turn_indicator;
//        str += Nary_find_repetitions(node->children[main_branch_idx], m);
//    }
//    //str += to_string(node->data);
//    str += ")";
//
//    // Subtree already present (Note that we use 
//    // unordered_map instead of unordered_set 
//    // because we want to print multiple duplicates 
//    // only once, consider example of 4 in above 
//    // subtree, it should be printed only once. 
//    //if (m[str].oocur_time >= 1 && str.length()>3)
//    //	cout << node->bbx_index << " ";
//
//    std::set<Nary_TreeNode*>::iterator siter;
//    siter = m_selected_repetitions_nary_nodes.find(node);
//    if (siter == m_selected_repetitions_nary_nodes.end()) {
//        m[str].oocur_time++;
//        m[str].parent_node.push_back(node);
//    }
//
//    /*std::pair<std::unordered_set<TreeNode *>::iterator, bool> ret;
//    ret = m[str].parent_node.insert(node);
//    if (ret.second){
//    m[str].oocur_time++;
//    }*/
//
//    return str;
//}
//
//string Nary_select_prefer_repetition(unordered_map<string, Nary_repetition_node>& m, double weight, bool& find) {
//    string select_str = "";
//    std::unordered_map<string, Nary_repetition_node>::iterator iter;
//    //now we only get the largest sub-tree, but we prefer the one has selected before
//    int max_node_size = -1;
//
//    std::vector<string> selected_strs;
//    std::vector<int> node_sizes;
//    int max_node_size_in_selected = -1;
//    int max_node_pos_in_selected = 0;
//
//    std::pair<std::set<string>::iterator, bool> ret;
//
//    for (iter = m.begin(); iter != m.end(); iter++)
//    {
//        // get the node numbers of this sub-tree
//        string str = iter->first;
//        int num = 0;
//        for (int j = 0; j < str.length(); j++) {
//            if (str.at(j) == '(' || str.at(j) == ')' || str.at(j) == '-') {
//                num++;
//            }
//        }
//        int group_node_size = str.length() - num + 1;
//        iter->second.group_node_size = group_node_size;
//
//        std::set<string>::iterator siter;
//        siter = m_selected_repetitions.find(str);
//        //ret = m_selected_repetitions.insert(str);
//
//        /*
//        2020.02.08: Jianwei changed the 'group_node_size >= 3' to 'group_node_size >= 2'
//        */
//        if ((iter->second.oocur_time > 1 && group_node_size >= 3) || siter != m_selected_repetitions.end()) {
//            //if (iter->second.oocur_time > 1 && group_node_size >= 3){
//            if (group_node_size > max_node_size) {
//                max_node_size = group_node_size;
//                select_str = iter->first;
//            }
//            if (siter != m_selected_repetitions.end()) {
//                selected_strs.push_back(iter->first);
//                node_sizes.push_back(group_node_size);
//                if (group_node_size > max_node_size_in_selected) {
//                    max_node_size_in_selected = group_node_size;
//                    max_node_pos_in_selected = selected_strs.size() - 1;
//                }
//            }
//        }
//    }
//
//    if (selected_strs.empty()) {
//        find = false;
//        return select_str;
//    }
//    else {
//        if (max_node_size_in_selected == max_node_size) {
//            find = true;
//            return select_str;
//        }
//        else {
//            find = true;
//            return selected_strs[max_node_pos_in_selected];
//        }
//    }
//
//    return select_str;
//}
//
//void Nary_generate_conformal_grammar(Nary_TreeNode* root, unordered_map<string, Nary_repetition_node>& m) {
//
//    if (root == NULL) {
//        return;
//    }
//    int max_iter = 5;
//    int type_id = 0;
//    double weight = 0.5;
//    int real_iters = 0;
//    for (int k = 0; k < max_iter; k++) {
//        //find all repetitions in current tree/sub-tree
//        bool new_repetition = false;
//        m.clear();
//        Nary_find_repetitions(root, m);
//        std::unordered_map<string, Nary_repetition_node>::iterator iter;
//        for (iter = m.begin(); iter != m.end(); iter++) {
//            if (iter->second.oocur_time > 1 && iter->second.oocur_time > iter->second.last_groups_numer) {
//                new_repetition = true;
//            }
//        }
//        //stop if cannot cluster anymore
//        bool find_again = false;
//        string str = Nary_select_prefer_repetition(m, weight, find_again);
//        if (str == "") {
//            //std::cout << "No selected repetition!" << std::endl;
//            if (root->parent != NULL && !root->old_repetition) {
//                ruleSuccessor rule = Nary_write_rules(root, true);
//                m_rules[alphabet[m_alphabet_pointer]].push_back(rule);
//                //std::cout << alphabet[m_alphabet_pointer] << " -> " << rule.successor << endl;
//                //m_alphabet_pointer++;
//            }
//            break;
//        }
//        m_selected_repetitions.insert(str);
//
//        //update the cluster information according to current repetition
//        Nary_repetition_node r_node = m[str];
//        int last_groups_numer = r_node.last_groups_numer;
//        if (!find_again) {
//            m_used_symbols[str] = alphabet[m_alphabet_pointer];
//        }
//
//        //int cluster_id = 0;
//        for (int i = last_groups_numer; i < r_node.parent_node.size(); i++) {
//            r_node.parent_node[i]->old_repetition = find_again;
//            m_selected_repetitions_nary_nodes.insert(r_node.parent_node[i]);
//            if (i == 0) {// we only genrate the grammar once for current repetition
//                Nary_generate_conformal_grammar(r_node.parent_node[i], m);
//                //if (find_again) m_alphabet_pointer--;
//            }
//            if (find_again) {
//                bool update = Nary_update_cluster_infomation(r_node.parent_node[i], i, m_used_symbols[str]); //update the node information;
//            }
//            else {
//                bool update = Nary_update_cluster_infomation(r_node.parent_node[i], i, alphabet[m_alphabet_pointer]); //update the node information;
//            }
//
//            Nary_perform_clustring(r_node.parent_node[i], r_node.parent_node[i]->old_repetition); // perform real clustring on the sub-tree, i.e., collpase each repetition to one node 
//        }
//        if (!find_again) m_alphabet_pointer++;
//
//        real_iters++;
//
//        //std::cout << "***********After iteration " << k + 1 << "**************" << std::endl;
//        //Nary_print_tree(root, 0);
//    }
//
//    //cout << "Our Greedy Iterations: " << real_iters << endl;
//}
//
//void GlViewer::grammar_induction() {
//    // collect information about the grammar 
//    std::vector<ruleProductions> productions;
//    newAssociativeArray::const_iterator iter;
//    std::unordered_map<string, int> symbols_infor;
//    for (iter = m_rules.begin(); iter != m_rules.end(); ++iter)
//    {
//        string left = iter->first;
//        symbols_infor[left] = 0;
//        vector<ruleSuccessor> sucs = iter->second;
//        for (int i = 0; i < sucs.size(); i++) {
//            string right = sucs[i].successor;
//            //bool has_non_terminals = rule_has_non_terminals(right);
//            /*bool has_non_terminals = false;
//            for (int i = 0; i < right.length(); i++) {
//            if (!(right.at(i) == '(' || right.at(i) == ')' || right.at(i) == '+' || right.at(i) == '-' || right.at(i) == '[' || right.at(i) == ']' || right.at(i) == 'F')) {
//            has_non_terminals = true;
//            break;
//            }
//            }*/
//            //if(has_non_terminals) {
//            ruleProductions rule(left, right);
//            productions.push_back(rule);
//            symbols_infor[left] += 1;
//            //}
//        }
//    }
//
//    // we merge similar rules in a greedy way
//    int max_iter = 10;
//    double edit_dis_threshold = 6;
//    for (int i = 0; i < max_iter; i++) {
//        bool merged = false;
//        //compute current grammar length
//        int grammar_length = compute_grammar_length(productions, symbols_infor);
//
//        //find and merge two rules that can be merged
//        //std::vector<pair_rule> candidate_pairs;
//        //std::vector<int> min_loss_ids;
//        double min_loss = 100000;
//        std::vector<ruleProductions> productions_new;
//        std::vector<ruleProductions> productions_final;
//        //bool update = false;
//        for (int j = 0; j < productions.size(); j++)
//        {
//            //cout << iter->second[0].successor << endl;
//            string left1 = productions[j].precessor;
//            string right1 = productions[j].successor;
//            //if (left1 == "S") {
//            for (int k = 0; k < productions.size(); k++) {
//                bool update = false;
//                if (j == k) continue;
//                //cout << iter2->second[0].successor << endl;
//                string left2 = productions[k].precessor;
//                string right2 = productions[k].successor;
//
//                if (left1 == left2) continue;
//
//                double min_edit_distance;
//                if (left2 == "S") {
//                    min_edit_distance = edit_distance_DP(right1, right2);
//                }
//                else {
//                    min_edit_distance = edit_distance_DP(right2, right1);
//                }
//
//                if (productions.size() == 2) {
//                    min_edit_distance = 0;
//                }
//                if (min_edit_distance > edit_dis_threshold) {
//                    continue;
//                }
//                productions_new.clear();
//                productions_new.assign(productions.begin(), productions.end()); // copy all rules
//
//                if (left2 == "S") {
//                    merged = merge_two_rules(productions_new, k, j, symbols_infor);
//                }
//                else {
//                    merged = merge_two_rules(productions_new, j, k, symbols_infor);
//                }
//                int grammar_length_new = compute_grammar_length(productions_new, symbols_infor);
//                double loss = (grammar_length_new - grammar_length) + min_edit_distance;
//                if (loss < min_loss) {
//                    min_loss = loss;
//                    productions_final.clear();
//                    productions_final.assign(productions_new.begin(), productions_new.end());
//                    update = true;
//                }
//                //swap
//                if (update) {
//                    productions.assign(productions_final.begin(), productions_final.end());
//                    productions_final.clear();
//                }
//            }
//            //}
//        }
//
//        //cout << "Merge iteration: " << i << endl;
//        if (merged == false) break;
//    }
//
//    //output 
//    cout << "After grammar generalization..." << endl;
//    m_final_grammar_length_ = 0;
//    for (int k = 0; k < productions.size(); k++) {
//
//        string right = productions[k].successor;
//        bool has_non_terminals = false;
//        for (int i = 0; i < right.length(); i++) {
//            if (!(right.at(i) == '(' || right.at(i) == ')' || right.at(i) == '+' || right.at(i) == '-' || right.at(i) == '[' || right.at(i) == ']' || right.at(i) == 'F')) {
//                has_non_terminals = true;
//                break;
//            }
//        }
//        if (!has_non_terminals) continue; // if the right side only contains terminal 'F', we discard this rule
//
//        if (symbols_infor[productions[k].precessor] > 1) {
//            string::size_type idx = productions[k].successor.find(productions[k].precessor);
//            if (idx != string::npos) {
//                cout << productions[k].precessor << "->" << productions[k].successor << endl;
//                m_final_grammar_length_ += productions[k].precessor.length();
//                m_final_grammar_length_ += productions[k].successor.length();
//            }
//            continue;
//        }
//
//        cout << productions[k].precessor << "->" << productions[k].successor << endl;
//        m_final_grammar_length_ += productions[k].precessor.length();
//        m_final_grammar_length_ += productions[k].successor.length();
//    }
//}

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
    buildGraph(); // TODO: print出来的data不对，有bug

    removeCycles();
    extractMinimalSpanningTree();
    buildNaryTree();

    return MS::kSuccess;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Main Logics Functions
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
#if Debug_parseBoundingBoxData
    for (size_t i = 0; i < m_bbx_parse.size(); ++i) {
        const auto& bbx = m_bbx_parse[i];
        MGlobal::displayInfo(MString("BBX ") + std::to_string(i + 1).c_str() + ": Center (" + std::to_string(bbx.center_position.X()).c_str() + ", " + std::to_string(bbx.center_position.Y()).c_str() + "), Width: " + std::to_string(bbx.width).c_str() + ", Height: " + std::to_string(bbx.height).c_str() + ", Angle: " + std::to_string(bbx.angleFromY).c_str());
    }
#endif
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
#if Debug_buildGraph
    auto ei = edges(m_graph); // Assuming this function exists and returns a pair of iterators
    MGlobal::displayInfo("Number of edges = " + toMString(num_edges(m_graph)));
    MGlobal::displayInfo("Edge list:");

    for (auto it = ei.first; it != ei.second; ++it) {
        MGlobal::displayInfo(toMString(*it));
    }

    // Adjacency iteration for a vertex, example vertex id '2' is used here
    UndirectedGraph::adjacency_iterator vit, vend;
    std::tie(vit, vend) = boost::adjacent_vertices(2, m_graph);
    MGlobal::displayInfo("Adjacent vertices to vertex 2:");
    for (; vit != vend; ++vit) {
        MGlobal::displayInfo(toMString(*vit));
    }
#endif
}

void ImportImageCmd::removeCycles() {
    int iters = 0;
    bool has_cycles = false;
    do {
        //find cycles
        std::vector<std::vector<vertex_t>> cycles = udgcd::FindCycles<UndirectedGraph, vertex_t>(m_graph);
        if (cycles.size() == 0) break;
#if Debug_removeCycles
        MGlobal::displayInfo("Cycles number: " + MString() + cycles.size());
        udgcd::PrintPaths(std::cout, cycles);
#endif
        has_cycles = true;
        iters++;

        // Create property_map from edges to weights
        boost::property_map<UndirectedGraph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, m_graph);
        std::vector<vertex_t> p(boost::num_vertices(m_graph));
        std::vector<double> d(num_vertices(m_graph));
        boost::dijkstra_shortest_paths(m_graph, m_root_id, boost::predecessor_map(&p[0]).distance_map(&d[0]));

        //break cycles
        for (int i = 0; i < cycles.size(); i++) {

            if (cycles[i].size() == 3) {
                double min_distance = 10000;
                int indx;
                for (int k = 0; k < 3; k++) {
                    int goal = cycles[i][k];
                    std::vector<int> nodes;
                    double real_dis = 0;
                    while (goal != m_root_id) {
                        nodes.push_back(goal);
                        real_dis += d[goal];
                        goal = p[goal];
                    }

                    if (min_distance > nodes.size()) {
                        //if (max_distance < real_dis){
                        min_distance = nodes.size();
                        //max_distance = real_dis;
                        indx = k;
                    }
                }
                if (indx == 0) {
                    //int left = cycles[i][1], right = cycles[i][2];
                    boost::remove_edge(cycles[i][1], cycles[i][2], m_graph);
                }
                else if (indx == 1) {
                    boost::remove_edge(cycles[i][0], cycles[i][2], m_graph);
                }
                else {
                    boost::remove_edge(cycles[i][0], cycles[i][1], m_graph);
                }

                continue;
            }

            std::vector<std::pair<int, int>> remove_edges;

            for (int j = 0; j < cycles[i].size(); j++) {
                int b_left, b_middle, b_right;
                if (j == 0) {
                    b_left = cycles[i][cycles[i].size() - 1], b_middle = cycles[i][j], b_right = cycles[i][j + 1];
                }
                else {
                    b_left = cycles[i][j - 1], b_middle = cycles[i][j], b_right = cycles[i][(j + 1) % cycles[i].size()];
                }
                R2Vector bm_to_bl = m_bbx_parse[b_left].center_position_LC - m_bbx_parse[b_middle].center_position_LC;
                R2Vector bm_to_br = m_bbx_parse[b_right].center_position_LC - m_bbx_parse[b_middle].center_position_LC;
                double dot_value = bm_to_bl.Dot(bm_to_br);
                double cos_value = dot_value / (bm_to_bl.Length() * bm_to_br.Length());
                double angle_1 = RAD2DEG(std::acos(cos_value));
                if (angle_1 < 90.0) {
                    dot_value = m_bbx_parse[b_middle].direction_LC.Dot(m_bbx_parse[b_left].direction_LC);
                    cos_value = dot_value / (m_bbx_parse[b_middle].direction_LC.Length() * m_bbx_parse[b_left].direction_LC.Length());
                    double angle_left = RAD2DEG(std::acos(cos_value));

                    dot_value = m_bbx_parse[b_middle].direction_LC.Dot(m_bbx_parse[b_right].direction_LC);
                    cos_value = dot_value / (m_bbx_parse[b_middle].direction_LC.Length() * m_bbx_parse[b_right].direction_LC.Length());
                    double angle_right = RAD2DEG(std::acos(cos_value));

                    if (angle_left < angle_right) {
                        remove_edges.push_back(std::make_pair(b_right, b_middle));
                    }
                    else {
                        remove_edges.push_back(std::make_pair(b_left, b_middle));
                    }
                }
            }

            if (remove_edges.size() > 0) {
                double max_distance = -10000;
                int indx;

                for (int k = 0; k < remove_edges.size(); k++) {
                    int goal = remove_edges[k].second;
                    std::vector<int> nodes;
                    std::vector<float> distances;
                    double real_dis = 0;
                    while (goal != m_root_id) {
                        nodes.push_back(goal);
                        distances.push_back(d[goal]);
                        real_dis += d[goal];
                        goal = p[goal];
                    }

                    if (max_distance < nodes.size()) {
                        //if (max_distance < real_dis){
                        max_distance = nodes.size();
                        //max_distance = real_dis;
                        indx = k;
                    }
                }
                boost::remove_edge(remove_edges[indx].first, remove_edges[indx].second, m_graph);
            }
        }

    } while (has_cycles || iters < 5);
#if Debug_removeCycles
    MGlobal::displayInfo(MString("Break cycles done! Iters: ") + MString() + iters);
#endif
}

void ImportImageCmd::extractMinimalSpanningTree() {
    boost::property_map<UndirectedGraph, boost::edge_weight_t >::type weight = get(boost::edge_weight, m_graph);
    std::vector<edge_descriptor> spanning_tree;

    boost::kruskal_minimum_spanning_tree(m_graph, std::back_inserter(spanning_tree));

    UndirectedGraph new_graph;
    for (std::vector<edge_descriptor>::iterator ei = spanning_tree.begin();
        ei != spanning_tree.end(); ++ei)
    {
        boost::add_edge(source(*ei, m_graph), target(*ei, m_graph), weight[*ei], new_graph);
    }

    m_graph.clear();
    m_graph = new_graph;
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
#if Debug_buildNaryTree
    MGlobal::displayInfo(MString("Build tree done! The initial tree: "));
    Nary_print_tree(root_node, 0);
#endif
    // TODO: More codes here
    //Nary_compute_strahler_number(root_node);
    //bool output_strahler_number = true;

    ////write expansion grammer
    //std::cout << "Expansion grammar (input string) with parameters: " << std::endl;
    //string expansion_grammer_withParas;
    ////expansion_grammer_withParas = Nary_write_grammar(root_node, true);
    //expansion_grammer_withParas = Nary_write_grammar_forPaper(root_node, true);
    //std::cout << expansion_grammer_withParas << std::endl;

    //std::cout << "Expansion grammar (input string) without parameters: " << std::endl;
    //string expansion_grammer = Nary_write_grammar(root_node, false);
    //std::cout << expansion_grammer << std::endl;
    //std::cout << std::endl;

    ////get the scale and branching angle parameters
    //m_tree_node_size_ = 0;
    //m_average_branch_angle_ = 0.0;
    //m_tree_node_size_angle_ = 0;
    //double scalar_para = Nary_get_scalar_para(root_node);
    //scalar_para /= m_tree_node_size_;
    //m_average_branch_angle_ /= m_tree_node_size_angle_;

    //m_rules.clear();

    //if (m_optAlgorithm_ == "Our") {
    //    //inference using our method
    //    m_alphabet_pointer = 2;
    //    unordered_map<string, Nary_repetition_node> m;
    //    m_selected_repetitions.clear();
    //    m_selected_repetitions_nary_nodes.clear();
    //    m_used_symbols.clear();
    //    Nary_generate_conformal_grammar(root_node, m);
    //    // write the last rule
    //    ruleSuccessor rule = Nary_write_rules(root_node, true);
    //    m_rules[alphabet[1]].push_back(rule);
    //    std::cout << alphabet[1] << " -> " << rule.successor << std::endl;
    //}

    //std::cout << std::endl;

    ////print grammer
    //std::cout << "The compact conformal grammar: " << std::endl;
    //newAssociativeArray::const_iterator iter;
    //int compact_grammar_len = 0;
    //for (iter = m_rules.begin(); iter != m_rules.end(); ++iter)
    //{
    //    string left = iter->first;
    //    vector<ruleSuccessor> sucs = iter->second;
    //    for (int i = 0; i < sucs.size(); i++) {
    //        string right = sucs[i].successor;
    //        cout << left << " = " << right << endl;
    //        compact_grammar_len += left.length();
    //        compact_grammar_len += right.length();
    //    }
    //}
    //std::cout << std::endl;

    ////merge similar rules
    //grammar_induction();

    ////write the last rule about the paramter
    //std::ostringstream streamObj3;
    //streamObj3 << std::fixed; // Set Fixed -Point Notation
    //streamObj3 << std::setprecision(2); // Set precision to 2 digits
    //streamObj3 << scalar_para; //Add double to stream
    ////string succ = "<*(" + streamObj3.str() + ")F>";
    //string succ = "F(*" + streamObj3.str() + ")";
    //std::cout << "F->" << succ << std::endl;

    //std::cout << std::endl;
}