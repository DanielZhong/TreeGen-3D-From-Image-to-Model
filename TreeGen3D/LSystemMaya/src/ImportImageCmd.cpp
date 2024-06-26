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
#include "global_definations.h"
#include <iostream>

#define Debug_parseBoundingBoxData 0
#define Debug_buildGraph 1
#define Debug_removeCycles 1
#define Debug_extractMinimalSpanningTree 1
#define Debug_buildNaryTree 1

// forward declaration
string Nary_write_grammar(MultiwayTreeNode* root, bool with_paras = true);
string Nary_write_grammar_forPaper(MultiwayTreeNode* root, bool with_paras = true);
double Nary_get_scalar_para(MultiwayTreeNode* root);
void Nary_generate_conformal_grammar(MultiwayTreeNode* root, unordered_map<string, Nary_repetition_node>& m);
string Nary_find_repetitions(MultiwayTreeNode* node, unordered_map<string, Nary_repetition_node>& m);
string Nary_select_prefer_repetition(unordered_map<string, Nary_repetition_node>& m, double weight, bool& find);
bool Nary_update_cluster_infomation(MultiwayTreeNode* node, int cluster_id, string symb);
void Nary_perform_clustring(MultiwayTreeNode* node, bool find = false);
bool Nary_perform_collaps_leaf(MultiwayTreeNode* node, bool find = false);
bool Nary_perform_move_leaf(MultiwayTreeNode* node, bool find);
double edit_distance_DP(string str1, string str2);

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
std::vector<BoundingBox> m_bbx_parse;
MultiwayTreeNode* rootNode = NULL; // Initialize pointers to NULL for safety
std::string m_save_image_name;
double m_min_len;
int m_root_id;
UndirectedGraph m_graph;
int m_tree_node_size_;
double m_average_branch_angle_;
int m_tree_node_size_angle_;
newAssociativeArray m_rules;

std::set<MultiwayTreeNode*> m_selected_repetitions_nary_nodes;
std::set<string> m_selected_repetitions;

double m_mianBranchAngle_thrs = 5.0;
int m_alphabet_pointer;
std::unordered_map<string, string> m_used_symbols;

// Helper Functions
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

bool inline Nary_GreaterSort(MultiwayTreeNode* a, MultiwayTreeNode* b) {
    return (a->bbx.angleFromParent > b->bbx.angleFromParent);
}

void Nary_print_tree(MultiwayTreeNode* root, int spaces) {
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

bool check_bbox_intersect(BoundingBox b1, BoundingBox b2) {
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

    // third, do the exact test
    QPolygonF a, b;
    a << QPointF(b1.l_t_corner_LC.X(), b1.l_t_corner_LC.Y()) << QPointF(b1.r_t_corner_LC.X(), b1.r_t_corner_LC.Y())
        << QPointF(b1.r_b_corner_LC.X(), b1.r_b_corner_LC.Y()) << QPointF(b1.l_b_corner_LC.X(), b1.l_b_corner_LC.Y());
    b << QPointF(b2.l_t_corner_LC.X(), b2.l_t_corner_LC.Y()) << QPointF(b2.r_t_corner_LC.X(), b2.r_t_corner_LC.Y())
        << QPointF(b2.r_b_corner_LC.X(), b2.r_b_corner_LC.Y()) << QPointF(b2.l_b_corner_LC.X(), b2.l_b_corner_LC.Y());

    for (int polyi = 0; polyi < 2; ++polyi)
    {
        const QPolygonF& polygon = polyi == 0 ? a : b;

        for (int i1 = 0; i1 < polygon.size(); ++i1)
        {
            const int i2 = (i1 + 1) % polygon.size();

            const double normalx = polygon[i2].y() - polygon[i1].y();
            const double normaly = polygon[i1].x() - polygon[i2].x();

            double minA = std::numeric_limits<double>::max();
            double maxA = std::numeric_limits<double>::min();
            for (int ai = 0; ai < a.size(); ++ai)
            {
                const double projected = normalx * a[ai].x() +
                    normaly * a[ai].y();
                if (projected < minA) minA = projected;
                if (projected > maxA) maxA = projected;
            }

            double minB = std::numeric_limits<double>::max();
            double maxB = std::numeric_limits<double>::min();
            for (int bi = 0; bi < b.size(); ++bi)
            {
                const double projected = normalx * b[bi].x() +
                    normaly * b[bi].y();
                if (projected < minB) minB = projected;
                if (projected > maxB) maxB = projected;
            }

            if (maxA < minB || maxB < minA)
                return false;
        }
    }

    return true;
}

double compute_relative_distance(BoundingBox b1, BoundingBox b2) {

    double pair_weight = 100;
    double max_height = (b1.height > b2.height) ? b1.height : b2.height;
    double cc_distance = (b1.center_position_LC - b2.center_position_LC).Length();
    if (cc_distance > 1 * max_height) {
        return pair_weight;
    }

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
        if (min_dis > 0.1 * max_height) {
            return pair_weight;
        }
    }

    R2Vector b2_to_b1 = (b1.center_position_LC - b2.center_position_LC);
    R2Vector b1_to_b2 = (b2.center_position_LC - b1.center_position_LC);
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

void Nary_compute_strahler_number(MultiwayTreeNode* root) {
    std::vector<std::vector<MultiwayTreeNode*> > matrix;
    if (root == NULL)
    {
        return;
    }

    stack<vector<MultiwayTreeNode*> > sv;
    vector<MultiwayTreeNode*> temp;
    temp.push_back(root);
    sv.push(temp);

    vector<MultiwayTreeNode*> path;
    path.push_back(root);

    int count = 1;
    while (!path.empty())
    {
        for (int i = 0; i < path[0]->children.size(); i++) {
            if (path[0]->children[i] != NULL) {
                path.push_back(path[0]->children[i]);
            }
        }
        path.erase(path.begin());
        count--;
        if (count == 0)
        {
            sv.push(path);
            count = path.size();
        }
    }

    while (!sv.empty())
    {
        if (sv.top().size() > 0)
        {
            matrix.push_back(sv.top());
        }
        sv.pop();
    }

    for (int i = 0; i < matrix.size(); i++) {
        for (int j = 0; j < matrix[i].size(); j++) {
            MultiwayTreeNode* cur_node = matrix[i][j];
            if (cur_node->children.size() == 0) {
                cur_node->strahler_number = 1;
                continue;
            }
            int max_strahler_number = cur_node->children[0]->strahler_number;
            bool same_order = true;
            for (int k = 1; k < cur_node->children.size(); k++) {
                if (cur_node->children[k]->strahler_number != max_strahler_number) {
                    max_strahler_number = std::max(cur_node->children[k]->strahler_number, max_strahler_number);
                    same_order = false;
                }
            }
            if ((same_order && cur_node->children.size() > 1) || cur_node->children.size() > 2) {
                cur_node->strahler_number = max_strahler_number + 1;
            }
            else {
                cur_node->strahler_number = max_strahler_number;
            }
        }
    }
}

string Nary_write_grammar_forPaper(MultiwayTreeNode* root, bool with_paras) {
    string str = "";
    if (root == NULL)
        return str;

    std::ostringstream streamObj1, streamObj2, streamObj3;

    streamObj3 << std::fixed; // Set Fixed -Point Notation
    streamObj3 << std::setprecision(1); // Set precision to 2 digits
    streamObj3 << abs(root->bbx.angleFromParent); //Add double to stream
    string turn_angle = streamObj3.str(); // Get string from output string stream

    streamObj1 << std::fixed; // Set Fixed -Point Notation
    streamObj1 << std::setprecision(1); // Set precision to 2 digits
    double scaler = root->bbx.height / m_min_len;
    streamObj1 << scaler; //Add double to stream
    string turn_scaler = streamObj1.str(); // Get string from output string stream

    streamObj2 << std::fixed; // Set Fixed -Point Notation
    streamObj2 << std::setprecision(1); // Set precision to 2 digits
    streamObj2 << abs(root->strahler_number); //Add double to stream
    string strahler_number = streamObj2.str(); // Get string from output string stream

    if (abs(root->bbx.angleFromParent) < m_mianBranchAngle_thrs) {
        if (with_paras) {
            string prefix = "F(" + turn_scaler + ")";
            str += prefix;
        }
        else {
            str += "F";
        }

        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_grammar_forPaper(root->children[i], with_paras);
        }
        if (main_branch_idx != -1) {
            str += Nary_write_grammar_forPaper(root->children[main_branch_idx], with_paras);
        }
    }
    else if (root->bbx.angleFromParent > 0) {
        if (with_paras) {
            string prefix1 = "F(" + turn_scaler + ")";
            string prfix = "[+(" + turn_angle + ")" + prefix1;
            str += prfix;
        }
        else {
            str += "[+F";
        }

        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_grammar_forPaper(root->children[i], with_paras);
        }
        if (main_branch_idx != -1) {
            str += Nary_write_grammar_forPaper(root->children[main_branch_idx], with_paras);
        }
        str += "]";
    }
    else if (root->bbx.angleFromParent < 0) {
        if (with_paras) {
            string prefix1 = "F(" + turn_scaler + ")";
            string prfix = "[-(" + turn_angle + ")" + prefix1;
            str += prfix;
        }
        else {
            str += "[-F";
        }

        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_grammar_forPaper(root->children[i], with_paras);
        }
        if (main_branch_idx != -1) {
            str += Nary_write_grammar_forPaper(root->children[main_branch_idx], with_paras);
        }
        str += "]";
    }
    return str;
}

string Nary_write_grammar(MultiwayTreeNode* root, bool with_paras) {
    string str = "";
    if (root == NULL)
        return str;

    std::ostringstream streamObj1, streamObj2, streamObj3;

    streamObj3 << std::fixed; // Set Fixed -Point Notation
    streamObj3 << std::setprecision(1); // Set precision to 2 digits
    streamObj3 << abs(root->bbx.angleFromParent); //Add double to stream
    string turn_angle = streamObj3.str(); // Get string from output string stream

    streamObj1 << std::fixed; // Set Fixed -Point Notation
    streamObj1 << std::setprecision(1); // Set precision to 2 digits
    double scaler = root->bbx.height / m_min_len;
    streamObj1 << scaler; //Add double to stream
    string turn_scaler = streamObj1.str(); // Get string from output string stream

    streamObj2 << std::fixed; // Set Fixed -Point Notation
    streamObj2 << std::setprecision(1); // Set precision to 2 digits
    streamObj2 << abs(root->strahler_number); //Add double to stream
    string strahler_number = streamObj2.str(); // Get string from output string stream

    if (abs(root->bbx.angleFromParent) < m_mianBranchAngle_thrs) {
        if (with_paras) {
            string prefix = "<*(" + turn_scaler + ")&(" + strahler_number + ")F>";
            str += prefix;
        }
        else {
            str += "F";
        }

        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_grammar(root->children[i], with_paras);
        }
        if (main_branch_idx != -1) {
            str += Nary_write_grammar(root->children[main_branch_idx], with_paras);
        }
    }
    else if (root->bbx.angleFromParent > 0) {
        if (with_paras) {
            string prefix1 = "<*(" + turn_scaler + ")&(" + strahler_number + ")F>";
            string prfix = "[+(" + turn_angle + ")" + prefix1;
            str += prfix;
        }
        else {
            str += "[+F";
        }

        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_grammar(root->children[i], with_paras);
        }
        if (main_branch_idx != -1) {
            str += Nary_write_grammar(root->children[main_branch_idx], with_paras);
        }
        str += "]";
    }
    else if (root->bbx.angleFromParent < 0) {
        if (with_paras) {
            string prefix1 = "<*(" + turn_scaler + ")&(" + strahler_number + ")F>";
            string prfix = "[-(" + turn_angle + ")" + prefix1;
            str += prfix;
        }
        else {
            str += "[-F";
        }

        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_grammar(root->children[i], with_paras);
        }
        if (main_branch_idx != -1) {
            str += Nary_write_grammar(root->children[main_branch_idx], with_paras);
        }
        str += "]";
    }
    return str;
}

double Nary_get_scalar_para(MultiwayTreeNode* root) {
    double scalar;

    if (root != NULL) {
        if (root->children.size() > 0) {
            double max_scale = -1000;
            for (int i = 0; i < root->children.size(); i++) {
                double s = root->bbx.height / root->children[i]->bbx.height;
                max_scale = std::max(max_scale, s);
                if (!root->children[i]->main_branch) {
                    m_average_branch_angle_ += std::abs(root->children[i]->bbx.angleFromParent);
                    m_tree_node_size_angle_++;
                }
            }
            scalar = std::max(max_scale, 1.0);
            m_tree_node_size_++;
        }
        else {
            return 0.0;
        }
    }
    else {
        return 0.0;
    }

    for (int i = 0; i < root->children.size(); i++) {
        scalar += Nary_get_scalar_para(root->children[i]);
    }
    return scalar;
}

string Nary_find_repetitions(MultiwayTreeNode* node, unordered_map<string, Nary_repetition_node>& m)
{
    if (!node)
        return "";

    string str = "(";
    int main_branch_idx = -1;
    for (int i = 0; i < node->children.size(); i++) {
        if (node->children[i]->main_branch) {
            main_branch_idx = i;
            continue;
        }
        if (node->cluster_level == node->children[i]->cluster_level) {
            str += node->children[i]->turn_indicator;
            str += Nary_find_repetitions(node->children[i], m);
        }

    }
    if (main_branch_idx != -1 && node->cluster_level == node->children[main_branch_idx]->cluster_level) {
        str += node->children[main_branch_idx]->turn_indicator;
        str += Nary_find_repetitions(node->children[main_branch_idx], m);
    }
    str += ")";

    std::set<MultiwayTreeNode*>::iterator siter;
    siter = m_selected_repetitions_nary_nodes.find(node);
    if (siter == m_selected_repetitions_nary_nodes.end()) {
        m[str].oocur_time++;
        m[str].parent_node.push_back(node);
    }


    return str;
}

string Nary_select_prefer_repetition(unordered_map<string, Nary_repetition_node>& m, double weight, bool& find) {
    string select_str = "";
    std::unordered_map<string, Nary_repetition_node>::iterator iter;
    //now we only get the largest sub-tree, but we prefer the one has selected before
    int max_node_size = -1;

    std::vector<string> selected_strs;
    std::vector<int> node_sizes;
    int max_node_size_in_selected = -1;
    int max_node_pos_in_selected = 0;

    std::pair<std::set<string>::iterator, bool> ret;

    for (iter = m.begin(); iter != m.end(); iter++)
    {
        string str = iter->first;
        int num = 0;
        for (int j = 0; j < str.length(); j++) {
            if (str.at(j) == '(' || str.at(j) == ')' || str.at(j) == '-') {
                num++;
            }
        }
        int group_node_size = str.length() - num + 1;
        iter->second.group_node_size = group_node_size;

        std::set<string>::iterator siter;
        siter = m_selected_repetitions.find(str);

        if ((iter->second.oocur_time > 1 && group_node_size >= 3) || siter != m_selected_repetitions.end()) {
            if (group_node_size > max_node_size) {
                max_node_size = group_node_size;
                select_str = iter->first;
            }
            if (siter != m_selected_repetitions.end()) {
                selected_strs.push_back(iter->first);
                node_sizes.push_back(group_node_size);
                if (group_node_size > max_node_size_in_selected) {
                    max_node_size_in_selected = group_node_size;
                    max_node_pos_in_selected = selected_strs.size() - 1;
                }
            }
        }
    }

    if (selected_strs.empty()) {
        find = false;
        return select_str;
    }
    else {
        if (max_node_size_in_selected == max_node_size) {
            find = true;
            return select_str;
        }
        else {
            find = true;
            return selected_strs[max_node_pos_in_selected];
        }
    }

    return select_str;
}

void Nary_perform_clustring(MultiwayTreeNode* node, bool find) {
    bool collaps_happen = true, move_happen = true;
    int max_iter = 10;
    for (int i = 0; i < max_iter; i++) {
        collaps_happen = Nary_perform_collaps_leaf(node, find);
        move_happen = Nary_perform_move_leaf(node, find);
        if (!collaps_happen && !move_happen) break;
    }
}

bool Nary_perform_move_leaf(MultiwayTreeNode* node, bool find) {
    bool move_happen = false, move3 = false;
    if (node == NULL)
        return false;

    int main_branch_idx = -1;
    for (int i = 0; i < node->children.size(); i++) {
        if (node->children[i]->main_branch) {
            main_branch_idx = i;
            continue;
        }
        bool move_1 = Nary_perform_move_leaf(node->children[i], find);
        move_happen = (move_happen || move_1);
    }
    if (main_branch_idx != -1) {
        bool move_2 = Nary_perform_move_leaf(node->children[main_branch_idx], find);
        move_happen = (move_happen || move_2);
    }

    if (node->parent != NULL && node->cluster_level >= 1) {
        TreeNode* left = NULL, * right = NULL;

        std::vector<MultiwayTreeNode*> children_tmp;
        int main_branch_idx = -1;
        for (int i = 0; i < node->children.size(); i++) {
            if (node->children[i]->cluster_level >= 1) {
                if (find && node->cluster_level != node->children[i]->cluster_level) {
                    children_tmp.push_back(node->children[i]);
                }
            }
        }
        if (children_tmp.size() == 0) return false;
        if (node->parent != NULL && node->cluster_level == node->parent->cluster_level) {
            for (int j = 0; j < children_tmp.size(); j++) {
                node->parent->children.push_back(children_tmp[j]);
                children_tmp[j]->parent = node->parent;
            }
            node->parent->children.erase(std::find(node->parent->children.begin(), node->parent->children.end(), node));
            //DestroyTree(node);
            move3 = true;
        }
    }

    return (move_happen || move3);
}

bool Nary_perform_collaps_leaf(MultiwayTreeNode* node, bool find) {
    bool collaps_happen = false, collaps3 = false;
    if (node == NULL)
        return false;

    int main_branch_idx = -1;
    for (int i = 0; i < node->children.size(); i++) {
        if (node->children[i]->main_branch) {
            main_branch_idx = i;
            continue;
        }
        bool collaps_1 = Nary_perform_collaps_leaf(node->children[i], find);
        collaps_happen = (collaps_happen || collaps_1);
    }
    if (main_branch_idx != -1) {
        bool collaps_2 = Nary_perform_collaps_leaf(node->children[main_branch_idx], find);
        collaps_happen = (collaps_happen || collaps_2);
    }
    if (node->children.size() == 0) {
        if (!find && node->alphabet_symbol == node->parent->alphabet_symbol && node->cluster_id == node->parent->cluster_id) {
            node->parent->children.erase(std::find(node->parent->children.begin(), node->parent->children.end(), node));
            DestroyTree(node);
            collaps3 = true;
        }
        if (find && node->alphabet_symbol == node->parent->alphabet_symbol && node->cluster_level == node->parent->cluster_level) {
            node->parent->children.erase(std::find(node->parent->children.begin(), node->parent->children.end(), node));
            DestroyTree(node);
            collaps3 = true;
        }
    }

    return (collaps_happen || collaps3);
}

bool Nary_update_cluster_infomation(MultiwayTreeNode* node, int cluster_id, string symb) {
    if (node == NULL) {
        return true;
    }
    bool update = true;
    node->alphabet_symbol = symb;
    node->cluster_id = cluster_id;
    node->cluster_level += 1;

    int main_branch_idx = -1;
    for (int i = 0; i < node->children.size(); i++) {
        if (node->children[i]->main_branch) {
            main_branch_idx = i;
            continue;
        }
        bool updateChild = Nary_update_cluster_infomation(node->children[i], cluster_id, symb);
    }
    if (main_branch_idx != -1) {
        bool updateChild = Nary_update_cluster_infomation(node->children[main_branch_idx], cluster_id, symb);
    }

    return update;
}

ruleSuccessor Nary_write_rules(MultiwayTreeNode* root, bool new_tree) {
    ruleSuccessor rule;
    string str = "";
    if (root == NULL)
        return rule;

    if (abs(root->bbx.angleFromParent) < m_mianBranchAngle_thrs) {
        str += root->alphabet_symbol;
        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_rules(root->children[i], false).successor;
        }
        if (main_branch_idx != -1) {
            str += Nary_write_rules(root->children[main_branch_idx], false).successor;
        }
    }
    else if (root->bbx.angleFromParent > 0) {
        if (new_tree) {
            str += root->alphabet_symbol;
        }
        else {
            str += "[+" + root->alphabet_symbol;
        }
        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_rules(root->children[i], false).successor;
        }
        if (main_branch_idx != -1) {
            str += Nary_write_rules(root->children[main_branch_idx], false).successor;
        }
        if (!new_tree) {
            str += "]";
        }
    }
    else if (root->bbx.angleFromParent < 0) {
        if (new_tree) {
            str += root->alphabet_symbol;
        }
        else {
            str += "[-" + root->alphabet_symbol;
        }
        int main_branch_idx = -1;
        for (int i = 0; i < root->children.size(); i++) {
            if (root->children[i]->main_branch) {
                main_branch_idx = i;
                continue;
            }
            str += Nary_write_rules(root->children[i], false).successor;
        }
        if (main_branch_idx != -1) {
            str += Nary_write_rules(root->children[main_branch_idx], false).successor;
        }
        if (!new_tree) {
            str += "]";
        }
    }
    rule.successor = str;
    return rule;
}

void Nary_generate_conformal_grammar(MultiwayTreeNode* root, unordered_map<string, Nary_repetition_node>& m) {

    if (root == NULL) {
        return;
    }
    int max_iter = 5;
    int type_id = 0;
    double weight = 0.5;
    int real_iters = 0;
    for (int k = 0; k < max_iter; k++) {
        bool new_repetition = false;
        m.clear();
        Nary_find_repetitions(root, m);
        std::unordered_map<string, Nary_repetition_node>::iterator iter;
        for (iter = m.begin(); iter != m.end(); iter++) {
            if (iter->second.oocur_time > 1 && iter->second.oocur_time > iter->second.last_groups_numer) {
                new_repetition = true;
            }
        }
        bool find_again = false;
        string str = Nary_select_prefer_repetition(m, weight, find_again);
        if (str == "") {
            if (root->parent != NULL && !root->old_repetition) {
                ruleSuccessor rule = Nary_write_rules(root, true);
                m_rules[alphabet[m_alphabet_pointer]].push_back(rule);
            }
            break;
        }
        m_selected_repetitions.insert(str);

        Nary_repetition_node r_node = m[str];
        int last_groups_numer = r_node.last_groups_numer;
        if (!find_again) {
            m_used_symbols[str] = alphabet[m_alphabet_pointer];
        }

        for (int i = last_groups_numer; i < r_node.parent_node.size(); i++) {
            r_node.parent_node[i]->old_repetition = find_again;
            m_selected_repetitions_nary_nodes.insert(r_node.parent_node[i]);
            if (i == 0) {
                Nary_generate_conformal_grammar(r_node.parent_node[i], m);
            }
            if (find_again) {
                bool update = Nary_update_cluster_infomation(r_node.parent_node[i], i, m_used_symbols[str]); //update the node information;
            }
            else {
                bool update = Nary_update_cluster_infomation(r_node.parent_node[i], i, alphabet[m_alphabet_pointer]); //update the node information;
            }

            Nary_perform_clustring(r_node.parent_node[i], r_node.parent_node[i]->old_repetition); // perform real clustring on the sub-tree, i.e., collpase each repetition to one node 
        }
        if (!find_again) m_alphabet_pointer++;

        real_iters++;
    }
}

int compute_grammar_length(std::vector<ruleProductions>& productions, std::unordered_map<string, int>& symbols_info) {
    int symbols_length = 0;
    int grammar_length = 0;
    for (int j = 0; j < productions.size(); j++) {
        string left = productions[j].precessor;
        std::unordered_map<string, int>::iterator it = symbols_info.find(left);
        if (it->second > 0) {
            symbols_length += left.length();
        }

        string right = productions[j].successor;
        grammar_length += right.length();
    }
    grammar_length += symbols_length;

    return grammar_length;
}

bool merge_two_rules(std::vector<ruleProductions>& productions, int ri, int rj, std::unordered_map<string, int>& symbols_info) {
    bool merged = false;
    string left1 = productions[ri].precessor;
    string left2 = productions[rj].precessor;
    for (int k = 0; k < productions.size(); k++) {
        if (productions[k].precessor == left2) {
            productions[k].precessor = left1;
            symbols_info[left2] -= 1;
            symbols_info[left1] += 1;
            merged = true;
        }
        string rhs = productions[k].successor;
        size_t pos = 0;
        while ((pos = rhs.find(left2, pos)) != std::string::npos) {
            rhs.replace(pos, left2.length(), left1);
            pos += left1.length();
        }
        productions[k].successor = rhs;
    }

    std::vector<ruleProductions> productions_copy;
    std::set<int> deleted_ids;
    for (int k = 0; k < productions.size(); k++) {
        for (int m = k + 1; m < productions.size(); m++) {
            if (k == m) continue;
            string left1 = productions[k].precessor;
            string right1 = productions[k].successor;

            string left2 = productions[m].precessor;
            string right2 = productions[m].successor;
            if (right1 == right2) {
                bool findLeft1 = (right1.find(left1) == std::string::npos) ? false : true;
                bool findLeft2 = (right2.find(left2) == std::string::npos) ? false : true;
                if (findLeft1) {
                    deleted_ids.insert(m);
                }
                else if (findLeft2) {
                    deleted_ids.insert(k);
                }
                else {
                    deleted_ids.insert(m);
                }
            }
        }
    }
    for (int k = 0; k < productions.size(); k++) {
        std::set<int>::iterator iter = deleted_ids.find(k);
        if (iter == deleted_ids.end()) {
            productions_copy.push_back(productions[k]);
        }
    }
    productions.assign(productions_copy.begin(), productions_copy.end());

    return merged;
}

void appendTextToScrollFieldFromCpp(const std::string& text) {
    MString mText(text.c_str());

    MString cmd = "appendTextToGrammarScrollField(\"" + mText + "\");";

    MGlobal::executeCommand(cmd);
}

bool grammar_induction() {
    bool IsSuccesed = false;
    std::vector<ruleProductions> productions;
    newAssociativeArray::const_iterator iter;
    std::unordered_map<string, int> symbols_infor;
    for (iter = m_rules.begin(); iter != m_rules.end(); ++iter)
    {
        string left = iter->first;
        symbols_infor[left] = 0;
        vector<ruleSuccessor> sucs = iter->second;
        for (int i = 0; i < sucs.size(); i++) {
            string right = sucs[i].successor;
            ruleProductions rule(left, right);
            productions.push_back(rule);
            symbols_infor[left] += 1;
        }
    }

    int max_iter = 10;
    double edit_dis_threshold = 6;
    for (int i = 0; i < max_iter; i++) {
        bool merged = false;
        //compute current grammar length
        int grammar_length = compute_grammar_length(productions, symbols_infor);

        double min_loss = 100000;
        std::vector<ruleProductions> productions_new;
        std::vector<ruleProductions> productions_final;
        for (int j = 0; j < productions.size(); j++)
        {
            string left1 = productions[j].precessor;
            string right1 = productions[j].successor;
            for (int k = 0; k < productions.size(); k++) {
                bool update = false;
                if (j == k) continue;
                string left2 = productions[k].precessor;
                string right2 = productions[k].successor;

                if (left1 == left2) continue;

                double min_edit_distance;
                if (left2 == "S") {
                    min_edit_distance = edit_distance_DP(right1, right2);
                }
                else {
                    min_edit_distance = edit_distance_DP(right2, right1);
                }

                if (productions.size() == 2) {
                    min_edit_distance = 0;
                }
                if (min_edit_distance > edit_dis_threshold) {
                    continue;
                }
                productions_new.clear();
                productions_new.assign(productions.begin(), productions.end()); // copy all rules

                if (left2 == "S") {
                    merged = merge_two_rules(productions_new, k, j, symbols_infor);
                }
                else {
                    merged = merge_two_rules(productions_new, j, k, symbols_infor);
                }
                int grammar_length_new = compute_grammar_length(productions_new, symbols_infor);
                double loss = (grammar_length_new - grammar_length) + min_edit_distance;
                if (loss < min_loss) {
                    min_loss = loss;
                    productions_final.clear();
                    productions_final.assign(productions_new.begin(), productions_new.end());
                    update = true;
                }
                if (update) {
                    productions.assign(productions_final.begin(), productions_final.end());
                    productions_final.clear();
                }
            }
        }
        if (merged == false) break;
    }

    //output 
    std::cout << "After grammar generalization..." << std::endl;
    std::string cmd = "scrollField -e -text \"" + std::string("S") + "\" $grammarScrollField;";
    MGlobal::executeCommand(cmd.c_str());
    for (int k = 0; k < productions.size(); k++) {

        string right = productions[k].successor;
        bool has_non_terminals = false;
        for (int i = 0; i < right.length(); i++) {
            if (!(right.at(i) == '(' || right.at(i) == ')' || right.at(i) == '+' || right.at(i) == '-' || right.at(i) == '[' || right.at(i) == ']' || right.at(i) == 'F')) {
                has_non_terminals = true;
                break;
            }
        }
        if (!has_non_terminals) continue;

        if (symbols_infor[productions[k].precessor] > 1) {
            IsSuccesed = true;
            string::size_type idx = productions[k].successor.find(productions[k].precessor);
            if (idx != string::npos) {
                std::cout << productions[k].precessor << "->" << productions[k].successor << std::endl;
                string myGrammar = productions[k].precessor + "->" + productions[k].successor;
                appendTextToScrollFieldFromCpp(myGrammar);
            }
            continue;
        }

        std::cout << productions[k].precessor << "->" << productions[k].successor << std::endl;
        string myGrammar = productions[k].precessor + "->" + productions[k].successor;
        appendTextToScrollFieldFromCpp(myGrammar);
    }
    return IsSuccesed;
}

double edit_distance_DP(string str1, string str2)
{
    // Create a table to store results of subproblems 
    int m = str1.length(), n = str2.length();
    double** dp;
    dp = new double* [m + 1];
    for (int i = 0; i <= m; i++)
    {
        dp[i] = new double[n + 1];
    }

    for (int i = 0; i <= m; i++)
    {
        if (i == 9) {
            int dddd = 0;
        }
        for (int j = 0; j <= n; j++)
        {
            if (i == 0) {
                dp[i][j] = j;
            } 
            else if (j == 0) {
                dp[i][j] = i; // Min. operations = i 
            }
            // If last characters are same, ignore last char and recur for remaining string 
            else if (str1[i - 1] == str2[j - 1]) {
                dp[i][j] = dp[i - 1][j - 1];
            }
            // If the last character is different, consider all possibilities and find the minimum 
            else {
                double insert_op = dp[i][j - 1];
                double remove_op = dp[i - 1][j];
                double replace_op = dp[i - 1][j - 1];
                dp[i][j] = 1 + std::min(insert_op, std::min(remove_op, replace_op));
            }
        }
    }

    double distance = dp[m][n];
    //release
    for (int i = 0; i < m + 1; i++)
    {
        delete[] dp[i];
    }
    delete dp;

    return distance;
}

void updateLocalCoordinates(BoundingBox& bbx) {
    bbx.center_position_LC = R2Vector(bbx.center_position.X(), -bbx.center_position.Y());
    bbx.l_t_corner_LC = R2Vector(bbx.l_t_corner.X(), -bbx.l_t_corner.Y());
    bbx.r_t_corner_LC = R2Vector(bbx.r_t_corner.X(), -bbx.r_t_corner.Y());
    bbx.r_b_corner_LC = R2Vector(bbx.r_b_corner.X(), -bbx.r_b_corner.Y());
    bbx.l_b_corner_LC = R2Vector(bbx.l_b_corner.X(), -bbx.l_b_corner.Y());
    bbx.direction_LC = R2Vector(bbx.direction.X(), -bbx.direction.Y());
    bbx.direction_LC_opp = R2Vector(bbx.direction_opp.X(), -bbx.direction_opp.Y());
    bbx.angleFromY_LC = RAD2DEG(angleFromY_LC(bbx.direction_LC));
    bbx.angleFromY_LC_opp = RAD2DEG(angleFromY_LC(bbx.direction_LC_opp));
}

void calculateRotatedBoundingBox(BoundingBox& bbx) {
    QPointF corners[4] = {
        QPointF(-bbx.width * 0.5, -bbx.height * 0.5),
        QPointF(bbx.width * 0.5, -bbx.height * 0.5),
        QPointF(bbx.width * 0.5, bbx.height * 0.5),
        QPointF(-bbx.width * 0.5, bbx.height * 0.5)
    };

    QPointF dir(0, 1);
    for (auto& corner : corners) {
        corner = my_rotate(corner, bbx.angleFromY);
    }

    bbx.l_t_corner = R2Vector(bbx.center_position.X() + corners[0].x(), bbx.center_position.Y() + corners[0].y());
    bbx.r_t_corner = R2Vector(bbx.center_position.X() + corners[1].x(), bbx.center_position.Y() + corners[1].y());
    bbx.r_b_corner = R2Vector(bbx.center_position.X() + corners[2].x(), bbx.center_position.Y() + corners[2].y());
    bbx.l_b_corner = R2Vector(bbx.center_position.X() + corners[3].x(), bbx.center_position.Y() + corners[3].y());
    dir = my_rotate(dir, bbx.angleFromY);
    bbx.direction = R2Vector(dir.x(), dir.y());
    bbx.direction_opp = R2Vector(-dir.x(), -dir.y());
    updateLocalCoordinates(bbx);
}

float normalizeAngle(float angle) {
    // Normalize an angle to the range -180 to 180
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
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

    removeCycles();
    extractMinimalSpanningTree();
    buildNaryTree();

    return MS::kSuccess;
}


void handleCyclesInGraph() {
    auto cycles = udgcd::FindCycles<UndirectedGraph, vertex_t>(m_graph);
    udgcd::PrintPaths(std::cout, cycles);
    MGlobal::displayInfo("Handled cycles in the graph.");
}

bool shouldAddEdge(const BoundingBox& bbx1, const BoundingBox& bbx2, int index1, int index2) {
    double pair_weight = compute_relative_distance(bbx1, bbx2);
    return pair_weight < 1.0;
}

void logEdgeAddition(bool condition, bool intersect) {
    MString message = condition ? "True: " : "False: ";
    message += intersect ? "0" : "1";

    MGlobal::displayInfo(message);
}

void initializeGraph() {
    m_graph.clear();
    double minY = std::numeric_limits<double>::max();
    for (int i = 0; i < m_bbx_parse.size(); ++i) {
        if (m_bbx_parse[i].center_position_LC.Y() < minY) {
            minY = m_bbx_parse[i].center_position_LC.Y();
            m_root_id = i;
        }
    }
    MGlobal::displayInfo(MString("Initialized graph with root ID: ") + m_root_id);
}

void addEdgesToGraph() {
    std::vector<pairwise_bbx> adjacentBoundingBoxes;
    for (int i = 0; i < m_bbx_parse.size(); ++i) {
        for (int j = 0; j < m_bbx_parse.size(); ++j) {
            if (i == j) continue;
            if (shouldAddEdge(m_bbx_parse[i], m_bbx_parse[j], i, j)) {
                pairwise_bbx pb(i, j, compute_relative_distance(m_bbx_parse[i], m_bbx_parse[j]));
                if (std::find(adjacentBoundingBoxes.begin(), adjacentBoundingBoxes.end(), pb) == adjacentBoundingBoxes.end()) {
                    adjacentBoundingBoxes.push_back(pb);
                    logEdgeAddition(true, check_bbox_intersect(m_bbx_parse[i], m_bbx_parse[j]));
                }
            }
        }
    }

    for (auto& edge : adjacentBoundingBoxes) {
        boost::add_edge(edge.i, edge.j, edge.weight, m_graph);
    }
}


// ---------------------------------------------------------------------------------------------------------------------------------------------------
// Main Logics Functions
void ImportImageCmd::parseBoundingBoxData(const std::string& filename) {
    // Prepare filenames for image and bounding box data
    m_save_image_name = filename.substr(0, filename.size() - 4) + "_bbx.jpg";
    std::string bbox_file = filename.substr(0, filename.size() - 3) + "txt";

    // Open the bounding box data file
    std::ifstream in(bbox_file);
    if (!in) {
        std::cerr << "Failed to open file: " << bbox_file << std::endl;
        return;
    }

    m_bbx_parse.clear();
    m_min_len = std::numeric_limits<float>::max();

    // Variables for parsing the bounding box data
    std::string class_name;
    float score, x_c, y_c, w, h, angle;

    while (in >> class_name >> score >> x_c >> y_c >> w >> h >> angle) {
        BoundingBox bbx;
        bbx.center_position = R2Vector(x_c, y_c);
        bbx.label_id = get_class_id(class_name);
        // Assign width and height based on orientation
        if (w > h) {
            std::swap(w, h);
            bbx.angleFromY = normalizeAngle(-angle + 90);
            bbx.angleFromY_opp = normalizeAngle(-(angle + 90));
        }
        else {
            bbx.angleFromY = normalizeAngle(-angle - 180);
            bbx.angleFromY_opp = normalizeAngle(-angle);
        }
        bbx.width = w;
        bbx.height = h;

        // Track minimum bounding box height
        m_min_len = std::min(m_min_len, bbx.height);

        // Calculate rotated corners and direction vectors
        calculateRotatedBoundingBox(bbx);

        m_bbx_parse.push_back(bbx);
    }

    in.close();

    // Debug information output
    if (Debug_parseBoundingBoxData) {
        for (size_t i = 0; i < m_bbx_parse.size(); ++i) {
            const auto& bbx = m_bbx_parse[i];
            std::cout << "BBX " << (i + 1) << ": Center (" << bbx.center_position.X() << ", "
                << bbx.center_position.Y() << "), Width: " << bbx.width
                << ", Height: " << bbx.height << ", Angle: " << bbx.angleFromY << std::endl;
        }
    }
}

void ImportImageCmd::buildGraph() {
    initializeGraph();  // Initialize the graph and identify the root node based on bounding box positions
    addEdgesToGraph();  // Check bounding box intersections and weights, then add edges accordingly
    handleCyclesInGraph();  // Detect and handle cycles in the graph
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
                        min_distance = nodes.size();
                        indx = k;
                    }
                }
                if (indx == 0) {
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
    std::vector<MultiwayTreeNode*> nodes_list;
    std::vector<bool> visited;
    std::vector<int> parents;
    //build tree nodes
    for (int i = 0; i < m_bbx_parse.size(); i++) {
        BoundingBox cur_bbx = m_bbx_parse[i];
        MultiwayTreeNode* cur_node = CreateNaryTreeNode(m_bbx_parse[i]);
        cur_node->bbx_index = i + 1;
        nodes_list.push_back(cur_node);
        visited.push_back(false);
    }
    //consturct the tree
    MultiwayTreeNode* root_node = nodes_list[m_root_id];
    root_node->bbx.angleFromParent = root_node->bbx.angleFromY_LC;
    visited[m_root_id] = true;

    queue<MultiwayTreeNode*> qt;
    qt.push(root_node);
    typedef boost::property_map<UndirectedGraph, boost::vertex_index_t>::type IndexMap;
    IndexMap index = get(boost::vertex_index, m_graph);

    while (!qt.empty()) {
        MultiwayTreeNode* cur_node = qt.front();
        qt.pop();
        if (cur_node) {
            UndirectedGraph::adjacency_iterator vit, vend;
            std::vector<MultiwayTreeNode*> children_list;
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
    Nary_compute_strahler_number(root_node);
    bool output_strahler_number = true;

    //write expansion grammer
    MGlobal::displayInfo("Expansion grammar (input string) with parameters: ");
    string expansion_grammer_withParas;
    //expansion_grammer_withParas = Nary_write_grammar(root_node, true);
    expansion_grammer_withParas = Nary_write_grammar_forPaper(root_node, true);
    MString expansionGrammarWithParams(expansion_grammer_withParas.c_str());
    MGlobal::displayInfo(expansionGrammarWithParams);

    MGlobal::displayInfo("Expansion grammar (input string) without parameters: ");
    string expansion_grammer = Nary_write_grammar(root_node, false);
    MString expansionGrammar(expansion_grammer.c_str());
    MGlobal::displayInfo(expansionGrammar);

    //get the scale and branching angle parameters
    m_tree_node_size_ = 0;
    m_average_branch_angle_ = 0.0;
    m_tree_node_size_angle_ = 0;
    double scalar_para = Nary_get_scalar_para(root_node);
    scalar_para /= m_tree_node_size_;
    m_average_branch_angle_ /= m_tree_node_size_angle_;

    m_rules.clear();


        //inference using our method
        m_alphabet_pointer = 2;
        unordered_map<string, Nary_repetition_node> m;
        m_selected_repetitions.clear();
        m_selected_repetitions_nary_nodes.clear();
        m_used_symbols.clear();
        Nary_generate_conformal_grammar(root_node, m);
        // write the last rule
        ruleSuccessor rule = Nary_write_rules(root_node, true);
        m_rules[alphabet[1]].push_back(rule);

        MString lastRuleInfo(alphabet[1].c_str());
        string clastRuleInfo = alphabet[1];

        lastRuleInfo += " -> ";
        clastRuleInfo += " -> ";

        string suss = rule.successor;
        clastRuleInfo += suss;
        lastRuleInfo += suss.c_str();
        
        MGlobal::displayInfo(lastRuleInfo);

        MString mRuleSuccessor(rule.successor.c_str()); // Convert to MString
        MString header("\"");
        MString newline("\\nF -> ");

        // Proper concatenation using MString
        MString SpawnedGrammar = header + expansionGrammar + newline + mRuleSuccessor + "\"";

    //print grammer
    MGlobal::displayInfo("The compact conformal grammar: ");
    newAssociativeArray::const_iterator iter;

    int compact_grammar_len = 0;
    for (iter = m_rules.begin(); iter != m_rules.end(); ++iter)
    {
        string left = iter->first;
        vector<ruleSuccessor> sucs = iter->second;
        for (int i = 0; i < sucs.size(); i++) {
            string right = sucs[i].successor;
            MString ruleInfo(left.c_str());
            ruleInfo += " = ";
            ruleInfo += right.c_str();

            // Print using MGlobal
            MGlobal::displayInfo(ruleInfo);

            // Calculate the length
            compact_grammar_len += left.length();
            compact_grammar_len += right.length();
        }
    }

    //merge similar rules
    if (grammar_induction() == false) {
        for (iter = m_rules.begin(); iter != m_rules.end(); ++iter)
        {
            string left = iter->first;
            vector<ruleSuccessor> sucs = iter->second;
            for (int i = 0; i < sucs.size(); i++) {
                string right = sucs[i].successor;
                MString ruleInfo(left.c_str());
                ruleInfo += " = ";
                ruleInfo += right.c_str();

                MGlobal::displayInfo(ruleInfo);

                appendTextToScrollFieldFromCpp(ruleInfo.asChar());
            }
        }
    }

    MGlobal::executeCommand("clearLSystemGroup()");
    MGlobal::executeCommand("callLSystemCmd()");

    std::ostringstream streamObj3;
    streamObj3 << std::fixed;
    streamObj3 << std::setprecision(2);
    streamObj3 << scalar_para;
    MString succ_MStr = "F(*" + MString(streamObj3.str().c_str()) + ")";
    MGlobal::displayInfo(MString("F->") + succ_MStr);
}