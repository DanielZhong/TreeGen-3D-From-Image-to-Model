/**
**/

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.)
#endif // DEG2RAD

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180./M_PI)
#endif // RAD2DEG

#ifndef _TREE_STRUCT_H_
#define _TREE_STRUCT_H_
#include <string>
#include <vector>
#include <stack>
#include <queue>
#include "R2/R2.h"

struct BoundingBox{
	R2Vector center_position, l_t_corner, r_t_corner, r_b_corner, l_b_corner, direction, direction_opp; //Coordinates in images
	R2Vector center_position_LC, l_t_corner_LC, r_t_corner_LC, r_b_corner_LC, l_b_corner_LC, direction_LC, direction_LC_opp; //Coordinates in the L-system
	double width, height;
	double angleFromY, angleFromY_LC;
	double angleFromY_opp, angleFromY_LC_opp;
	double angleFromParent;
	int group_id;
	int label_id;
	
	BoundingBox()
	{
		group_id = -1;
		angleFromY = 0;
		label_id = 0;
	}
	BoundingBox(int id) : group_id(id)
	{
		angleFromY = 0;
		label_id = 0;
	}
};

//Binary Tree
struct TreeNode
{
	TreeNode *left;
	TreeNode *middle;
	TreeNode *right;
	TreeNode *parent;
	BoundingBox bbx;
	std::vector<int> intersect_nodes;
	int bbx_index;
	bool main_branch;
	int type_id, cluster_id, cluster_size, cluster_level;
	string turn_indicator;
	string alphabet_symbol;
	bool old_repetition;

	TreeNode() : left(NULL), right(NULL), parent(NULL){
		bbx_index = -1;
		main_branch = false;
		type_id = -1;
		cluster_id = -1;
		cluster_size = 0;
		cluster_level = 0;
		turn_indicator = "";
		alphabet_symbol = "F";
		old_repetition = false;
	}

	TreeNode(BoundingBox bb) : bbx(bb), left(NULL), right(NULL), parent(NULL) {
		bbx_index = -1;
		main_branch = false;
		type_id = -1;
		cluster_id = -1;
		cluster_size = 0;
		cluster_level = 0;
		turn_indicator = "";
		alphabet_symbol = "F";
		old_repetition = false;
	}
};

static inline TreeNode *CreateTreeNode(BoundingBox bbx);
static inline void ConnectTreeNodes(TreeNode *pParent, TreeNode *pLeft, TreeNode *pRight);
static inline void ConnectTreeNodes(TreeNode *pParent, TreeNode *pLeft, TreeNode *pMiddle, TreeNode *pRight);
static inline void DestroyTree(TreeNode *pRoot);
static inline vector<vector<TreeNode *> > levelOrderNodeBottom(TreeNode *root);
//static inline vector<TreeNode *> getLeafNodes(TreeNode *root);

//Create new node
static inline TreeNode *CreateTreeNode(BoundingBox bbx)
{
	TreeNode *pNode = new TreeNode(bbx);
	return pNode;
}

//connnect the parent and children nodes
static inline void ConnectTreeNodes(TreeNode *pParent, TreeNode *pLeft, TreeNode *pRight)
{
	if (pParent != NULL)
	{
		pParent->left = pLeft;
		pParent->right = pRight;
		if (pParent->left != NULL){
			pParent->left->parent = pParent;
		}
		if (pParent->right != NULL){
			pParent->right->parent = pParent;
		}
	}
}

//connnect the parent and children nodes
static inline void ConnectTreeNodes(TreeNode *pParent, TreeNode *pLeft, TreeNode *pMiddle, TreeNode *pRight)
{
	if (pParent != NULL)
	{
		pParent->left = pLeft;
		pParent->right = pRight;
		pParent->middle = pMiddle;
		if (pParent->left != NULL){
			pParent->left->parent = pParent;
		}
		if (pParent->middle != NULL){
			pParent->middle->parent = pParent;
		}
		if (pParent->right != NULL){
			pParent->right->parent = pParent;
		}
	}
}

static inline void DestroyTree(TreeNode *pRoot)
{
	if (pRoot != NULL)
	{
		TreeNode *pLeft = pRoot->left;
		TreeNode *pRight = pRoot->right;

		delete pRoot;
		pRoot = NULL;

		DestroyTree(pLeft);
		DestroyTree(pRight);
	}
}

static inline vector<vector<TreeNode*> > levelOrderNodeBottom(TreeNode *root)
{
	vector<vector<TreeNode*> > matrix;
	if (root == NULL)
	{
		return matrix;
	}

	stack<vector<TreeNode*> > sv;
	vector<TreeNode*> temp;
	temp.push_back(root);
	sv.push(temp);

	vector<TreeNode *> path;
	path.push_back(root);

	int count = 1;
	while (!path.empty())
	{
		if (path[0]->left != NULL)
		{
			path.push_back(path[0]->left);
		}
		if (path[0]->right != NULL)
		{
			path.push_back(path[0]->right);
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
	return matrix;
}


//n-ary tree
struct MultiwayTreeNode
{
	std::vector<MultiwayTreeNode*> children;
	MultiwayTreeNode *parent;
	BoundingBox bbx;
	int bbx_index;
	bool main_branch;
	int type_id, cluster_id, cluster_size, cluster_level;
	string turn_indicator;
	string alphabet_symbol;
	bool old_repetition;
	int strahler_number; 

	MultiwayTreeNode() : parent(NULL){
		bbx_index = -1;
		main_branch = false;
		type_id = -1;
		cluster_id = -1;
		cluster_size = 0;
		cluster_level = 0;
		turn_indicator = "";
		alphabet_symbol = "F";
		old_repetition = false;
		strahler_number = 0;
	}

	MultiwayTreeNode(BoundingBox bb) : bbx(bb), parent(NULL) {
		bbx_index = -1;
		main_branch = false;
		type_id = -1;
		cluster_id = -1;
		cluster_size = 0;
		cluster_level = 0;
		turn_indicator = "";
		alphabet_symbol = "F";
		old_repetition = false;
		strahler_number = 0;
	}
};

static inline MultiwayTreeNode *CreateNaryTreeNode(BoundingBox bbx);
static inline void DestroyTree(MultiwayTreeNode *pRoot);

static inline MultiwayTreeNode *CreateNaryTreeNode(BoundingBox bbx)
{
	MultiwayTreeNode *pNode = new MultiwayTreeNode(bbx);
	return pNode;
}

static inline void DestroyTree(MultiwayTreeNode *pRoot){
	if (pRoot != NULL)
	{
		std::vector<MultiwayTreeNode*> children_copy;
		for (int i = 0; i < pRoot->children.size(); i++){
			children_copy.push_back(pRoot->children[i]);
		}

		delete pRoot;
		pRoot = NULL;

		for (int i = 0; i < children_copy.size(); i++){
			DestroyTree(children_copy[i]);
		}
	}
}

static inline void ConnectTreeNodes(MultiwayTreeNode *pParent, MultiwayTreeNode *pChild)
{
	if (pParent != NULL)
	{
		std::vector<MultiwayTreeNode*> children;

		pParent->children.push_back(pChild);
		if (pParent->children[pParent->children.size()-1] != NULL){
			pParent->children[pParent->children.size() - 1]->parent = pParent;
		}
	}
}

#endif