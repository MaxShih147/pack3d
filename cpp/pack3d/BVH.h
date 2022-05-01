#pragma once
#include <open3d/Open3D.h>

#include <vector>

class Tree;
typedef std::vector<Tree> Trees;
typedef open3d::geometry::AxisAlignedBoundingBox Box;
typedef std::vector<Box> Boxes;
typedef open3d::geometry::TriangleMesh Mesh;
typedef std::shared_ptr<Mesh> MeshPtr;

enum class Axis {
	AxisNone = -1,
	AxisX = 0,
	AxisY = 1,
	AxisZ = 2
};

static std::vector<Eigen::Vector3d> axisVector{
	Eigen::Vector3d(0, 0, 0),
	Eigen::Vector3d(1, 0, 0),
	Eigen::Vector3d(0, 1, 0),
	Eigen::Vector3d(0, 0, 1)
};

class Node {
public:
	Node(Box box_, Node* left_, Node* right_) : _box(box_), _left(left_), _right(right_) {}

public:
	void Flatten(Tree tree_, int boxIndex_);
	void Split(Boxes boxes_, int depth_);

private:
	Box _box;
	Node* _left;
	Node* _right;
};

class Tree {
public:
	Tree() {}
	Tree(int size_);

public:
	Tree Transform(Eigen::Matrix4d m_);
	bool Intersects(Tree b_, Eigen::Vector3d t1, Eigen::Vector3d t2);

private:
	bool intersects(Tree b_, Eigen::Vector3d t1, Eigen::Vector3d t2, int i, int j);

public:
	std::vector<Box> _boxes;
};

// Box
void Extend(Box& source_, Box target_);
bool ContainsBox(Box source_, Box target_);
bool Intersects(Box source_, Box target_);
Box Intersection(Box source_, Box target_);
Box BoxForBoxes(Boxes boxes_);
Box Offset(Box box_, double d_);
Eigen::Vector3d Anchor(Box box_, Eigen::Vector3d anchor_);
bool boxesIntersect(Box b1, Box b2, Eigen::Vector3d t1, Eigen::Vector3d t2);

// Node
Node* NewNode(Boxes boxes_, int depth_);

// Partition
std::pair<bool, bool> partitionBox(Box box_, Axis axis_, double point_);
double partitionScore(Boxes boxes_, Axis axis_, double point_, bool side_);
std::pair<Boxes, Boxes> partition(Boxes boxes_, Axis axis_, double point_, bool side_);