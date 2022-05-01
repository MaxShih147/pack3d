#include "BVH.h"

//Node::Node(Boxes boxes_, int depth_) {
//box: = fauxgl.BoxForBoxes(boxes).Offset(2.5);
//node : = &Node{ box, nil, nil }
//node.Split(boxes, depth)
//return node
//}

void Extend(Box& source_, Box target_) {

	source_.min_bound_.x() = min(source_.min_bound_.x(), target_.min_bound_.x());
	source_.min_bound_.y() = min(source_.min_bound_.y(), target_.min_bound_.y());
	source_.min_bound_.z() = min(source_.min_bound_.z(), target_.min_bound_.z());

	source_.max_bound_.x() = max(source_.max_bound_.x(), target_.max_bound_.x());
	source_.max_bound_.y() = max(source_.max_bound_.y(), target_.max_bound_.y());
	source_.max_bound_.z() = max(source_.max_bound_.z(), target_.max_bound_.z());
}

bool ContainsBox(Box source_, Box target_) {

	return 
		source_.min_bound_.x() <= target_.min_bound_.x() && source_.max_bound_.x() >= target_.max_bound_.x() &&
		source_.min_bound_.y() <= target_.min_bound_.y() && source_.max_bound_.y() >= target_.max_bound_.y() &&
		source_.min_bound_.z() <= target_.min_bound_.z() && source_.max_bound_.z() >= target_.max_bound_.z();
}

bool Intersects(Box source_, Box target_) {

	return
		!(
			source_.min_bound_.x() > target_.max_bound_.x() || source_.max_bound_.x() < target_.min_bound_.x() ||
			source_.min_bound_.y() > target_.max_bound_.y() || source_.max_bound_.y() < target_.min_bound_.y() ||
			source_.min_bound_.z() > target_.max_bound_.z() || source_.max_bound_.z() < target_.min_bound_.z());
}

Box Intersection(Box source_, Box target_) {

	Box result;

	if (!Intersects(source_, target_)) {
		return result;
	}

	Eigen::Vector3d bound_ab(
		max(source_.min_bound_.x(), target_.min_bound_.x()),
		max(source_.min_bound_.y(), target_.min_bound_.y()),
		max(source_.min_bound_.z(), target_.min_bound_.z())
	);

	Eigen::Vector3d bound_ba(
		min(source_.max_bound_.x(), target_.max_bound_.x()),
		min(source_.max_bound_.y(), target_.max_bound_.y()),
		min(source_.max_bound_.z(), target_.max_bound_.z())
	);
	
	result.min_bound_.x() = min(bound_ab.x(), bound_ba.x());
	result.min_bound_.y() = min(bound_ab.y(), bound_ba.y());
	result.min_bound_.z() = min(bound_ab.z(), bound_ba.z());

	result.max_bound_.x() = max(bound_ab.x(), bound_ba.x());
	result.max_bound_.y() = max(bound_ab.y(), bound_ba.y());
	result.max_bound_.z() = max(bound_ab.z(), bound_ba.z());

	return result;
}

Box BoxForBoxes(Boxes boxes_) {

	Box result;

	if (boxes_.empty()) {
		return result;
	}

	if (boxes_.size() == 1) {
		return boxes_[0];
	}

	double minX = boxes_[0].min_bound_.x();
	double minY = boxes_[0].min_bound_.y();
	double minZ = boxes_[0].min_bound_.z();
	double maxX = boxes_[0].max_bound_.x();
	double maxY = boxes_[0].max_bound_.y();
	double maxZ = boxes_[0].max_bound_.z();

	for (auto i = 1; i < boxes_.size(); ++i) {
		minX = min(minX, boxes_[i].min_bound_.x());
		minY = min(minY, boxes_[i].min_bound_.y());
		minZ = min(minZ, boxes_[i].min_bound_.z());
		maxX = max(maxX, boxes_[i].max_bound_.x());
		maxY = max(maxY, boxes_[i].max_bound_.y());
		maxZ = max(maxZ, boxes_[i].max_bound_.z());
	}

	result.min_bound_.x() = minX;
	result.min_bound_.y() = minY;
	result.min_bound_.z() = minZ;
	result.max_bound_.x() = maxX;
	result.max_bound_.y() = maxY;
	result.max_bound_.z() = maxZ;
	
	return result;
}

Box Offset(Box box_, double d_) {
	return Box(
		Eigen::Vector3d(
			box_.min_bound_.x() - d_,
			box_.min_bound_.y() - d_,
			box_.min_bound_.z() - d_),
		Eigen::Vector3d(
			box_.max_bound_.x() + d_,
			box_.max_bound_.y() + d_,
			box_.max_bound_.z() + d_));
}

Eigen::Vector3d Anchor(Box box_, Eigen::Vector3d anchor_) {

	auto size = box_.GetExtent();
	size.x() *= anchor_.x();
	size.y() *= anchor_.y();
	size.z() *= anchor_.z();
	
	return box_.min_bound_ + size;
}

Node* NewNode(Boxes boxes_, int depth_) {

	Box box = Offset(BoxForBoxes(boxes_), 2.5);
	Node* node = new Node(box, nullptr, nullptr);
	node->Split(boxes_, depth_);
	return node;
}

void Node::Flatten(Tree tree_, int boxIndex_) {

	tree_._boxes[boxIndex_] = this->_box;

	if (this->_left != nullptr) {
		this->_left->Flatten(tree_, boxIndex_ * 2 + 1);
	}

	if (this->_right != nullptr) {
		this->_right->Flatten(tree_, boxIndex_ * 2 + 2);
	}
}

void Node::Split(Boxes boxes_, int depth_) {

	if (depth_ == 0) {
		return;
	}

	auto box = this->_box;
	auto best = box.Volume();
	Axis bestAxis = Axis::AxisNone;
	double bestPoint = 0.0;
	bool bestSide = false;
	const int N = 16;

	for (auto s : { 0, 1 }) {

		bool side = s == 1;

		for (auto i = 1; i < N; ++i) {
			
			double p = double(i) / double(N);
			double x = box.min_bound_.x() + (box.max_bound_.x() - box.min_bound_.x()) * p;
			double y = box.min_bound_.y() + (box.max_bound_.y() - box.min_bound_.y()) * p;
			double z = box.min_bound_.z() + (box.max_bound_.z() - box.min_bound_.z()) * p;
			
			auto sx = partitionScore(boxes_, Axis::AxisX, x, side);
			if (sx < best) {
				best = sx;
				bestAxis = Axis::AxisX;
				bestPoint = x;
				bestSide = side;
			}

			auto sy = partitionScore(boxes_, Axis::AxisY, y, side);
			if (sy < best) {
				best = sy;
				bestAxis = Axis::AxisY;
				bestPoint = y;
				bestSide = side;
			}

			auto sz = partitionScore(boxes_, Axis::AxisZ, z, side);
			if (sz < best) {
				best = sz;
				bestAxis = Axis::AxisZ;
				bestPoint = z;
				bestSide = side;
			}
		}
	}

	if (bestAxis == Axis::AxisNone) {
		return;
	}
	
	auto result = partition(boxes_, bestAxis, bestPoint, bestSide);
	this->_left = NewNode(result.first, depth_ - 1);
	this->_right = NewNode(result.second, depth_ - 1);
}

Tree::Tree(int size_) {

	this->_boxes.resize(size_);
}

Tree Tree::Transform(Eigen::Matrix4d m_) {

	Tree result(this->_boxes.size());
	
	for (auto i = 0; i < this->_boxes.size(); ++i) {
		result._boxes[i] = this->_boxes[i];
		result._boxes[i].Transform(m_);
	}

	return result;
}

bool Tree::Intersects(Tree b_, Eigen::Vector3d t1, Eigen::Vector3d t2) {

	return intersects(b_, t1, t2, 0, 0);
}

bool Tree::intersects(Tree b_, Eigen::Vector3d t1, Eigen::Vector3d t2, int i, int j) {

	auto a = this->_boxes;
	auto b = b_._boxes;

	if (!boxesIntersect(a[i], b[j], t1, t2)) {
		return false;
	}

	auto i1 = i * 2 + 1;
	auto i2 = i * 2 + 2;
	auto j1 = j * 2 + 1;
	auto j2 = j * 2 + 2;
	if (i1 >= a.size() && j1 >= b.size()) {
		return true;
	}
	else if (i1 >= a.size()) {
		return intersects(b_, t1, t2, i, j1) || intersects(b_, t1, t2, i, j2);
	}
	else if (j1 >= b.size()) {
		return intersects(b_, t1, t2, i1, j) || intersects(b_, t1, t2, i2, j);
	}
	else {
		return
			intersects(b_, t1, t2, i1, j1) ||
			intersects(b_, t1, t2, i1, j2) ||
			intersects(b_, t1, t2, i2, j1) ||
			intersects(b_, t1, t2, i2, j2);
	}
}

bool boxesIntersect(Box b1, Box b2, Eigen::Vector3d t1, Eigen::Vector3d t2) {

	if (b1.IsEmpty() || b2.IsEmpty()) {
		return false;
	}
	
	return !(
		b1.min_bound_.x() + t1.x() > b2.max_bound_.x() + t2.x() ||
		b1.max_bound_.x() + t1.x() < b2.min_bound_.x() + t2.x() ||
		b1.min_bound_.y() + t1.y() > b2.max_bound_.y() + t2.y() ||
		b1.max_bound_.y() + t1.y() < b2.min_bound_.y() + t2.y() ||
		b1.min_bound_.z() + t1.z() > b2.max_bound_.z() + t2.z() ||
		b1.max_bound_.z() + t1.z() < b2.min_bound_.z() + t2.z());
}

std::pair<bool, bool> partitionBox(Box box_, Axis axis_, double point_) {

	bool left = false;
	bool right = false;
	switch (axis_)
	{
	case Axis::AxisX:
		left = box_.min_bound_.x() <= point_;
		right = box_.max_bound_.x() >= point_;
		break;
	case Axis::AxisY:
		left = box_.min_bound_.y() <= point_;
		right = box_.max_bound_.y() >= point_;
		break;
	case Axis::AxisZ:
		left = box_.min_bound_.z() <= point_;
		right = box_.max_bound_.z() >= point_;
		break;
	}
	return std::make_pair(left, right);
}

double partitionScore(Boxes boxes_, Axis axis_, double point_, bool side_) {

	Box major;
	for (auto i = 0; i < boxes_.size(); ++i) {
		auto collide = partitionBox(boxes_[i], axis_, point_);
		bool left = collide.first;
		bool right = collide.second;
		if ((left && right) || (left && side_) || (right && !side_)) {
			Extend(major, boxes_[i]);
		}
	}
	Box minor;
	for (auto i = 0; i < boxes_.size(); ++i) {
		if (!ContainsBox(major, boxes_[i])) {
			Extend(minor, boxes_[i]);
		}
	}
	
	return major.Volume() + minor.Volume() - Intersection(major, minor).Volume();
}

std::pair<Boxes, Boxes> partition(Boxes boxes_, Axis axis_, double point_, bool side_) {

	Box major;
	for (auto i = 0; i < boxes_.size(); ++i) {
		auto collide = partitionBox(boxes_[i], axis_, point_);
		bool left = collide.first;
		bool right = collide.second;
		if ((left && right) || (left && side_) || (right && !side_)) {
			Extend(major, boxes_[i]);
		}
	}

	Boxes left;
	Boxes right;
	for (auto i = 0; i < boxes_.size(); ++i) {
		if (ContainsBox(major, boxes_[i])) {
			left.push_back(boxes_[i]);
		}
		else {
			right.push_back(boxes_[i]);
		}
	}

	if (!side_) {
		std::swap(left, right);
	}

	return std::make_pair(left, right);
}