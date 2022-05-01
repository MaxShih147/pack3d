#include "Model.h"
#include "Anneal.h"

#include <random>

std::vector<Eigen::Matrix4d> g_rotations;

double pi = 3.14159265359;

Eigen::Matrix4d GetRotationFromAxisAndAngle(Eigen::Vector3d axis_, double angle_) {

	axis_.normalize();
	auto s = sin(angle_);
	auto c = cos(angle_);
	auto m = 1.0 - c;

	Eigen::Matrix4d T;
	T <<
		m * axis_.x() * axis_.x() + c,
		m * axis_.x() * axis_.y() + axis_.z() * s,
		m * axis_.z() * axis_.x() - axis_.y() * s,
		0,
		m * axis_.x() * axis_.y() - axis_.z() * s,
		m * axis_.y() * axis_.y() + c,
		m * axis_.y() * axis_.z() + axis_.x() * s,
		0,
		m * axis_.z() * axis_.x() + axis_.y() * s,
		m * axis_.y() * axis_.z() - axis_.x() * s,
		m * axis_.z() * axis_.z() + c,
		0,
		0, 0, 0, 1;
	
	return T;
}

Eigen::Vector3d Perpendicular(Eigen::Vector3d a) {

	if (a.x() == 0 && a.y() == 0) {
		if (a.z() == 0) {
			return Eigen::Vector3d(0.0, 0.0, 0.0);
		}
		return Eigen::Vector3d(0.0, 1.0, 0.0);
	}

	Eigen::Vector3d ap(-a.y(), a.x(), 0);
	ap.normalize();
	return ap;
}

Eigen::Matrix4d GetRotationWithinTwoDirections(Eigen::Vector3d source_, Eigen::Vector3d target_) {
	
	auto dot = source_.dot(target_);
	if (dot == 1) {

		Eigen::Matrix4d result;
		result.setIdentity();
		return result;
	}
	else if (dot == -1) {

		return GetRotationFromAxisAndAngle(Perpendicular(source_), pi);
	}
	else {

		auto v = target_.cross(source_);
		v.normalize();
		return GetRotationFromAxisAndAngle(v, acos(dot));
	}
}

void init() {

	for (auto i = 0; i < 4; ++i) {
		for (auto s = -1; s <= 1; s += 2) {
			for (auto a = 0; a < 3; ++a) {

				const auto &up = axisVector[int(Axis::AxisZ)];

				// -> [matrix.go] func Rotate(v Vector, a float64) Matrix
				auto m = GetRotationFromAxisAndAngle(up, i * 0.5 * pi);

				m = GetRotationWithinTwoDirections(up, axisVector[a] * s) * m;
				g_rotations.push_back(m);
			}
		}
	}
}

Tree NewTreeForMesh(MeshPtr mesh_, int depth_) {

	// -> mesh = mesh.Copy()
	MeshPtr mesh = std::make_shared<Mesh>();
	*mesh = *mesh_;
	
	Center(mesh);

	Boxes boxes(mesh->triangles_.size());
	for (auto i = 0; i < mesh->triangles_.size(); ++i) {	
		boxes[i] = open3d::geometry::AxisAlignedBoundingBox();
	}
	
	auto root = NewNode(boxes, depth_);
	auto size = 1 << (unsigned int)(depth_ + 1) - 1;
	Tree tree(size);
	root->Flatten(tree, 0);
	return tree;
}

void MoveTo(MeshPtr mesh_, Eigen::Vector3d position_, Eigen::Vector3d anchor_) {

	auto box = mesh_->GetAxisAlignedBoundingBox();
	auto sub = Anchor(box, anchor_);
	mesh_->Translate(position_ - sub);
}

void Center(MeshPtr mesh_) {

	MoveTo(
		mesh_,
		Eigen::Vector3d(0.0, 0.0, 0.0),
		Eigen::Vector3d(0.5, 0.5, 0.5)
	);
}

void Model::Add(MeshPtr mesh_, int detail_, int count_) {
	
	auto tree = NewTreeForMesh(mesh_, detail_);
	auto trees = Trees(g_rotations.size());

	for (auto i = 0; i < g_rotations.size(); ++i) {
		trees[i] = tree.Transform(g_rotations[i]);
	}
	
	for (auto i = 0; i < count_; ++i) {
		add(mesh_, trees);
	}
}

void Model::add(MeshPtr mesh_, Trees trees_) {

	int index = _items.size();
	Item item(mesh_, trees_, 0, Eigen::Vector3d());
	this->_items.push_back(item);

	double d = 1.0;
	
	while (!ValidChange(index)) {

		//rand.Intn(n) -> return a random number within [0, n-1].
		auto n = g_rotations.size();
		std::uniform_int_distribution<> distrib(0, n - 1);
		this->_items.back()._rotation = distrib(gen);
		//this->_items.back()._translation = fauxgl.RandomUnitVector().MulScalar(d);
		d *= 1.2;
	}

	auto tree = trees_[0];
	//m.MinVolume = math.Max(m.MinVolume, tree[0].Volume())
	//m.MaxVolume += tree[0].Volume()
}

void Model::Reset() {

	auto items = this->_items;
	
	// m.Items = nil
	this->_items.clear();
	this->_minVolume = 0;
	this->_maxVolume = 0;

	for (auto i = 0; i < items.size(); ++i) {
		add(items[i]._mesh, items[i]._trees);
	}
}

//std::vector<MeshPtr> Model::Meshes() {
//
//	std::vector<MeshPtr> result(this->_items);
//result: = make([] * fauxgl.Mesh, len(m.Items))
//	for i, item : = range m.Items {
//		mesh: = item.Mesh.Copy()
//		mesh.Transform(item.Matrix())
//		result[i] = mesh
//	}
//	return result
//}

bool Model::ValidChange(int i_) {

	auto item1 = this->_items[i_];
	auto tree1 = item1._trees[item1._rotation];

	for (auto j = 0; j < this->_items.size(); ++j) {

		if (j == i_) {
			continue;
		}

		auto item2 = this->_items[j];
		auto tree2 = item2._trees[item2._rotation];
		
		if (tree1.Intersects(tree2, item1._translation, item2._translation)) {
			return false;
		}
	}

	return true;
}

Box Model::BoundingBox() {

	Box box;

	for (auto i = 0; i < this->_items.size(); ++i) {

		auto tree = this->_items[i]._trees[this->_items[i]._rotation];
		Extend(box, tree._boxes[0].Translate(this->_items[i]._translation));
	}

	return box;
}

double Model::Volume() {

	return BoundingBox().Volume();
}

double Model::Energy() {

	return Volume() / this->_maxVolume;
}

Undo Model::DoMove() {

	//rand.Intn(n) -> return a random number within [0, n-1].
	auto n = this->_items.size();
	std::uniform_int_distribution<> distrib(0, n - 1);
	auto i = distrib(gen);

	auto item = this->_items[i];
	Undo undo(i, item._rotation, item._translation);

	while (1) {

		std::uniform_int_distribution<> dice(0, 3);
		
		if (dice(gen) == 0) {
		
			// rotate
			std::uniform_int_distribution<> rand_rotation(0, g_rotations.size() - 1);
			item._rotation = rand_rotation(gen);
		}
		else {

			// translate
			std::uniform_int_distribution<> rand_axis(0, 3);
			auto offset = axisVector[rand_axis(gen)];

			std::normal_distribution<> rand_scale{0.0, 1.0};
			offset *= rand_scale(gen) * this->_deviation;
			item._translation = item._translation + offset;
		}
		if (ValidChange(i)) {
			break;
		}
	
		item._rotation = undo._rotation;
		item._translation = undo._translation;
	}

	return undo;
}

void Model::UndoMove(Undo undo_) {

	this->_items[undo_._index]._rotation = undo_._rotation;
	this->_items[undo_._index]._translation = undo_._translation;
}

ModelPtr Pack(ModelPtr model_, int iterations_/*, callback AnnealCallback*/) {

	double e = 0.5;
	return Anneal(model_, 1e0 * e, 1e-4 * e, iterations_/*, callback*/);
}