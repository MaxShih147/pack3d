#pragma once
#include <open3d/Open3D.h>

// pack3d
#include "BVH.h"

class Model;
typedef std::shared_ptr<Model> ModelPtr;

class Item {
public:
	Item(MeshPtr mesh_, Trees trees_, int rot_, Eigen::Vector3d trans_) 
		: _mesh(mesh_), _trees(trees_), _rotation(rot_), _translation(trans_) {
	}
public:
	MeshPtr _mesh;
	Trees _trees;
	int _rotation;
	Eigen::Vector3d _translation;
};

class Undo {
public:
	Undo(int index_, int rotation_, Eigen::Vector3d translation_)
		: _index(index_), _rotation(rotation_), _translation(translation_) {
	}

public:
	int _index;
	int _rotation;
	Eigen::Vector3d _translation;
};

class Model {
public:
	// -> NewModel()
	Model() : _minVolume(0.0), _maxVolume(0.0), _deviation(1.0) {
		_items.clear();
	}
public:
	void Add(MeshPtr mesh_, int detail_, int count_);
	void Reset();

	// todo maybe we don't need these...
	//func(m* Model) Meshes()[] * fauxgl.Mesh
	//func(m* Model) Mesh()* fauxgl.Mesh
	//func(m* Model) TreeMeshes()[] * fauxgl.Mesh
	//func(m* Model) TreeMesh()* fauxgl.Mesh

	bool ValidChange(int itemIndex);
	Box BoundingBox();
	double Volume();
	double Energy();

	Undo DoMove();
	void UndoMove(Undo undo_);

	// todo maybe we don't need these...
	//func(m* Model) Copy()

private:
	void add(MeshPtr mesh_, Trees trees_);

public:
	std::vector<Item> _items;
	double _minVolume;
	double _maxVolume;
	double _deviation;
};

Tree NewTreeForMesh(MeshPtr mesh_, int depth_);

void MoveTo(MeshPtr mesh_, Eigen::Vector3d position_, Eigen::Vector3d anchor_);
void Center(MeshPtr mesh_);

ModelPtr Pack(ModelPtr model_, int iterations_/*, callback AnnealCallback*/);