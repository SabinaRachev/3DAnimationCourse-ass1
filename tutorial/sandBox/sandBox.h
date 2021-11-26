#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <igl\vertex_triangle_adjacency.h>


class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void initData();
	void Init(const std::string& config);
    void initData(int i);
	void resizeDataStructers(int size);
	void caculateQMatrix(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int index);
	void caculateCostAndPlacment(int index, int edge, Eigen::MatrixXd& V);
	void simplification();
	bool collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int id);
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue
	std::vector<Eigen::VectorXi> EMAP; //edages to faces
	std::vector<Eigen::MatrixXi> E,EF,EI;
	typedef std::set<std::pair<double, int> > PriorityQueue;
	std::vector < PriorityQueue> Q;		//priority queue - cost for every edge
	std::vector < std::vector<PriorityQueue::iterator > > Qit;
	std::vector < Eigen::MatrixXd> C; ///positions 
	std::vector < std::vector <Eigen::Matrix4d> > Qmatrix; //list of Q matrix for each vertical
	std::vector<int> num_collapsed;
	
	void Animate();
};

