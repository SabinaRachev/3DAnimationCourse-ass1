#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include <igl\vertex_triangle_adjacency.h>
#include "Eigen/dense"
#include <igl/circulation.h>
#include <functional>



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
		//	data().SetCenterOfRotation(Eigen::Vector3d(0, 0, 0));

			
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}




SandBox::~SandBox()
{

}


void SandBox::initData()
{
	int size = data_list.size();
	//reasize all data structures
	resizeDataStructers(size);
	//initialize data structures
	for (int i = 0; i < size; i++) {
		initData(i);
	}

}
void SandBox::resizeDataStructers(int size) {
	//reasize all data structures
	E.resize(size);
	EMAP.resize(size);
	EF.resize(size);
	Q.resize(size);
	EI.resize(size);
	C.resize(size);
	Qit.resize(size);
	Qmatrix.resize(size);
	num_collapsed.resize(size);
}
void SandBox::initData(int i) {	
	Eigen::MatrixXd V = data_list[i].V;  //vertice matrix
	Eigen::MatrixXi F = data_list[i].F; //faces matrix
	igl::edge_flaps(F, E[i], EMAP[i], EF[i], EI[i]);//init data_structures
	C[i].resize(E[i].rows(), V.cols());
	Qit[i].resize(E[i].rows()); //number of edges 
	caculateQMatrix(V, F, i);
	Q[i].clear();
	num_collapsed[i] = 0;
	//caculate egdes cost
	for (int j = 0; j < E[i].rows(); j++)
		caculateCostAndPlacment(i, j, V);

}


void SandBox::caculateQMatrix(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int index){
	std::vector<std::vector<int> > VF;// vertex to faces
	std::vector<std::vector<int> > VFi;//not used
	int n = V.rows();
	Qmatrix[index].resize(n);
	igl::vertex_triangle_adjacency(n, F, VF, VFi);
	Eigen::MatrixXd F_normals = data_list[index].F_normals;
	

	for (int i = 0; i < n; i++) {
		//initialize 
		Qmatrix[index][i] = Eigen::Matrix4d::Zero();

		//caculate vertex  Q matrix 
		for (int j = 0; j < VF[i].size(); j++) {
			Eigen::Vector3d normal = F_normals.row(VF[i][j]).normalized();//get face normal
			// the equation is ax+by+cz+d=0
			Eigen::Matrix4d curr;
			double a = normal[0];
			double b = normal[1];
			double c = normal[2];
			double d = V.row(i) * normal;
			d *= -1;
			curr.row(0) = Eigen::Vector4d(a*a, a*b, a*c, a*d);
			curr.row(1) = Eigen::Vector4d(a*b, b*b, b*c, b*d);
			curr.row(2) = Eigen::Vector4d(a*c, b*c, c*c, c*d);
			curr.row(3) = Eigen::Vector4d(a*d,b*d, c*d, d*d);
			Qmatrix[index][i] += curr;
		}

	}
}
// compute cost and potential placement and place in queue
void SandBox::caculateCostAndPlacment(int index, int edge, Eigen::MatrixXd& V)
{
	//vertexes of the edge
	int v1 = E[index](edge, 0);
	int v2 = E[index](edge, 1);

	Eigen::Matrix4d Qedge= Qmatrix[index][v1] + Qmatrix[index][v2];

	Eigen::Matrix4d Qposition = Qedge; //we will use this to find v` position
	Qposition.row(3) = Eigen::Vector4d(0, 0, 0, 1);
	Eigen::Vector4d vposition;
	double cost;
	bool isInversable;
	Qposition.computeInverseWithCheck(Qposition, isInversable);
	if (isInversable) {
		vposition = Qposition * (Eigen::Vector4d(0, 0, 0, 1));
		cost = vposition.transpose() * Qedge * vposition;
	}
	else {
		//find min error from v1 v2 v1+v2/2
		Eigen::Vector4d v1p;
		v1p<< V.row(v1), 1;;
		double cost1 = v1p.transpose() * Qedge * v1p;

		Eigen::Vector4d v2p;
		v1p << V.row(v2), 1;;
		double cost2 = v2p.transpose() * Qedge * v2p;

		Eigen::Vector4d v12p;
		v1p << ((V.row(v1)+ V.row(v2))/2), 1;;
		double cost3 = v12p.transpose() * Qedge * v12p;
		if (cost1 < cost2 && cost1 < cost3) {
			vposition = v1p;
			cost = cost1;
		}
		else if (cost2 < cost1 && cost2 < cost3) {
			vposition = v2p;
			cost = cost2;
		}
		else {
			vposition = v12p;
			cost = cost3;

		}
	}
	Eigen::Vector3d pos;
	pos[0] = vposition[0];
	pos[1] = vposition[1];
	pos[2] = vposition[2];
	C[index].row(edge) = pos;
	Qit[index][edge] = Q[index].insert(std::pair<double, int>(cost, edge)).first;

}

void SandBox::simplification(){
	int id = data().id;
	Eigen::MatrixXd& V = data().V;  //vertice matrix
	Eigen::MatrixXi& F = data().F; //faces matrix
	bool something_collapsed = false;
	// collapse edge
	const int max_iter = std::ceil(0.05 * Q[id].size());//collapse 5%
	for (int j = 0; j < max_iter; j++)
	{
		if (!collapse_edge(V,F,id)){
			break;
		}
		something_collapsed = true;
		num_collapsed[id]++;
	}

	if (something_collapsed)
	{   
		//data().clear();
		data().set_mesh(V, F);
		data().set_face_based(true);
		data().dirty = 157;

	}
}
bool SandBox::collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int id){
	PriorityQueue&  curr_Q=Q[id];
	std::vector<PriorityQueue::iterator >& curr_Qit = Qit[id];
	int e1, e2, f1, f2; //be used in the igl collapse_edge function
	if (curr_Q.empty())
	{
		// no edges to collapse
		return false;
	}
	std::pair<double, int> pair = *(curr_Q.begin());
	if (pair.first == std::numeric_limits<double>::infinity())
	{
		// min cost edge is infinite cost
		return false;
	}
	curr_Q.erase(curr_Q.begin()); //delete from the queue
	int e = pair.second; //the lowest cost edge in the queue
	//the 2 vertix of the edge
	int v1 = E[id].row(e)[0];
	int v2 = E[id].row(e)[1];

	curr_Qit[e] = curr_Q.end();

	//get the  list of faces around the end point the edge
	std::vector<int> N = igl::circulation(e, true, EMAP[id], EF[id], EI[id]);
	std::vector<int> Nd = igl::circulation(e, false, EMAP[id], EF[id], EI[id]);
	N.insert(N.begin(), Nd.begin(), Nd.end());

	//collapse the edage
	bool is_collapsed = igl::collapse_edge(e, C[id].row(e), V, F, E[id], EMAP[id], EF[id], EI[id], e1, e2, f1, f2);
	if(is_collapsed){


		// Erase the two, other collapsed edges
		curr_Q.erase(curr_Qit[e1]);
		curr_Qit[e1] = curr_Q.end();
		curr_Q.erase(curr_Qit[e2]);
		curr_Qit[e2] = curr_Q.end();

		//update the Q matrix for the 2 veterixes we collapsed 
		Qmatrix[id][v1] = Qmatrix[id][v1] + Qmatrix[id][v2];
		Qmatrix[id][v2] = Qmatrix[id][v1] + Qmatrix[id][v2];

		Eigen::VectorXd newPosition;
		// update local neighbors
		// loop over original face neighbors
		for (auto n : N)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// get edge id
					 const  int ei = EMAP[id](v * F.rows() + n);
					// erase old entry
					curr_Q.erase(curr_Qit[ei]);
					// compute cost and potential placement and place in queue
					caculateCostAndPlacment(id, ei, V);
					newPosition = C[id].row(ei);
				}
			}
		}
		std::cout << "edge " << e << ",cost " << pair.first << ",new position (" << newPosition[0] << ","
			<< newPosition[1] << "," << newPosition[2] << ")" << std::endl;
	}
	else
	{
		// reinsert with infinite weight (the provided cost function must **not**
		// have given this un-collapsable edge inf cost already)
		pair.first = std::numeric_limits<double>::infinity();
		curr_Qit[e] = curr_Q.insert(pair).first;
	}
	return is_collapsed;

}




void SandBox::Animate()
{
	if (isActive)
	{
		
		
		
	}
}


