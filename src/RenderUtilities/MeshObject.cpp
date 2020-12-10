#include <map>
#include <set>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include "../CDT/include/CDT.h"
//#include "../CDT/extras/VerifyTopology.h"

#define CGAL_MESH_2_OPTIMIZER_VERBOSE
//#define CGAL_MESH_2_OPTIMIZERS_DEBUG
//#define CGAL_MESH_2_SIZING_FIELD_USE_BARYCENTRIC_COORDINATES
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include "MeshObject.h"

#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel           CGAL_K;
typedef CGAL::Delaunay_mesh_vertex_base_2<CGAL_K>                     CGAL_Vb;
typedef CGAL::Delaunay_mesh_face_base_2<CGAL_K>                       CGAL_Fb;
typedef CGAL::Triangulation_data_structure_2<CGAL_Vb, CGAL_Fb>        CGAL_Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<CGAL_K, CGAL_Tds>  CGAL_CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CGAL_CDT>                 CGAL_Criteria;

typedef CGAL_CDT::Vertex_handle CGAL_Vertex_handle;
typedef CGAL_CDT::Point CGAL_Point;

struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
	assert(!(hi < lo));
	return (v < lo) ? lo : (hi < v) ? hi : v;
}

inline float cot(MyMesh::Point v, MyMesh::Point w) {
	v.normalize();
	w.normalize();
	if (v == w)
		return std::numeric_limits<float>::infinity();

	return(v.dot(w) / v.cross(w).norm());
};

#pragma region MyMesh

MyMesh::MyMesh()
{
	request_vertex_normals();
	request_vertex_status();
	request_face_status();
	request_edge_status();
	request_halfedge_status();

	this->add_property(prop_G, "prop_G"); 
	this->add_property(prop_W, "prop_W");
}

MyMesh::~MyMesh()
{

}

void MyMesh::Initialization()
{
	std::cout << "Vert : " << n_vertices() << std::endl;
	std::cout << "Face : " << n_faces() << std::endl;
	deformed_vertices.clear();
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		deformed_vertices.push_back(point(v_it));
	}
	Registration();
	InitCompilation();
}

void MyMesh::Registration()
{
	// precompute L1, L2 and G
	preComputeG();
	preComputeL1();
	preComputeL2();
}

void MyMesh::preComputeG()
{
	// pre G
	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {
		int rows = (is_boundary(e_it) ? 6 : 8);
		int row = 0;

		Eigen::MatrixXd G(rows, 2);
		double Weight = 0;

		HalfedgeHandle heh = halfedge_handle(e_it, 0);
		MyMesh::Point pFrom = point(from_vertex_handle(heh));
		G(row, 0) = pFrom[0]; G(row, 1) = pFrom[2]; row += 1;
		G(row, 0) = pFrom[2]; G(row, 1) = -pFrom[0]; row += 1;

		MyMesh::Point pTo = point(to_vertex_handle(heh));
		G(row, 0) = pTo[0]; G(row, 1) = pTo[2]; row += 1;
		G(row, 0) = pTo[2]; G(row, 1) = -pTo[0]; row += 1;

		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			MyMesh::Point p0 = point(vh0);
			G(row, 0) = p0[0]; G(row, 1) = p0[2]; row += 1;
			G(row, 0) = p0[2]; G(row, 1) = -p0[0]; row += 1;
			Weight += abs(cot(pTo - p0, pFrom - p0));
		}

		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			MyMesh::Point p1 = point(vh1);
			G(row, 0) = p1[0]; G(row, 1) = p1[2]; row += 1;
			G(row, 0) = p1[2]; G(row, 1) = -p1[0];

			Weight += abs(cot(pTo - p1, pFrom - p1));
		}

		if (!is_boundary(e_it)) {
			Weight *= 0.5;
		}

		this->property(prop_G, e_it) = (G.transpose() * G).inverse() * G.transpose();
		this->property(prop_W, e_it) = 1;// Weight;
	}
}

void MyMesh::preComputeL1()
{
	const int N_E(n_edges());
	const int N_V(n_vertices());

	L1 = Eigen::SparseMatrix<double>(N_E * 2, N_V * 2);
	std::vector<Eigen::Triplet<double>> triplet_list_L1;

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int cols = (is_boundary(e_it) ? 6 : 8);
		int row = e_it->idx() * 2;

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		Eigen::MatrixXd& G = this->property(prop_G, e_it);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		// edge vector
		MyMesh::Point pFrom = point(vh_from);
		MyMesh::Point e = point(vh_to) - pFrom;
		Eigen::Matrix2d e_mat;
		e_mat << e[0], e[2],
			e[2], -e[0];

		Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, cols);
		h(0, 0) = -1;
		h(0, 2) = 1;
		h(1, 1) = -1;
		h(1, 3) = 1;

		h = h - e_mat * G;

		double Weight = this->property(prop_W, e_it);
		h *= Weight;

		int col = vh_from.idx() * 2;
		int hcol = 0;
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));

		col = vh_to.idx() * 2;
		hcol += 2;
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));

		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			col = vh0.idx() * 2;
			hcol += 2;
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));
		}

		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			col = vh1.idx() * 2;
			hcol += 2;
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));
		}
	}

	L1.setFromTriplets(triplet_list_L1.begin(), triplet_list_L1.end());
	LL1 = L1.transpose() * L1;
}

void MyMesh::preComputeL2()
{
	const int N_E(n_edges());
	const int N_V(n_vertices());

	L2 = Eigen::SparseMatrix<double>(N_E, N_V);
	std::vector<Eigen::Triplet<double>> triplet_list_L2;

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int row = e_it->idx();

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		double Weight = this->property(prop_W, e_it);

		triplet_list_L2.push_back(Eigen::Triplet<double>(row, vh_from.idx(), -1 * Weight));
		triplet_list_L2.push_back(Eigen::Triplet<double>(row, vh_to.idx(), 1 * Weight));
	}

	L2.setFromTriplets(triplet_list_L2.begin(), triplet_list_L2.end());
	LL2 = L2.transpose() * L2;
}

void MyMesh::select(unsigned int face_ID, MyMesh::Point p)
{
	FaceHandle fh = this->face_handle(face_ID);

	FaceVertexIter fv_it = fv_iter(fh);
	MyMesh::Point& dp0 = deformed_vertices[fv_it->idx()];
	MyMesh::Point p0 = point(fv_it); ++fv_it;

	MyMesh::Point& dp1 = deformed_vertices[fv_it->idx()];
	MyMesh::Point p1 = point(fv_it); ++fv_it;

	MyMesh::Point& dp2 = deformed_vertices[fv_it->idx()];
	MyMesh::Point p2 = point(fv_it);

	MyMesh::Point vp0 = dp0 - p;
	MyMesh::Point vp1 = dp1 - p;
	MyMesh::Point vp2 = dp2 - p;

	double a0 = vp1.cross(vp2).length();
	double a1 = vp0.cross(vp2).length();
	double a2 = vp0.cross(vp1).length();

	double i_area = 1.0 / (a0 + a1 + a2);

	ControlPoint cp;
	cp.fh = fh;
	cp.w[0] = i_area * a0;
	cp.w[1] = i_area * a1;
	cp.w[2] = i_area * a2;
	cp.o = cp.w[0] * p0 + cp.w[1] * p1 + cp.w[2] * p2;
	cp.c = cp.w[0] * dp0 + cp.w[1] * dp1 + cp.w[2] * dp2;

	std::cout << cp.o << std::endl;
	//cp.c = MyMesh::Point(0,0,0);

	AddControlPoint(cp);
}

void MyMesh::InitCompilation()
{
	C1_triplets.clear();
	C2_triplets.clear();
}

void MyMesh::AddControlPoint(ControlPoint cp)
{
	FaceVertexIter fv_it = fv_iter(cp.fh);
	int p0_idx = fv_it->idx(); ++fv_it;
	int p1_idx = fv_it->idx(); ++fv_it;
	int p2_idx = fv_it->idx();

	int row = controlPoints.size();

	int c1x_idx = row * 2;
	int c1y_idx = row * 2 + 1;
	C1_triplets.push_back(Eigen::Triplet<double>(c1x_idx, p0_idx * 2, cp.w[0] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(c1y_idx, p0_idx * 2 + 1, cp.w[0] * W));

	C1_triplets.push_back(Eigen::Triplet<double>(c1x_idx, p1_idx * 2, cp.w[1] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(c1y_idx, p1_idx * 2 + 1, cp.w[1] * W));

	C1_triplets.push_back(Eigen::Triplet<double>(c1x_idx, p2_idx * 2, cp.w[2] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(c1y_idx, p2_idx * 2 + 1, cp.w[2] * W));

	int c2_idx = row;
	C2_triplets.push_back(Eigen::Triplet<double>(c2_idx, p0_idx, cp.w[0] * W));
	C2_triplets.push_back(Eigen::Triplet<double>(c2_idx, p1_idx, cp.w[1] * W));
	C2_triplets.push_back(Eigen::Triplet<double>(c2_idx, p2_idx, cp.w[2] * W));

	controlPoints.push_back(cp);

	Compilation();
}

void MyMesh::RemoveControlPoint(unsigned int idx)
{
	int n_controlPoints = controlPoints.size();
	int s_id = (n_controlPoints - 1);

	ControlPoint& cp = controlPoints[idx];

	for (int i = 0; i < 3; i++) {
		int c1_idx = idx * 6 + i * 2;
		int c1_sidx = s_id * 6 + i * 2;
		C1_triplets[c1_idx] = Eigen::Triplet<double>(C1_triplets[c1_idx].row(), C1_triplets[c1_sidx].col(), C1_triplets[c1_sidx].value());
		C1_triplets[c1_idx+1]= Eigen::Triplet<double>(C1_triplets[c1_idx+1].row(), C1_triplets[c1_sidx+1].col(), C1_triplets[c1_sidx].value());

		int c2_idx = idx * 3 + i;
		int c2_sidx = s_id * 3 + i;
		C2_triplets[c2_idx] = Eigen::Triplet<double>(C2_triplets[c2_idx].row(), C2_triplets[c2_sidx].col(), C2_triplets[c2_sidx].value());
	}

	C1_triplets.erase(C1_triplets.end() - 6, C1_triplets.end());
	C2_triplets.erase(C2_triplets.end() - 3, C2_triplets.end());

	controlPoints[idx] = controlPoints[s_id];
	controlPoints.erase(controlPoints.end() - 1);

	if (!controlPoints.empty()) {
		Compilation();
	}
}

void MyMesh::Compilation()
{
	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	C1.resize(N_C * 2, N_V * 2); // solve x,y together
	C2.resize(N_C, N_V); // solve x, y respectively

	C1.setFromTriplets(C1_triplets.begin(), C1_triplets.end());
	C2.setFromTriplets(C2_triplets.begin(), C2_triplets.end());

	CC1 = C1.transpose() * C1;
	CC2 = C2.transpose() * C2;

	A1.resize(N_E * 2 + N_C * 2, N_V * 2);
	A1.topRows(N_E * 2) = L1;
	A1.bottomRows(N_C * 2) = C1;

	AA1 = LL1 + CC1;

	b1.resize(N_E * 2 + N_C * 2, 1);
	b1 = Eigen::MatrixXd::Zero(N_E * 2 + N_C * 2, 1);

	A2.resize(N_E + N_C, N_V);
	A2.topRows(N_E) = L2;
	A2.bottomRows(N_C) = C2;

	AA2 = LL2 + CC2;

	b2x.resize(N_E + N_C, 1);
	b2x = Eigen::MatrixXd::Zero(N_E + N_C, 1);
	b2y.resize(N_E + N_C, 1);
	b2y = Eigen::MatrixXd::Zero(N_E + N_C, 1);

}

unsigned int MyMesh::FindControlPoint(MyMesh::Point, double)
{
	return 0;
}

void MyMesh::Compute(unsigned int id)
{
	int N(n_vertices());

	if (controlPoints.size() == 1) {
		offset = (controlPoints[0].c - controlPoints[0].o);
		for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
		{
			deformed_vertices[v_it->idx()] = point(v_it) + offset;
		}
		return;
	}

	float md = controlPoints[0].o.length();
	offset = (controlPoints[0].c);
	for (int i = 1; i < controlPoints.size(); i++) {
		float d = controlPoints[i].o.length();
		if (md > d) {
			md = d;
			offset = (controlPoints[i].c);
		}
	}

	/*offset = Point(0, 0, 0);
	for (int i = 1; i < controlPoints.size(); i++) {
		offset += controlPoints[i].c;
	}
	offset /= controlPoints.size();*/

	//std::cout << "offset : "<< offset << "          \r";

	Step1();
	Step2();

	for (int i = 0; i < N; i++)
	{
		deformed_vertices[i] = MyMesh::Point(V2x(i) + offset[0], 0, V2y(i) + offset[2]);
	}
}

void MyMesh::Step1()
{
	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	for (int i = 0; i < controlPoints.size(); i++) {
		b1(N_E * 2 + i * 2, 0) = (controlPoints[i].c[0] - offset[0]) * W;
		b1(N_E * 2 + i * 2 + 1, 0) = (controlPoints[i].c[2] - offset[2]) * W;
	}

	//Eigen::SparseLU< Eigen::SparseMatrix<double>> solver(AA1);
	//Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AA1);

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(AA1);
	V1 = solver.solve(A1.transpose() * b1);
}

void MyMesh::Step2()
{
	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	
	/*V2x = Eigen::VectorXd(N_V);
	V2y = Eigen::VectorXd(N_V);

	for (int i = 0; i < N_V; i++) {
		V2x(i) = V1(i * 2);
		V2y(i) = V1(i * 2 + 1);
	}
	return;*/
	

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int cols = (is_boundary(e_it) ? 6 : 8);
		int row = e_it->idx();

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		Eigen::MatrixXd& G = this->property(prop_G, e_it);

		double Weight = this->property(prop_W, e_it);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		double pFromX = V1(vh_from.idx() * 2);
		double pFromY = V1(vh_from.idx() * 2 + 1);

		double pToX = V1(vh_to.idx() * 2);
		double pToY = V1(vh_to.idx() * 2 + 1);

		double c = 0;
		double s = 0;

		c += G(0, 0) * pFromX + G(0, 1) * pFromY; c += G(0, 2) * pToX + G(0, 3) * pToY;
		s += G(1, 0) * pFromX + G(1, 1) * pFromY; s += G(1, 2) * pToX + G(1, 3) * pToY;

		int v_row = 2;
		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			double p0X = V1(vh0.idx() * 2);
			double p0Y = V1(vh0.idx() * 2 + 1);
			c += G(0, v_row * 2) * p0X + G(0, v_row * 2 + 1) * p0Y;
			s += G(1, v_row * 2) * p0X + G(1, v_row * 2 + 1) * p0Y;
			v_row += 1;
		}
		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			double p1X = V1(vh1.idx() * 2);
			double p1Y = V1(vh1.idx() * 2 + 1);
			c += G(0, v_row * 2) * p1X + G(0, v_row * 2 + 1) * p1Y;
			s += G(1, v_row * 2) * p1X + G(1, v_row * 2 + 1) * p1Y;
		}

		MyMesh::Point e = point(vh_to) - point(vh_from);
		double det = (c * c + s * s);
		if (det == 0) {
			b2x(row, 0) = e[0] * Weight;
			b2y(row, 0) = e[2] * Weight;
			std::cout << "ZERO ZERO DET!!\n";
		}
		else {
			double norm = 1.0 / sqrt(det);
			b2x(row, 0) = (e[0] * c + e[2] * s) * norm * Weight;
			b2y(row, 0) = (e[2] * c - e[0] * s) * norm * Weight;
		}
	}

	for (int i = 0; i < controlPoints.size(); i++) {
		b2x(N_E + i, 0) = (controlPoints[i].c[0] - offset[0]) * W;
		b2y(N_E + i, 0) = (controlPoints[i].c[2] - offset[2]) * W;
	}

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(AA2);

	V2x = solver.solve(A2.transpose() * b2x);
	V2y = solver.solve(A2.transpose() * b2y);
}

#pragma endregion

#pragma region GLMesh

GLMesh::GLMesh()
{
}

GLMesh::~GLMesh()
{

}

bool GLMesh::Init(std::string fileName)
{
	std::string filetype = fileName.substr(fileName.find_last_of(".") + 1);
	bool success = false;

	if (filetype == "bmp") {
		success = Load2DImage(fileName);
	}
	else if (filetype == "jpg") {
		success = Load2DImage(fileName);
	}
	else if (filetype == "png") {
		success = Load2DImage(fileName);
	}
	else if (filetype == "txt") {
		success = Load2DModel(fileName);
	}

	if (success)
	{
		glGenVertexArrays(1, &this->vao.vao);
		glBindVertexArray(this->vao.vao);

		glGenBuffers(3, this->vao.vbo);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(1);

		glGenBuffers(1, &this->vao.ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		LoadToShader();

		std::cout << "SUCCESS" << std::endl;
		return true;
	}

	std::cout << "FAILED" << std::endl;
	return false;
}

void GLMesh::renderMesh()
{
	if (this->vao.element_amount > 0)
	{
		glBindVertexArray(this->vao.vao);
		glDrawElements(GL_TRIANGLES, this->vao.element_amount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}
}
void GLMesh::renderSelectedMesh()
{
	if (selected_faces.size() > 0)
	{
		std::vector<unsigned int*> offsets(selected_faces.size());
		for (int i = 0; i < offsets.size(); ++i)
		{
			offsets[i] = (GLuint*)(selected_faces[i] * 3 * sizeof(GLuint));
		}

		std::vector<int> count(selected_faces.size(), 3);

		glBindVertexArray(this->vao.vao);
		glMultiDrawElements(GL_TRIANGLES, &count[0], GL_UNSIGNED_INT, (const GLvoid**)&offsets[0], selected_faces.size());
		glBindVertexArray(0);
	}
}
void GLMesh::renderControlPoints()
{
	//glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(10);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	size_t n_controlPoints = this->mesh.controlPoints.size();
	for (size_t i = 0; i < n_controlPoints; i++) {
		if (i == select_id) {
			glColor3d(1, 1, 0);
		}
		else {
			glColor3d(0.2, 0.2, 0.9);
		}
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[i];
		glVertex3f(cp.c[0], 0, cp.c[2]);
	}
	glEnd();
	glDisable(GL_POINT_SMOOTH);
	//glDisable(GL_PROGRAM_POINT_SIZE);
}

void GLMesh::select(unsigned int tri_ID, MyMesh::Point p)
{
	this->mesh.select(tri_ID, p);
	this->AddSelectedFace(tri_ID);
}

void GLMesh::remove_selected()
{
	if (validID(select_id)) {
		mesh.RemoveControlPoint(select_id);
		selected_faces[select_id] = selected_faces.back();
		selected_faces.pop_back();
	}
	select_id = -1;
}

void GLMesh::selectControlPoint(MyMesh::Point p)
{
	select_id = -1;
	float min_d = 20;
	size_t n_controlPoints = this->mesh.controlPoints.size();
	for (size_t i = 0; i < n_controlPoints; i++) {
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[i];

		float d = (cp.c - p).length();
		if (d < min_d) {
			min_d = d;
			select_id = i;
		}
	}
}

void GLMesh::dragControlPoint(MyMesh::Point p)
{
	if (select_id == -1)
		return;

	this->mesh.controlPoints[select_id].c = p;

	this->mesh.Compute(select_id);

	std::vector<MyMesh::Normal> normals;
	std::vector<unsigned int> indices;

	LoadToShader(mesh.deformed_vertices, normals, indices);
}

bool GLMesh::validID(unsigned int faceID)
{
	return (faceID < mesh.n_faces());
}

typedef CGAL::Delaunay_mesher_2<CGAL_CDT, CGAL_Criteria> Mesher;
bool GLMesh::Load2DImage(std::string fileName)
{
	cv::Mat img = cv::imread(fileName, 0);

	//cv::Canny(img, edges, 100, 210);
	//floodFill(edges, cv::Point2i(edges.cols / 2, edges.rows / 2), cv::Scalar(255, 255, 255));

	std::vector<std::vector<cv::Point>> contour;
	cv::findContours(img, contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	if (contour.size() == 0)
		return false;

	int contour_id = 0;
	int max = 0;
	for (int i = 0; i < contour.size(); i++) {
		if (contour[i].size() > max) {
			max = contour[i].size();
			contour_id = i;
		}
	}

	float epsilon = 0.005 * cv::arcLength(contour[contour_id], true);

	std::vector<cv::Point> approx;
	cv::approxPolyDP(contour[contour_id], approx, epsilon, true);

	// find bounding box
	int max_x = approx[0].x;
	int max_y = approx[0].y;
	int min_x = approx[0].x;
	int min_y = approx[0].y;
	for (int i = 0; i < approx.size(); i++)
	{
		max_x = std::max(approx[i].x, max_x);
		max_y = std::max(approx[i].y, max_y);
		min_x = std::min(approx[i].x, min_x);
		min_y = std::min(approx[i].y, min_y);
	}

	float norm_size = 1.0f;
	float norm_scale = norm_size / std::max(abs(max_x - min_x), abs(max_y - min_y));
	float x_offset = (max_x + min_x) * norm_scale * 0.5f;
	float y_offset = (max_y + min_y) * norm_scale * 0.5f;

	// create constrainted delaunay triangulation handler
	CGAL_CDT cdt;

	// insertion
	std::vector<CGAL_Vertex_handle> vertices;
	for (int i = 0; i < approx.size(); i++)
	{
		vertices.push_back(
			cdt.insert(CGAL_Point(approx[i].x * norm_scale - x_offset, (1 - approx[i].y * norm_scale) - y_offset))
		);
	}

	for (std::size_t i = 1; i < approx.size(); ++i)
	{
		cdt.insert_constraint(vertices[i-1], vertices[i]);
	}
	cdt.insert_constraint(vertices[0], vertices[approx.size() -1]);

	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;
	Mesher mesher(cdt);
	std::cout << "Meshing with criterias..." << std::endl;
	mesher.set_criteria(CGAL_Criteria(0.125, 0.08));
	mesher.refine_mesh();
	std::cout << " done." << std::endl;

	if (cdt.number_of_vertices() == 0)
		return false;

	float scale = 280;
	std::map<CGAL_Vertex_handle, MyMesh::VertexHandle> v_handles;
	for (auto v_it = cdt.finite_vertices_begin(); v_it != cdt.finite_vertices_end(); ++v_it)
	{
		CGAL_Vertex_handle h = v_it->handle();
		auto& p = v_it->point();
		OpenMesh::Vec3f v(p.x() * scale, 0, p.y() * scale);
		v_handles[h] = mesh.add_vertex(v);
	}

	std::vector<MyMesh::VertexHandle> face_vhandles;
	for (auto f_it = cdt.finite_faces_begin(); f_it != cdt.finite_faces_end(); ++f_it)
	{
		if (f_it->is_in_domain()) {

			CGAL_Vertex_handle h0 = f_it->vertex(0)->handle();
			CGAL_Vertex_handle h1 = f_it->vertex(1)->handle();
			CGAL_Vertex_handle h2 = f_it->vertex(2)->handle();

			face_vhandles.clear();
			face_vhandles.push_back(v_handles[h0]);
			face_vhandles.push_back(v_handles[h1]);
			face_vhandles.push_back(v_handles[h2]);

			mesh.add_face(face_vhandles);
		}
	}

	mesh.Initialization();

	return true;
}
bool GLMesh::Load2DModel(std::string fileName)
{
	std::ifstream ifs(fileName);
	if (!ifs.is_open()) {
		std::cout << "Cannot open file \"" << fileName << "\" !!" << std::endl;
		return false;
	}

	std::size_t nPts, nEdges;
	ifs >> nPts >> nEdges;

	std::vector<MyMesh::Point> m_points;

	for (std::size_t i = 0; i < nPts; ++i)
	{
		float x1, y1;
		ifs >> x1 >> y1;
		m_points.push_back(MyMesh::Point(x1, y1, 0));
	}
	// find bounding box
	float max_x = m_points[0][0];
	float max_y = m_points[0][1];
	float min_x = m_points[0][0];
	float min_y = m_points[0][1];
	for (int i = 0; i < m_points.size(); i++)
	{
		max_x = std::max(m_points[i][0], max_x);
		max_y = std::max(m_points[i][1], max_y);
		min_x = std::min(m_points[i][0], min_x);
		min_y = std::min(m_points[i][1], min_y);
	}

	float norm_size = 1.0f;
	float norm_scale = norm_size / std::max(std::max(abs(max_x - min_x), abs(max_y - min_y)), 1.0f);
	float x_offset = (max_x + min_x) * norm_scale * 0.5f;
	float y_offset = (max_y + min_y) * norm_scale * 0.5f;

	// create constrainted delaunay triangulation handler
	CGAL_CDT cdt;

	// insertion
	std::vector<CGAL_Vertex_handle> vertices;
	for (int i = 0; i < m_points.size(); i++)
	{
		vertices.push_back(
			cdt.insert(CGAL_Point(m_points[i][0] * norm_scale - x_offset, m_points[i][1] * norm_scale - y_offset))
		);
	}

	for (std::size_t i = 0; i < nEdges; ++i)
	{
		unsigned int v1, v2;
		ifs >> v1 >> v2;
		cdt.insert_constraint(vertices[v1], vertices[v2]);
	}

	std::list<CGAL_Point> list_of_seeds;
	if (!ifs.eof()) {
		std::size_t nSeeds;
		ifs >> nSeeds;
		for (std::size_t i = 0; i < nSeeds; ++i)
		{
			float x1, y1;
			ifs >> x1 >> y1;
			list_of_seeds.push_back(CGAL_Point(x1 * norm_scale - x_offset, y1 * norm_scale - y_offset));
		}
	}

	ifs.close();

	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;

	std::cout << "Meshing..." << std::endl;
	CGAL::refine_Delaunay_mesh_2(cdt, list_of_seeds.begin(), list_of_seeds.end(),
		CGAL_Criteria(0.125, 0.12));

	std::cout << " done." << std::endl;

	if (cdt.number_of_vertices() == 0)
		return false;

	float scale = 250.0f;
	std::map<CGAL_Vertex_handle, MyMesh::VertexHandle> v_handles;
	for (auto v_it = cdt.finite_vertices_begin(); v_it != cdt.finite_vertices_end(); ++v_it)
	{
		CGAL_Vertex_handle h = v_it->handle();
		auto& p = v_it->point();
		OpenMesh::Vec3f v(p.x() * scale, 0, p.y() * scale);
		v_handles[h] = mesh.add_vertex(v);
	}

	std::vector<MyMesh::VertexHandle> face_vhandles;
	for (auto f_it = cdt.finite_faces_begin(); f_it != cdt.finite_faces_end(); ++f_it)
	{
		if (f_it->is_in_domain()) {

			CGAL_Vertex_handle h0 = f_it->vertex(0)->handle();
			CGAL_Vertex_handle h1 = f_it->vertex(1)->handle();
			CGAL_Vertex_handle h2 = f_it->vertex(2)->handle();

			face_vhandles.clear();
			face_vhandles.push_back(v_handles[h0]);
			face_vhandles.push_back(v_handles[h1]);
			face_vhandles.push_back(v_handles[h2]);

			mesh.add_face(face_vhandles);
		}
	}

	mesh.Initialization();

	return true;
}

void GLMesh::LoadToShader()
{
	std::vector<MyMesh::Point> vertices;
	std::vector<MyMesh::Normal> normals;
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(mesh.point(*v_it));
		normals.push_back(mesh.normal(*v_it));
	}
	std::vector<unsigned int> indices;
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			indices.push_back(fv_it->idx());

	LoadToShader(vertices, normals, indices);
}
void GLMesh::LoadToShader(
	std::vector<MyMesh::Point>& vertices,
	std::vector<MyMesh::Normal>& normals,
	std::vector<unsigned int>& indices)
{
	if (!vertices.empty()) {
		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);
	}
	if (!normals.empty()) {
		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * normals.size(), &normals[0], GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(1);
	}
	if (!indices.empty()) {
		this->vao.element_amount = indices.size();
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), &indices[0], GL_STATIC_DRAW);
	}
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void GLMesh::LoadTexCoordToShader()
{
	if (mesh.has_vertex_texcoords2D())
	{
		std::vector<MyMesh::TexCoord2D> texCoords;
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			MyMesh::TexCoord2D texCoord = mesh.texcoord2D(*v_it);
			texCoords.push_back(texCoord);
		}

		glBindVertexArray(this->vao.vao);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[2]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::TexCoord2D) * texCoords.size(), &texCoords[0], GL_STATIC_DRAW);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}
}

void GLMesh::resetMesh()
{
	//mesh.reloadMesh(initial_vertices, initial_indices);
	LoadToShader();
}
bool GLMesh::exportMesh()
{
	// write mesh to output.obj
	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, "output.obj"))
		{
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
			return false;
		}
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
		return false;
	}
	return true;
}
bool GLMesh::AddSelectedFace(unsigned int faceID)
{
	if (std::find(selected_faces.begin(), selected_faces.end(), faceID) == selected_faces.end() &&
		faceID >= 0 && faceID < mesh.n_faces())
	{
		selected_faces.push_back(faceID);
		return true;
	}
	return false;
}

#pragma endregion
