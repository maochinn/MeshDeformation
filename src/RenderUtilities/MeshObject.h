#pragma once
#include <string>
#include <queue>

#include <glad/glad.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include "BufferObject.h"
#include "Texture.h"

//class serverController;

typedef OpenMesh::TriMesh_ArrayKernelT<>  TriMesh;

typedef CGAL::Exact_predicates_inexact_constructions_kernel           CGAL_K;
typedef CGAL::Delaunay_mesh_vertex_base_2<CGAL_K>                     CGAL_Vb;
typedef CGAL::Delaunay_mesh_face_base_2<CGAL_K>                       CGAL_Fb;
typedef CGAL::Triangulation_data_structure_2<CGAL_Vb, CGAL_Fb>        CGAL_Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<CGAL_K, CGAL_Tds>  CGAL_CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CGAL_CDT>                 CGAL_Criteria;

typedef CGAL_CDT::Vertex_handle CGAL_Vertex_handle;
typedef CGAL_CDT::Point CGAL_Point;

#define CGAL_MESH_2_OPTIMIZER_VERBOSE
//#define CGAL_MESH_2_OPTIMIZERS_DEBUG
//#define CGAL_MESH_2_SIZING_FIELD_USE_BARYCENTRIC_COORDINATES

#define SELECT_TYPE_CONTROL_POINT 0
#define SELECT_TYPE_WEIGHT 1

class MyMesh : public TriMesh
{
public:
	MyMesh();
	~MyMesh();

	void Reset();

	void Initialization();
	void Registration();
	void preComputeG();
	void preComputeL1();
	void preComputeL2();

	struct ControlPoint {
		MyMesh::FaceHandle fh;
		double w[3];
		MyMesh::Point o;
		MyMesh::Point c;
	};

	ControlPoint createControlPoint(unsigned int, MyMesh::Point);

	void InitCompilation();
	void AddControlPoint(ControlPoint);
	void AddControlPoints(std::vector<ControlPoint>&);
	void RemoveControlPoint(unsigned int);
	void Compilation();

	void SetEdgeWeights(std::set<unsigned int>&);

	void Compute(unsigned int);
	void Step1();
	void Step2();

	const double W = 1000;

	std::vector<ControlPoint> controlPoints;
	std::vector<MyMesh::Point> deformed_vertices;

private:
	bool holdCompilation = false;

	Eigen::SparseMatrix<double> L1, L2, LL1, LL2;
	Eigen::SparseMatrix<double> C1, C2, CC1, CC2;

	std::vector<Eigen::Triplet<double>> C1_triplets;
	std::vector<Eigen::Triplet<double>> C2_triplets;

	Eigen::SparseMatrix<double, Eigen::RowMajor> A1, A2;
	Eigen::SparseMatrix<double> AA1, AA2;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver1, solver2;

	Eigen::MatrixXd b1, b2x, b2y;
	Eigen::VectorXd V1, V2x, V2y;

	OpenMesh::EPropHandleT<Eigen::MatrixXd> prop_G;
	OpenMesh::EPropHandleT<double> prop_W;
	OpenMesh::EPropHandleT<double> prop_W_C;

};

class GLMesh
{
public:
	GLMesh(int frames);
	~GLMesh();

	const float SIZE = 250.0f;

	bool Init(std::string fileName);
	void resetMesh();

	// i / o
	bool exportMesh(std::string fileName);
	// control point io
	bool importPreset(std::string fname);
	bool exportPreset(std::string fname);

	// rendering
	void renderMesh();
	void renderSelectedMesh();
	void renderControlPoints();
	void renderKeyPoints();

	// select triangle and add constraint
	
	//void selectTri(unsigned int, bool);
	void addConstrainedTriangle(unsigned int);
	void removeConstrainedTriangle(unsigned int);
	void applyTriangleWeights();
	
	// control points control
	int select_id = -1;
	void addControlPoint(unsigned int, MyMesh::Point);
	void selectControlPoint(MyMesh::Point);
	void dragControlPoint(MyMesh::Point);
	void removeSelectedControlPoint();

	// keypoints control
	int select_k_id = -1;
	MyMesh::Point current_key;
	void addKeyPoint(MyMesh::Point);
	void selectKeyPoint(MyMesh::Point);
	void dragKeyPoint(MyMesh::Point);
	void removeSelectedKeyPoint();
	void Interpolate();
	  
	// frame operator
	void setFrameControlPoint(int);
	void loadFrameControlPoint(int);
	

	// utilities
	bool validID(int);

private:
	MyMesh mesh;
	VAO vao;
	Texture2D* texture = nullptr;

	//keyData store control points per keypoint
	std::vector<std::vector<MyMesh::Point>> keyData;
	std::vector<MyMesh::Point> keyPoints;

	//frameData store control points per frame
	std::vector<std::vector<MyMesh::Point>> frameData;

	std::set<unsigned int> constrainedTriIDs;

	static void CgalCdtToOpenMesh(MyMesh& mesh, const CGAL_CDT& cdt, float scale = 250);

	bool Load2DImage(std::string fileName);
	bool Load2DModel(std::string fileName);
	bool LoadMesh(std::string fileName);
	void LoadToShader();
	void LoadToShader(
		std::vector<MyMesh::Point>& vertices,
		std::vector<MyMesh::Normal>& normals,
		std::vector<unsigned int>& indices);
	void UpdateShader();
	void LoadTexCoordToShader();
};

