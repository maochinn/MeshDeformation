#include <map>
#include <set>
#include <algorithm>
#include "MeshObject.h"
struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;


#pragma region MyMesh

MyMesh::MyMesh()
{
	request_vertex_normals();
	request_vertex_status();
	request_face_status();
	request_edge_status();
	//request_halfedge_status();
}

MyMesh::~MyMesh()
{

}
int MyMesh::FindVertex(MyMesh::Point pointToFind)
{
	int idx = -1;
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		MyMesh::Point p = point(*v_it);
		if (pointToFind == p)
		{
			idx = v_it->idx();
			break;
		}
	}

	return idx;
}
void MyMesh::ClearMesh()
{
	if (!faces_empty())
	{
		for (MyMesh::FaceIter f_it = faces_begin(); f_it != faces_end(); ++f_it)
		{
			delete_face(*f_it, true);
		}

		garbage_collection();
	}
}
#pragma endregion

#pragma region GLMesh

GLMesh::GLMesh()
{
	this->vao.element_amount = 0;

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

	this->skeleton.element_amount = 0;

	glGenVertexArrays(1, &this->skeleton.vao);
	glBindVertexArray(this->skeleton.vao);

	glGenBuffers(3, this->skeleton.vbo);

	glBindBuffer(GL_ARRAY_BUFFER, this->skeleton.vbo[0]);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, this->skeleton.vbo[1]);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glGenBuffers(1, &this->skeleton.ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->skeleton.ebo);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

GLMesh::~GLMesh()
{

}

bool GLMesh::Init(std::string fileName)
{
	if (LoadModel(fileName))
	{
		//std::cout << "vertex number: " << mesh.n_vertices() << std::endl;
		//std::cout << "edge number: " << mesh.n_edges() << std::endl;
		//std::cout << "half edge number: " << mesh.n_halfedges() << std::endl;
		//std::cout << "face number: " << mesh.n_faces() << std::endl;
		std::vector<MyMesh::Point> vertices;
		std::vector<MyMesh::Normal> normals;
		std::vector<unsigned int> indices;

		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			vertices.push_back(mesh.point(*v_it));
			normals.push_back(mesh.normal(*v_it));
		}
		for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
			for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
				indices.push_back(fv_it->idx());

		LoadToShader(vertices, normals, indices);
	
		return true;
	}
	return false;
}

void GLMesh::renderMesh()
{
	if (this->vao.element_amount)
	{
		glBindVertexArray(this->vao.vao);
		glDrawElements(GL_TRIANGLES, this->vao.element_amount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}
}

bool GLMesh::LoadModel(std::string fileName)
{
	OpenMesh::IO::Options ropt;
	if (OpenMesh::IO::read_mesh(mesh, fileName, ropt))
	{
		if (!ropt.check(OpenMesh::IO::Options::VertexNormal) && mesh.has_vertex_normals())
		{
			mesh.request_face_normals();
			mesh.update_normals();
			//mesh.release_face_normals();
		}
		return true;
	}

	return false;
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
	this->vao.element_amount = indices.size();

	glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * normals.size(), &normals[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), &indices[0], GL_STATIC_DRAW);

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
#pragma endregion
