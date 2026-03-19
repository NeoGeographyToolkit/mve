// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef SFM_MESH_RENDERER_HEADER
#define SFM_MESH_RENDERER_HEADER

#include <utility>
#include <vector>
#include <string>
#include <memory>

#include <QOpenGLShaderProgram>

#include "SfmMath.h"
#include "GlCommon.h"

#define SFM_ATTRIB_POSITION "pos"
#define SFM_ATTRIB_NORMAL "normal"
#define SFM_ATTRIB_COLOR "color"
#define SFM_ATTRIB_TEXCOORD "texuv"

namespace sfm {

// VertexBuffer

// OpenGL vertex buffer object (VBO) abstraction.
class VertexBuffer {
public:
  typedef std::shared_ptr<VertexBuffer> Ptr;

  ~VertexBuffer(void);
  static Ptr create(void);

  void set_data(GLfloat const* data, GLsizei elems, GLint vpv);
  void set_indices(GLuint const* data, GLsizei num_indices);

  GLenum get_data_type(void) const;
  GLint get_values_per_vertex(void) const;
  GLsizei get_element_amount(void) const;

  void bind(void);

private:
  VertexBuffer(void);

  GLuint vbo_id;
  GLenum vbo_target;
  GLenum datatype;
  GLint vpv;
  GLsizei elems;
};

inline GLint
VertexBuffer::get_values_per_vertex(void) const {
  return this->vpv;
}

inline GLsizei
VertexBuffer::get_element_amount(void) const {
  return this->elems;
}

inline GLenum
VertexBuffer::get_data_type(void) const {
  return this->datatype;
}

// VertexArray

// OpenGL vertex array object (VAO) abstraction.
class VertexArray {
public:
  typedef std::shared_ptr<VertexArray> Ptr;

  typedef std::pair<VertexBuffer::Ptr, std::string> BoundVBO;
  typedef std::vector<BoundVBO> VBOList;

  virtual ~VertexArray(void);

  void set_primitive(GLuint primitive);
  void set_shader(QOpenGLShaderProgram* shader);
  void set_vertex_vbo(VertexBuffer::Ptr vbo);
  void set_index_vbo(VertexBuffer::Ptr vbo);
  void add_vbo(VertexBuffer::Ptr vbo, std::string const& name);
  void reset_vertex_array(void);
  void draw(void);

protected:
  VertexArray(void);
  void assign_attrib(BoundVBO const& bound_vbo);

private:
  GLuint vao_id;
  GLuint primitive;
  QOpenGLShaderProgram* shader;

  VertexBuffer::Ptr vert_vbo;
  VertexBuffer::Ptr index_vbo;
  VBOList vbo_list;
};

inline void
VertexArray::set_primitive(GLuint primitive) {
  this->primitive = primitive;
}

inline void
VertexArray::set_vertex_vbo(VertexBuffer::Ptr vbo) {
  this->vert_vbo = vbo;
}

inline void
VertexArray::set_index_vbo(VertexBuffer::Ptr vbo) {
  this->index_vbo = vbo;
}

inline void
VertexArray::add_vbo(VertexBuffer::Ptr vbo, std::string const& name) {
  this->vbo_list.push_back(std::make_pair(vbo, name));
}

inline void
VertexArray::set_shader(QOpenGLShaderProgram* shader) {
  this->shader = shader;
}

// MeshRenderer

// Takes a TriangleMesh and creates VBOs for rendering.
class MeshRenderer: public VertexArray {
public:
  typedef std::shared_ptr<MeshRenderer> Ptr;

  static Ptr create(sfm::TriangleMesh::ConstPtr mesh);
  void set_mesh(sfm::TriangleMesh::ConstPtr mesh);

private:
  MeshRenderer(sfm::TriangleMesh::ConstPtr mesh);
};

}

#endif // SFM_MESH_RENDERER_HEADER
