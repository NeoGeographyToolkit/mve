// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef SFM_MESH_RENDERER_HEADER
#define SFM_MESH_RENDERER_HEADER

#include <utility>
#include <vector>
#include <string>
#include <memory>

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

#include "SfmMath.h"
#include "GlCommon.h"

#define SFM_ATTRIB_POSITION "pos"
#define SFM_ATTRIB_NORMAL "normal"
#define SFM_ATTRIB_COLOR "color"
#define SFM_ATTRIB_TEXCOORD "texuv"

namespace sfm {

// VBO wrapper: QOpenGLBuffer plus metadata for vertex attributes.
struct VertexBuffer {
  typedef std::shared_ptr<VertexBuffer> Ptr;

  QOpenGLBuffer buf;
  GLenum datatype = GL_FLOAT;
  GLint vpv = 0;       // values per vertex
  GLsizei elems = 0;   // number of elements

  static Ptr create();
  void set_data(GLfloat const* data, GLsizei elems, GLint vpv);
  void set_indices(GLuint const* data, GLsizei num_indices);
};

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
