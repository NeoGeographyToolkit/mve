// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef GL_CONTEXT_HEADER
#define GL_CONTEXT_HEADER

#include "SfmMath.h"
#include "GlCommon.h"

GL_NAMESPACE_BEGIN

// Trackball camera control that consumes mouse events.
class CamTrackball {
public:
  CamTrackball(void);

  void set_camera(Camera* camera);
  bool consume_event(MouseEvent const& event);

  math::Vec3f get_campos(void) const;
  math::Vec3f get_viewdir(void) const;
  math::Vec3f const& get_upvec(void) const;

private:
  void handle_tb_rotation(int x, int y);
  math::Vec3f get_ball_normal(int x, int y);

  Camera* cam;

  float tb_radius;
  math::Vec3f tb_center;
  math::Vec3f tb_tocam;
  math::Vec3f tb_upvec;

  int rot_mouse_x;
  int rot_mouse_y;
  math::Vec3f rot_tb_tocam;
  math::Vec3f rot_tb_upvec;

  float zoom_tb_radius;
  int zoom_mouse_y;
};

inline void CamTrackball::set_camera(Camera* camera) {
  this->cam = camera;
}

inline math::Vec3f CamTrackball::get_campos(void) const {
  return this->tb_center + this->tb_tocam * this->tb_radius;
}

inline math::Vec3f CamTrackball::get_viewdir(void) const {
  return -this->tb_tocam;
}

inline math::Vec3f const& CamTrackball::get_upvec(void) const {
  return this->tb_upvec;
}

// Rendering context with trackball camera control.
// Subclass and override init_impl, resize_impl, paint_impl.
class GlContext {
public:
  virtual ~GlContext(void) {}

  void init(void);
  void resize(int new_width, int new_height);
  void paint(void);
  bool mouse_event(MouseEvent const& event);

protected:
  virtual void init_impl(void) = 0;
  virtual void resize_impl(int old_width, int old_height);
  virtual void paint_impl(void) = 0;
  void update_camera(void);

  Camera camera;
  CamTrackball controller;
  int width = 0;
  int height = 0;
};

GL_NAMESPACE_END

#endif // GL_CONTEXT_HEADER
