// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef GL_CONTEXT_HEADER
#define GL_CONTEXT_HEADER

#include "SfmMath.h"
#include "GlCommon.h"

namespace sfm {

// Trackball camera control that consumes mouse events.
class CamTrackball {
public:
  CamTrackball(void);

  void set_camera(SfmCamera* camera);
  bool consume_event(MouseEvent const& event);

  sfm::Vec3f get_campos(void) const;
  sfm::Vec3f get_viewdir(void) const;
  sfm::Vec3f const& get_upvec(void) const;

private:
  void handle_tb_rotation(int x, int y);
  sfm::Vec3f get_ball_normal(int x, int y);

  SfmCamera* cam;

  float tb_radius;
  sfm::Vec3f tb_center;
  sfm::Vec3f tb_tocam;
  sfm::Vec3f tb_upvec;

  int rot_mouse_x;
  int rot_mouse_y;
  sfm::Vec3f rot_tb_tocam;
  sfm::Vec3f rot_tb_upvec;

  float zoom_tb_radius;
  int zoom_mouse_y;
};

inline void CamTrackball::set_camera(SfmCamera* camera) {
  this->cam = camera;
}

inline sfm::Vec3f CamTrackball::get_campos(void) const {
  return this->tb_center + this->tb_tocam * this->tb_radius;
}

inline sfm::Vec3f CamTrackball::get_viewdir(void) const {
  return -this->tb_tocam;
}

inline sfm::Vec3f const& CamTrackball::get_upvec(void) const {
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

  SfmCamera camera;
  CamTrackball controller;
  int width = 0;
  int height = 0;
};

}

#endif // GL_CONTEXT_HEADER
