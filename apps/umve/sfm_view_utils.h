// Copyright (c) 2015, Simon Fuhrmann (TU Darmstadt)
// Copyright (c) 2023-2026, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration.
// All rights reserved.
// BSD 3-Clause license. See LICENSE.txt.
//
// Lightweight replacements for mve::CameraInfo, mve::View, and mve::Scene
// used by sfm_view. These avoid pulling in the full MVE library for types
// that are simple data containers in this application.

#ifndef SFM_VIEW_UTILS_HEADER
#define SFM_VIEW_UTILS_HEADER

#include <memory>
#include <string>
#include <vector>

namespace sfm {

// Camera intrinsic and extrinsic parameters.
// Stores world-to-camera rotation and translation.
struct CameraInfo {
  CameraInfo();

  // Compute camera position: pos = -R^T * t. Output is 3 floats.
  void fill_camera_pos(float* pos) const;

  // Store the viewing direction (third row of rotation). Output is 3 floats.
  void fill_viewing_direction(float* viewdir) const;

  // Store the 4x4 camera-to-world matrix. Output is 16 floats.
  void fill_cam_to_world(float* mat) const;

  // Read camera parameters from a .tsai pinhole model file.
  void read_tsai(std::string const& filename);

  // Intrinsic parameters
  float flen;

  // Extrinsic parameters (double for large satellite orbit values)
  double trans[3]; // Camera translation: pos = -R^T * trans
  double rot[9];   // World-to-camera rotation (row-major 3x3)
};

// A single camera view: holds a name and camera parameters.
class View {
public:
  using Ptr = std::shared_ptr<View>;
  static Ptr create();

  // Load camera from a .tsai file. The view name is set to the
  // basename of camera_path.
  void load_view(std::string const& image_path,
                 std::string const& camera_path);

  std::string const& get_name() const;

  CameraInfo const& get_camera() const;
  void set_camera(CameraInfo const& cam);

  // No-op. Kept for API compatibility with code that prevents save prompts.
  void set_dirty(bool dirty);

private:
  View() = default;
  std::string name_;
  CameraInfo camera_;
};

// A collection of camera views.
class Scene {
public:
  using Ptr = std::shared_ptr<Scene>;
  using ViewList = std::vector<View::Ptr>;

  // Create a scene from paired image and .tsai camera files.
  static Ptr create(std::vector<std::string> const& image_files,
                    std::vector<std::string> const& camera_files);

  ViewList const& get_views() const;
  ViewList& get_views();
  View::Ptr get_view_by_id(std::size_t id);

private:
  Scene() = default;
  ViewList views_;
};

// Inline implementations

inline View::Ptr View::create() {
  return Ptr(new View);
}

inline std::string const& View::get_name() const {
  return name_;
}

inline CameraInfo const& View::get_camera() const {
  return camera_;
}

inline void View::set_camera(CameraInfo const& cam) {
  camera_ = cam;
}

inline void View::set_dirty(bool /*dirty*/) {
}

inline Scene::ViewList const& Scene::get_views() const {
  return views_;
}

inline Scene::ViewList& Scene::get_views() {
  return views_;
}

inline View::Ptr Scene::get_view_by_id(std::size_t id) {
  if (id < views_.size())
    return views_[id];
  return View::Ptr();
}

} // namespace sfm

#endif // SFM_VIEW_UTILS_HEADER
