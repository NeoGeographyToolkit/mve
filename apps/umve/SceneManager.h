// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef SFM_SCENE_MANAGER_HEADER
#define SFM_SCENE_MANAGER_HEADER

#include <QObject>

#include "SfmUtils.h"

// Singleton signal hub for scene and view selection.
// SfmMainWindow selects the scene, SceneOverview selects views.
class SceneManager: public QObject {
  Q_OBJECT

private:
  sfm::Scene::Ptr scene;
  sfm::View::Ptr view;

signals:
  void scene_selected(sfm::Scene::Ptr scene);
  void view_selected(sfm::View::Ptr view);

public:
  SceneManager(void);
  ~SceneManager(void);
  static SceneManager& get(void);

  void select_scene(sfm::Scene::Ptr scene);
  void select_view(sfm::View::Ptr view);

  sfm::Scene::Ptr get_scene(void);

  void reset_scene(void);
  void reset_view(void);
};

inline void SceneManager::select_scene(sfm::Scene::Ptr scene) {
  this->scene = scene;
  emit this->scene_selected(scene);
}

inline void SceneManager::select_view(sfm::View::Ptr view) {
  this->view = view;
  emit this->view_selected(view);
}

inline sfm::Scene::Ptr SceneManager::get_scene(void) {
  return this->scene;
}

inline void SceneManager::reset_scene(void) {
  this->select_scene(sfm::Scene::Ptr());
}

inline void SceneManager::reset_view(void) {
  this->select_view(sfm::View::Ptr());
}

#endif // SFM_SCENE_MANAGER_HEADER
