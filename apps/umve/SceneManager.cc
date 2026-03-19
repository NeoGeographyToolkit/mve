// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#include "SceneManager.h"

SceneManager::SceneManager(void) {
}

SceneManager::~SceneManager(void) {
}

SceneManager& SceneManager::get(void) {
  static SceneManager instance;
  return instance;
}

void SceneManager::select_scene(sfm::Scene::Ptr scene) {
  this->scene = scene;
  emit this->scene_selected(scene);
}

void SceneManager::select_view(sfm::View::Ptr view) {
  this->view = view;
  emit this->view_selected(view);
}

sfm::Scene::Ptr SceneManager::get_scene(void) {
  return this->scene;
}

void SceneManager::reset_scene(void) {
  this->select_scene(sfm::Scene::Ptr());
}

void SceneManager::reset_view(void) {
  this->select_view(sfm::View::Ptr());
}
