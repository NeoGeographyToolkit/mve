// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#include "SceneManager.h"

SceneManager::SceneManager (void)
{
}

SceneManager::~SceneManager (void)
{
}

SceneManager&
SceneManager::get (void)
{
    static SceneManager instance;
    return instance;
}
