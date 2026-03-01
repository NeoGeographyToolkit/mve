/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <QVBoxLayout>

#include "util/strings.h"

#include "scenemanager.h"
#include "sceneoverview.h"

SceneOverview::SceneOverview (QWidget* parent)
    : QWidget(parent)
{
    this->viewlist = new QListWidget();
    this->viewlist->setEnabled(false);

    QVBoxLayout* vbox = new QVBoxLayout(this);
    vbox->setSpacing(0);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->addWidget(this->viewlist);

    this->connect(this->viewlist, SIGNAL(currentRowChanged(int)),
        this, SLOT(on_row_changed(int)));
    this->connect(&SceneManager::get(), SIGNAL(scene_selected(mve::Scene::Ptr)),
        this, SLOT(on_scene_changed(mve::Scene::Ptr)));
}

void
SceneOverview::on_scene_changed (mve::Scene::Ptr scene)
{
    this->viewlist->clear();
    this->viewlist->setEnabled(false);

    if (scene == nullptr)
        return;

    mve::Scene::ViewList& sl(scene->get_views());
    if (sl.empty()) {
        QListWidgetItem* item = new QListWidgetItem("Scene has no views!");
        item->setData(Qt::UserRole, -1);
        this->viewlist->addItem(item);
    } else {
        this->viewlist->setEnabled(true);
    }

    for (std::size_t i = 0; i < sl.size(); ++i) {
        mve::View::Ptr view(sl[i]);
        if (view == nullptr)
            continue;
        this->add_view_to_layout(i, view);
    }
}

void
SceneOverview::add_view_to_layout (std::size_t id, mve::View::Ptr view)
{
    if (view == nullptr)
        return;

    std::string const& view_name = view->get_name();
    std::string view_id = util::string::get(view->get_id());
    bool cam_valid = view->get_camera().flen != 0.0f;

    QString name = QString("ID %2: %1")
        .arg(QString(view_name.c_str()), QString(view_id.c_str()));

    QListWidgetItem* item = new QListWidgetItem(name);
    if (!cam_valid)
        item->setBackgroundColor(QColor(255, 221, 221));
    item->setData(Qt::UserRole, (int)id);
    this->viewlist->addItem(item);
}

void
SceneOverview::on_row_changed (int id)
{
    if (id < 0)
        return;

    QListWidgetItem* item = this->viewlist->item(id);
    std::size_t view_id = (std::size_t)item->data(Qt::UserRole).toInt();

    mve::Scene::Ptr scene(SceneManager::get().get_scene());
    mve::View::Ptr view(scene->get_view_by_id(view_id));
    SceneManager::get().select_view(view);
}
