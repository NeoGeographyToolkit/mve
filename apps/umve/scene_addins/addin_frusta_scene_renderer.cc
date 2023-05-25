/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include "guihelpers.h"

#include "scenemanager.h"
#include "scene_addins/addin_frusta_base.h"
#include "scene_addins/addin_frusta_scene_renderer.h"

#include <iostream>

AddinFrustaSceneRenderer::AddinFrustaSceneRenderer (void)
{
    this->render_frusta_cb = new QCheckBox("Draw camera frusta");
    this->render_viewdir_cb = new QCheckBox("Draw viewing direction");
    this->render_frusta_cb->setChecked(true);
    this->render_viewdir_cb->setChecked(true);

    this->frusta_size_slider = new QSlider();
    this->frusta_size_slider->setMinimum(1);
    this->frusta_size_slider->setMaximum(100);
    this->frusta_size_slider->setValue(10);
    this->frusta_size_slider->setOrientation(Qt::Horizontal);

    /* Create frusta rendering layout. */
    this->render_frusta_form = new QFormLayout();
    this->render_frusta_form->setVerticalSpacing(0);
    this->render_frusta_form->addRow(this->render_frusta_cb);
    this->render_frusta_form->addRow(this->render_viewdir_cb);
    this->render_frusta_form->addRow("Frusta Size:", this->frusta_size_slider);

    this->connect(&SceneManager::get(), SIGNAL(scene_bundle_changed()),
        this, SLOT(reset_frusta_renderer()));
    this->connect(&SceneManager::get(), SIGNAL(scene_selected(mve::Scene::Ptr)),
        this, SLOT(reset_frusta_renderer()));
    this->connect(&SceneManager::get(), SIGNAL(view_selected(mve::View::Ptr)),
        this, SLOT(reset_viewdir_renderer()));
    this->connect(this->frusta_size_slider, SIGNAL(valueChanged(int)),
        this, SLOT(reset_frusta_renderer()));
    this->connect(this->frusta_size_slider, SIGNAL(valueChanged(int)),
        this, SLOT(repaint()));
    this->connect(this->render_frusta_cb, SIGNAL(clicked()),
        this, SLOT(repaint()));
    this->connect(this->render_viewdir_cb, SIGNAL(clicked()),
        this, SLOT(repaint()));
}

QWidget*
AddinFrustaSceneRenderer::get_sidebar_widget (void)
{
    return get_wrapper(this->render_frusta_form);
}

void
AddinFrustaSceneRenderer::reset_frusta_renderer (void)
{
    this->frusta_renderer.reset();
}

void
AddinFrustaSceneRenderer::reset_viewdir_renderer (void)
{
    this->viewdir_renderer.reset();
}

void
AddinFrustaSceneRenderer::paint_impl (void)
{
    if (this->render_frusta_cb->isChecked())
    {
        if (this->frusta_renderer == nullptr)
            this->create_frusta_renderer();
        if (this->frusta_renderer != nullptr)
            this->frusta_renderer->draw();
    }

    if (this->render_viewdir_cb->isChecked())
    {
        if (this->viewdir_renderer == nullptr)
            this->create_viewdir_renderer();
        if (this->viewdir_renderer != nullptr)
            this->viewdir_renderer->draw();
    }
}

// TODO(oalexan1): Move this function to utils somewhere.

// Find shortest distance from camera center to the origin
double find_shortest_distance(std::vector<math::Vec3d> centers) {
    // If the vector is empty, return 1.0
    if (centers.size() == 0)
        return 1.0;
    
    double shortest = centers[0].norm();
    for (std::size_t i = 1; i < centers.size(); i++) {
        double dist = centers[i].norm();
        if (dist < shortest)
            shortest = dist;
    }
    return shortest;
}

// Function to find the mean camera position
math::Vec3d find_mean_camera_pos(std::vector<math::Vec3d> const& centers) {
    // If the vector is empty, return 0,0,0
    if (centers.size() == 0)
        return math::Vec3d(0,0,0);
    
    math::Vec3d mean(0,0,0);
    for (std::size_t i = 0; i < centers.size(); i++) {
        mean += centers[i];
    }
    mean /= centers.size();
    return mean;
}

// Collect the cam2world matrices and camera centers
// in vectors.
void extractCameraPoses(mve::Scene::ViewList const & views,
    // Outputs
    std::vector<math::Vec3d> & cam_centers,
    std::vector<math::Matrix3d> & cam2world_vec) {

    // First wipe the outputs
    cam_centers.clear();
    cam2world_vec.clear();

    for (std::size_t i = 0; i < views.size(); i++) {
    
        if (views[i].get() == nullptr) {
            std::cerr << "Error: Empty camera.\n";
            continue;
        }
        
        mve::CameraInfo const& cam = views[i]->get_camera(); // alias

        // The cameras store trans = -inverse(camera2world) * camera_center
        // and rot = inverse(camera2world).
        // We need to invert the camera2world matrix to get the camera center.
        // So, need to find:
        // and camera2world = transpose(rot)
        // camera_center = -camera2world * trans

        math::Matrix3d world2cam(cam.rot);
        math::Vec3d t(cam.trans);
        math::Matrix3d cam2world = world2cam.transposed();
        math::Vec3d ctr = -cam2world.mult(t);

        // Append to the vector of centers and rotations
        cam_centers.push_back(ctr);
        cam2world_vec.push_back(cam2world);
    }

    return;
}

// Apply the camera2world transforms and camera centers to the cameras in the scene
void applyCameraPoses(std::vector<math::Vec3d>    const & cam_centers,
                      std::vector<math::Matrix3d> const & cam2world_vec,
                      mve::Scene::ViewList              & views) {

    for (std::size_t i = 0; i < views.size(); i++) {
        if (views[i].get() == nullptr) {
            std::cerr << "Error: Empty camera.\n";
            continue;
        }
        
        // Make a copy of the camera
        mve::CameraInfo cam = views[i]->get_camera(); 

        // Compute T = -cam2word^T * C and copy to the camera
        math::Vec3d t = -cam2world_vec[i].transposed().mult(cam_centers[i]);
        for (int coord = 0; coord < 3; coord++)
            cam.trans[coord] = t[coord];
        
        // Compute R = cam2world^T and copy it to the camera
        math::Matrix3d R = cam2world_vec[i].transposed();
        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
                cam.rot[row * 3 + col] = R(row, col);

        views[i]->set_camera(cam);
        views[i]->set_dirty(false); // so it does not ask to save the scene on quitting
    }
    return;
}

// Given a vector, find a rotation matrix that rotates the vector to the y-axis.
void completeVectorToRotation(math::Vec3d & y, math::Matrix3d & R) {

    int largest_comp = 0;
    for (int i = 1; i < 3; i++) {
        if (std::abs(y[i]) > std::abs(y[largest_comp]))
            largest_comp = i;
    }

    int j = largest_comp + 1;
    if (j == 3) 
        j = 0;
    
    math::Vec3d x(0.0, 0.0, 0.0);
    x[j] = y[largest_comp];
    x[largest_comp] = -y[j];

    if (std::abs(y[largest_comp]) == 0.0) {
        // Handle degenerate case, will return the identity matrix
        x = math::Vec3d(1.0, 0.0, 0.0);
        y = math::Vec3d(0.0, 1.0, 0.0);
    }

    x = x / x.norm();
    y = y / y.norm();

    // Find y as the cross product of plane normal and x
    math::Vec3d z = x.cross(y);
    z = z / z.norm();

    // Find the matrix with x, y, z as columns
    for (int row = 0; row < 3; row++) {
        R(row, 0) = x[row];
        R(row, 1) = y[row];
        R(row, 2) = z[row];
    }

    return;
}

// Compute a ground plane, scale the orbital camera positions relative to the plane,
// then plot the cameras and the ground plane.
// TODO(oalexan1): Connect this to the gui checkboxes that can turn on and off
// showing the cameras and the ground plane.
void AddinFrustaSceneRenderer::create_frusta_renderer (void) {

    if (this->state->scene == nullptr) {
        // Comment this out because this customized viewer will not load scenes
        // std::cerr << "Error: No scene loaded.\n";
        return;
    }

    // Find the camera centers and camera2world matrices
    std::vector<math::Vec3d> cam_centers;
    std::vector<math::Matrix3d> cam2world_vec;
    mve::Scene::ViewList & views(this->state->scene->get_views());
    extractCameraPoses(views, cam_centers, cam2world_vec);

    // Find shortest distance from camera center to the origin
    double shortest_dist = find_shortest_distance(cam_centers);

    // Divide by the shortest distance, if at least 10.0
    for (size_t cam = 0; cam < cam_centers.size(); cam++)
            cam_centers[cam] = cam_centers[cam] / shortest_dist;

    // Find the mean camera center, use it as normal to the ground plane
    math::Vec3d mean_cam_center = find_mean_camera_pos(cam_centers);
    
    // Will map the cameras so that the ground plane is the x-z plane
    // This is consistent with how OpenGL by default renders this plane
    // as horizontal (z axis pointing to user, y goes up, x goes right).
    math::Vec3d y = mean_cam_center; // will change below
    math::Matrix3d R;
    completeVectorToRotation(y, R);

    // Inverse matrix
    math::Matrix3d Rinv = R.transposed();

    // Find the bounding box of the camera centers in the coordinate system
    // having the ground plane as x and z axes. The y axis will point up,
    // consistent with the OpenGL default.
    double x_min = std::numeric_limits<double>::max(),
           z_min = x_min, x_max = -x_min, z_max = x_max;
    for (std::size_t i = 0; i < cam_centers.size(); i++) {
        // Transform the camera center to the new coordinate system
        math::Vec3d cam_center = cam_centers[i];
        math::Vec3d trans_center = Rinv.mult(cam_center);
        x_min = std::min(x_min, trans_center[0]);
        x_max = std::max(x_max, trans_center[0]);
        z_min = std::min(z_min, trans_center[2]);
        z_max = std::max(z_max, trans_center[2]);
    }
    double len = std::max(x_max - x_min, z_max - z_min);
    if (len == 0.0)
        len = 1.0; // careful not to divide by zero
    double mid_x = (x_min + x_max)/2.0;
    double mid_z = (z_min + z_max)/2.0;
    std::cout << "mid_x = " << mid_x << "\n";
    std::cout << "mid_z = " << mid_z << "\n";
    std::cout << "max_x - min_x = " << x_max - x_min << "\n";
    std::cout << "max_z - min_z = " << z_max - z_min << "\n";

    // Rotate the camera centers so that the ground plane is the x-z plane
    // then normalize them to be to be on the order of 1.0. Rotate
    // the camera orientations as well.
    for (std::size_t i = 0; i < cam_centers.size(); i++) {
        // Transform the camera center to the new coordinate system
        math::Vec3d cam_center = cam_centers[i];
        math::Vec3d trans_center = Rinv.mult(cam_center);
        trans_center[0] = (trans_center[0] - mid_x)/len;
        std::cout << "The height below is wrong!\n";
        trans_center[1] = 0.2;
        trans_center[2] = (trans_center[2] - mid_z)/len;
        cam_centers[i] = R.mult(trans_center);
        //cam2world_vec[i] = Rinv.mult(cam2world_vec[i]);
    }

    if (z_max - z_min > x_max - x_min) {
        // Compute the 90 degree rotation in the x-z plane
        math::Matrix3d R90;
        R90[0] = 0; R90[1] = 0; R90[2] = -1;
        R90[3] = 0; R90[4] = 1; R90[5] = 0;
        R90[6] = 1; R90[7] = 0; R90[8] = 0;
        Rinv = R90.mult(Rinv);
    }

    // Rotate the camera positions and orientations
    for (std::size_t i = 0; i < cam_centers.size(); i++) {
        cam_centers[i] = Rinv.mult(cam_centers[i]);
        cam2world_vec[i] = Rinv.mult(cam2world_vec[i]);
    }


    applyCameraPoses(cam_centers, cam2world_vec, views);

    // Plot the cameras as meshes
    // TODO(oalexan1): Add a switch for turning this off.
    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
    mve::TriangleMesh::FaceList& faces(mesh->get_faces());
    mve::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());
    math::Vec4f color(0, 1, 0, 1); // green
    float size = this->frusta_size_slider->value() / 100.0f;
    for (std::size_t i = 0; i < views.size(); i++) {
        if (views[i].get() == nullptr) {
            std::cerr << "Error: Empty camera.\n";
            continue;
        }

        mve::CameraInfo const& cam = views[i]->get_camera();
        if (cam.flen == 0.0f) {
            std::cerr << "Error: Camera focal length is 0.\n";
            continue;
        }
        add_camera_to_mesh(cam, size, mesh);
    }

    // Plot the ground plane half way between camera centers and origin
    // TODO(oalexan1): Make this plane filled and in different color.
    // TODO(oalexan1): Add a switch for turning this off.
    double d = 10.0;
    for (size_t i = 1; i <= 22; i++) {
        double x[2], z[2];
        if (i <= 11) {
            z[0] = 2.0*(i - 6.0)/d; z[1] = z[0];
            x[0] = -1.0; x[1] = 1.0;
        } else {
            x[0] = 2.0*(i - 11 - 6.0)/d; x[1] = x[0];
            z[0] = -1.0; z[1] = 1.0;
        }
    
        // The cameras are roughly at y = 1 before applying the rotation R,
        // so here we put the plane at y = ht, so it is more down.
        double ht = -0.8;
        math::Vec3d v1(x[0], ht, z[0]);
        math::Vec3d v2(x[1], ht, z[1]);

        verts.push_back(v1);
        verts.push_back(v2);
        faces.push_back(verts.size() - 2);
        faces.push_back(verts.size() - 1);
        colors.push_back(color);
        colors.push_back(color);
    }

    this->frusta_renderer = ogl::MeshRenderer::create(mesh);
    this->frusta_renderer->set_shader(this->state->wireframe_shader);
    this->frusta_renderer->set_primitive(GL_LINES);
}

void
AddinFrustaSceneRenderer::create_viewdir_renderer (void)
{
    if (this->state->view == nullptr)
        return;

    math::Vec3f campos, viewdir;
    mve::CameraInfo const& cam(this->state->view->get_camera());
    cam.fill_camera_pos(*campos);
    cam.fill_viewing_direction(*viewdir);

    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    mve::TriangleMesh::VertexList& verts = mesh->get_vertices();
    mve::TriangleMesh::ColorList& colors = mesh->get_vertex_colors();
    verts.push_back(campos);
    verts.push_back(campos + viewdir * 100.0f);
    colors.push_back(math::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));
    colors.push_back(math::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));

    this->viewdir_renderer = ogl::MeshRenderer::create(mesh);
    this->viewdir_renderer->set_shader(this->state->wireframe_shader);
    this->viewdir_renderer->set_primitive(GL_LINES);
}
