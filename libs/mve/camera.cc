/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <fstream>

#include "math/matrix_tools.h"
#include "mve/camera.h"

MVE_NAMESPACE_BEGIN

CameraInfo::CameraInfo (void)
{
    this->flen = 0.0f;
    this->paspect = 1.0f;
    std::fill(this->ppoint, this->ppoint + 2, 0.5f);
    std::fill(this->dist, this->dist + 2, 0.0f);

    std::fill(this->trans, this->trans + 3, 0.0f);
    math::matrix_set_identity(this->rot, 3);
}

/* ---------------------------------------------------------------- */

// Must set the rotation before the translation
void
CameraInfo::fill_camera_pos (float* pos) const
{
    pos[0] = -rot[0] * trans[0] - rot[3] * trans[1] - rot[6] * trans[2];
    pos[1] = -rot[1] * trans[0] - rot[4] * trans[1] - rot[7] * trans[2];
    pos[2] = -rot[2] * trans[0] - rot[5] * trans[1] - rot[8] * trans[2];
}

/* Same as above, but with double precision. */
// Must set the rotation before the translation
void
CameraInfo::fill_camera_pos (double* pos) const
{
    pos[0] = -rot[0] * trans[0] - rot[3] * trans[1] - rot[6] * trans[2];
    pos[1] = -rot[1] * trans[0] - rot[4] * trans[1] - rot[7] * trans[2];
    pos[2] = -rot[2] * trans[0] - rot[5] * trans[1] - rot[8] * trans[2];
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_camera_translation (float* trans) const
{
    std::copy(this->trans, this->trans + 3, trans);
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_viewing_direction (float* viewdir) const
{
    for (int i = 0; i < 3; ++i)
        viewdir[i] = this->rot[6 + i];
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_world_to_cam (float* mat) const
{
    mat[0]  = rot[0]; mat[1]  = rot[1]; mat[2]  = rot[2]; mat[3]  = trans[0];
    mat[4]  = rot[3]; mat[5]  = rot[4]; mat[6]  = rot[5]; mat[7]  = trans[1];
    mat[8]  = rot[6]; mat[9]  = rot[7]; mat[10] = rot[8]; mat[11] = trans[2];
    mat[12] = 0.0f;   mat[13] = 0.0f;   mat[14] = 0.0f;   mat[15] = 1.0f;
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_gl_viewtrans (float* mat) const
{
    mat[0]  =  rot[0]; mat[1]  =  rot[1]; mat[2]  =  rot[2]; mat[3]  =  trans[0];
    mat[4]  = -rot[3]; mat[5]  = -rot[4]; mat[6]  = -rot[5]; mat[7]  = -trans[1];
    mat[8]  = -rot[6]; mat[9]  = -rot[7]; mat[10] = -rot[8]; mat[11] = -trans[2];
    mat[12] =  0.0f;   mat[13] =  0.0f;   mat[14] =  0.0f;   mat[15] = 1.0f;
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_cam_to_world (float* mat) const
{
    mat[0]  = rot[0]; mat[1] = rot[3]; mat[2]  = rot[6];
    mat[4]  = rot[1]; mat[5] = rot[4]; mat[6]  = rot[7];
    mat[8]  = rot[2]; mat[9] = rot[5]; mat[10] = rot[8];
    mat[3]  = -(rot[0] * trans[0] + rot[3] * trans[1] + rot[6] * trans[2]);
    mat[7]  = -(rot[1] * trans[0] + rot[4] * trans[1] + rot[7] * trans[2]);
    mat[11] = -(rot[2] * trans[0] + rot[5] * trans[1] + rot[8] * trans[2]);
    mat[12] = 0.0f;   mat[13] = 0.0f;   mat[14] = 0.0f;   mat[15] = 1.0f;
}

void
CameraInfo::fill_world_to_cam_rot (float* mat) const
{
    std::copy(this->rot, this->rot + 9, mat);
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_cam_to_world_rot (float* mat) const
{
    mat[0]  = rot[0]; mat[1] = rot[3]; mat[2] = rot[6];
    mat[3]  = rot[1]; mat[4] = rot[4]; mat[5] = rot[7];
    mat[6]  = rot[2]; mat[7] = rot[5]; mat[8] = rot[8];
}

/* Same thing, but with doubles*/

void
CameraInfo::fill_cam_to_world_rot (double* mat) const
{
    mat[0]  = rot[0]; mat[1] = rot[3]; mat[2] = rot[6];
    mat[3]  = rot[1]; mat[4] = rot[4]; mat[5] = rot[7];
    mat[6]  = rot[2]; mat[7] = rot[5]; mat[8] = rot[8];
}

/* ---------------------------------------------------------------- */

void
CameraInfo::set_transformation (float const* mat)
{
    rot[0] = mat[0]; rot[1] = mat[1]; rot[2] = mat[2];  trans[0] = mat[3];
    rot[3] = mat[4]; rot[4] = mat[5]; rot[5] = mat[6];  trans[1] = mat[7];
    rot[6] = mat[8]; rot[7] = mat[9]; rot[8] = mat[10]; trans[2] = mat[11];
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_calibration (float* mat, float width, float height) const
{
    float dim_aspect = width / height;
    float image_aspect = dim_aspect * this->paspect;
    float ax, ay;
    if (image_aspect < 1.0f) /* Portrait. */
    {
        ax = this->flen * height / this->paspect;
        ay = this->flen * height;
    }
    else /* Landscape. */
    {
        ax = this->flen * width;
        ay = this->flen * width * this->paspect;
    }

    mat[0] =   ax; mat[1] = 0.0f; mat[2] = width * this->ppoint[0];
    mat[3] = 0.0f; mat[4] =   ay; mat[5] = height * this->ppoint[1];
    mat[6] = 0.0f; mat[7] = 0.0f; mat[8] = 1.0f;
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_gl_projection (float* mat, float width, float height,
    float znear, float zfar) const
{
    float dim_aspect = width / height;
    float image_aspect = dim_aspect * this->paspect;
    float ax, ay;
    if (image_aspect < 1.0f) /* Portrait. */
    {
        ax = this->flen / image_aspect;
        ay = this->flen;
    }
    else /* Landscape */
    {
        ax = this->flen;
        ay = this->flen * image_aspect;
    }

    std::fill(mat, mat + 16, 0.0f);

    mat[4 * 0 + 0] = 2.0f * ax;
    mat[4 * 0 + 2] = 2.0f * (this->ppoint[0] - 0.5f);
    mat[4 * 1 + 1] = 2.0f * ay;
    mat[4 * 1 + 2] = 2.0f * (this->ppoint[1] - 0.5f);
    mat[4 * 2 + 2] = -(zfar + znear) / (zfar - znear);
    mat[4 * 2 + 3] = -2.0f * zfar * znear / (zfar - znear);
    mat[4 * 3 + 2] = -1.0f;
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_inverse_calibration (float* mat,
    float width, float height) const
{
    float dim_aspect = width / height;
    float image_aspect = dim_aspect * this->paspect;
    float ax, ay;
    if (image_aspect < 1.0f) /* Portrait. */
    {
        ax = this->flen * height / this->paspect;
        ay = this->flen * height;
    }
    else /* Landscape. */
    {
        ax = this->flen * width;
        ay = this->flen * width * this->paspect;
    }

    mat[0] = 1.0f / ax; mat[1] = 0.0f; mat[2] = -width * this->ppoint[0] / ax;
    mat[3] = 0.0f; mat[4] = 1.0f / ay; mat[5] = -height * this->ppoint[1] / ay;
    mat[6] = 0.0f; mat[7] = 0.0f;      mat[8] = 1.0f;
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_reprojection (CameraInfo const& destination,
    float src_width, float src_height, float dst_width, float dst_height,
    float* mat, float* vec) const
{
    math::Matrix3f dst_K, dst_R, src_Ri, src_Ki;
    math::Vec3f dst_t, src_t;
    destination.fill_calibration(dst_K.begin(), dst_width, dst_height);
    destination.fill_world_to_cam_rot(dst_R.begin());
    destination.fill_camera_translation(dst_t.begin());
    this->fill_cam_to_world_rot(src_Ri.begin());
    this->fill_inverse_calibration(src_Ki.begin(), src_width, src_height);
    this->fill_camera_translation(src_t.begin());

    math::Matrix3f ret_mat = dst_K * dst_R * src_Ri * src_Ki;
    math::Vec3f ret_vec = dst_K * (dst_t - dst_R * src_Ri * src_t);
    std::copy(ret_mat.begin(), ret_mat.end(), mat);
    std::copy(ret_vec.begin(), ret_vec.end(), vec);
}

/* ---------------------------------------------------------------- */

std::string
CameraInfo::get_rotation_string (void) const
{
    std::stringstream ss;
    ss << std::setprecision(10);
    for (int i = 0; i < 9; ++i)
        ss << this->rot[i] << (i < 8 ? " " : "");
    return ss.str();
}

std::string
CameraInfo::get_translation_string (void) const
{
    std::stringstream ss;
    ss << std::setprecision(10);
    for (int i = 0; i < 3; ++i)
        ss << this->trans[i] << (i < 2 ? " " : "");
    return ss.str();
}

void
CameraInfo::set_translation_from_string (std::string const& trans_string)
{
    std::stringstream ss(trans_string);
    for (int i = 0; i < 3; ++i)
        ss >> this->trans[i];
}

void
CameraInfo::set_rotation_from_string (std::string const& rot_string)
{
    std::stringstream ss(rot_string);
    for (int i = 0; i < 9; ++i)
        ss >> this->rot[i];
}

void CameraInfo::read_tsai(std::string const& filename) { 
    
  // TODO(oalexan1): Must convert all entries to double, and read below with
  // %lf and not %f. For now just the camera position and orientation are
  // read as double, which are the most important ones.

  // Open the input file
  std::ifstream cam_file;
  cam_file.open(filename.c_str());

  std::string text; 
  if (cam_file.fail()) {
    text = "PinholeModel::read_file: Could not open file: " + filename;
    throw std::invalid_argument(text.c_str());
  }

  // Check for version number on the first line
  int file_version = 1; // The default version written before the 2016 changes
  std::string line;
  std::getline(cam_file, line);
  if (line.find("VERSION") != std::string::npos) {
    sscanf(line.c_str(),"VERSION_%d", &file_version); // Parse the version of the input file

    std::getline(cam_file, line); // Go ahead and get the next line

    // If we get version 3, continue on to the rest of the file.
    if (file_version == 4) {
      // Version 4 added support for multiple camera types.
      if (line.find("PINHOLE") == std::string::npos) {
        text = "PinholeModel::read_file: Expected PINHOLE type, but got type "
                                + line;
        throw std::invalid_argument(text.c_str());
      }
      std::getline(cam_file, line); // Grab the following line
    }

  }else{
    // Check if the first line is correct
    if (sscanf(line.c_str(),"fu = %f", &this->flen) != 1) {
      text = std::string("PinholeModel::read_file(): A Pinhole camera file must ")
            + "start either with the VERSION line or the focal length. Got instead:\n"
            + line;
        throw std::invalid_argument(text.c_str());
    }
  }

  // Do not read the true focal length as when it is large the frustrum is displayed
  // poorly.  
  this->flen = 1.0f;

  // Start parsing all the parameters from the lines.
  double f_u;
  if (!cam_file.good() || sscanf(line.c_str(),"fu = %lf", &f_u) != 1) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the x focal length.\n";
    throw std::invalid_argument(text.c_str());
  }

    //std::cout << "Not reading true focal length and optical center "
    //    << "for rendering reasons.\n";

  double f_v;
  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"fv = %lf", &f_v) != 1) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the y focal length\n";
    throw std::invalid_argument(text.c_str());
  }

  if (f_u != f_v)
        std::cerr << "Expecting the focal length in the x and y directions to be the "
            << "same. Ignoring the focal length in y.\n";
   
    // Do not read the true optical center as when it is large the frustum is displayed
    // poorly.
    ppoint[0] = 0.5f;
    ppoint[1] = 0.5f;

  double cu, cv;
  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"cu = %lf", &cu) != 1) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the x principal point\n";
    throw std::invalid_argument(text.c_str());
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"cv = %lf", &cv) != 1) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the y principal point\n";
    throw std::invalid_argument(text.c_str());
  }

  std::getline(cam_file, line);
  double u_direction[3];
  if (!cam_file.good() || sscanf(line.c_str(),"u_direction = %lf %lf %lf", 
        &u_direction[0], &u_direction[1], &u_direction[2]) != 3) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the u direction vector\n";
    throw std::invalid_argument(text.c_str());
  }

  std::getline(cam_file, line);
  double v_direction[3];
  if (!cam_file.good() || sscanf(line.c_str(),"v_direction = %lf %lf %lf", 
        &v_direction[0], &v_direction[1], &v_direction[2]) != 3) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the v direction vector\n";
    throw std::invalid_argument(text.c_str());
  }

  std::getline(cam_file, line);
  double w_direction[3];
  if (!cam_file.good() || sscanf(line.c_str(),"w_direction = %lf %lf %lf", 
        &w_direction[0], &w_direction[1], &w_direction[2]) != 3) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the w direction vector\n";
    throw std::invalid_argument(text.c_str());
  }

  // Read extrinsic parameters

  // Camera center in world coordinates
  std::getline(cam_file, line);
  double camera_center[3];
  if (!cam_file.good() || sscanf(line.c_str(),"C = %lf %lf %lf", 
        &camera_center[0], &camera_center[1], &camera_center[2]) != 3) {
    cam_file.close();
    text = "PinholeModel::read_file: Could not read C (the camera center) vector\n";
    throw std::invalid_argument(text.c_str());
  }

  // Read the rotation matrix, the transform from the camera to the world
  double rotation[9];
  std::getline(cam_file, line);
  if ( !cam_file.good() ||
       sscanf(line.c_str(), "R = %lf %lf %lf %lf %lf %lf %lf %lf %lf",
              &rotation[0], &rotation[1], &rotation[2],
              &rotation[3], &rotation[4], &rotation[5],
              &rotation[6], &rotation[7], &rotation[8]) != 9 ) {
    cam_file.close();
    text = "PinholeModel::read_file(): Could not read the rotation matrix\n";
    throw std::invalid_argument(text.c_str());
  }

    // Find the transpose of the rotation matrix,
    // because we want to go from world to camera coordinates.
    this->rot[0] = rotation[0];
    this->rot[1] = rotation[3];
    this->rot[2] = rotation[6];
    this->rot[3] = rotation[1];
    this->rot[4] = rotation[4];
    this->rot[5] = rotation[7];
    this->rot[6] = rotation[2];
    this->rot[7] = rotation[5];
    this->rot[8] = rotation[8];

    // Go from camera center in world coordinates to the vector 'trans'.
    // trans = -inverse(camera2world) * camera_center
    // trans = - world2camera * camera_center
    this->trans[0] = -(this->rot[0]*camera_center[0] + this->rot[1]*camera_center[1] + this->rot[2]*camera_center[2]);
    this->trans[1] = -(this->rot[3]*camera_center[0] + this->rot[4]*camera_center[1] + this->rot[5]*camera_center[2]);
    this->trans[2] = -(this->rot[6]*camera_center[0] + this->rot[7]*camera_center[1] + this->rot[8]*camera_center[2]);

#if 0
  // Ignore the rest of the file for now

  // The pitch line does not exist in older files, so don't fail if it is missing.
  // - At this point we need to hang on to the position immediately before the 
  int lens_start = cam_file.tellg();
  std::getline(cam_file, line);
  if (line.find("pitch") != std::string::npos) {
    if (!cam_file.good() || sscanf(line.c_str(),"pitch = %lf", &m_pixel_pitch) != 1) {
      cam_file.close();
      vw_throw( IOErr() << "PinholeModel::read_file(): Could not read the pixel pitch\n" );
    }
    lens_start = cam_file.tellg();
    std::getline(cam_file, line); // After reading the pitch, read the next line.
  }
  else // Pixel pitch not specified, use 1.0 as the default.
  {
    if (file_version > 2){
      cam_file.close();
      vw_throw( IOErr() << "PinholeModel::read_file(): Pitch value required in this file version!\n" );
    }
    m_pixel_pitch = 1.0;
  }

  // Now that we have loaded all the parameters, update the dependent class members.
  this->rebuild_camera_matrix();

  // This creates m_distortion but we still need to read the parameters.
  bool found_name = construct_lens_distortion(line, file_version);

  if (!found_name && (file_version > 2)){
    cam_file.close();
    vw_throw( IOErr() << "PinholeModel::read_file(): Distortion name required in this file version!\n" );
  }
  
  // If there was no line containing the distortion model name (true for old files)
  //  then we need to back up to before the distortion parameters begin in the file.
  if (!found_name)
    cam_file.seekg(lens_start, std::ios_base::beg);    

  // The lens distortion class knows how to parse the rest of the input stream.
  m_distortion->read(cam_file);
#endif
}
/* ---------------------------------------------------------------- */

void
CameraInfo::debug_print (void) const
{
    std::cout << "Extrinsic camera parameters:" << std::endl
        << "  Trans: " << math::Vec3f(this->trans) << std::endl
        << "  Rot: " << math::Vec3f(&this->rot[0]) << std::endl
        << "       " << math::Vec3f(&this->rot[3]) << std::endl
        << "       " << math::Vec3f(&this->rot[6]) << std::endl
        << "Intrinsic camera parameters:" << std::endl
        << "  Focal Length: " << this->flen << std::endl
        << "  Principal Point: " << math::Vec2f(this->ppoint) << std::endl
        << "  Pixel aspect: " << this->paspect << std::endl
        << "  Radial distortion: " << math::Vec2f(this->dist) << std::endl
        << std::endl;
}

MVE_NAMESPACE_END
