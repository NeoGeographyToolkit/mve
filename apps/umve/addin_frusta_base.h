/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UMVE_ADDIN_FRUSTA_BASE_HEADER
#define UMVE_ADDIN_FRUSTA_BASE_HEADER

#include "mve/mesh.h"
#include "sfm_view_utils.h"

void add_camera_to_mesh (sfm::CameraInfo const& camera,
    float size, mve::TriangleMesh::Ptr mesh);

#endif /* UMVE_ADDIN_FRUSTA_BASE_HEADER */
