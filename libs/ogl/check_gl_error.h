/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_CHECK_GL_ERROR_HEADER
#define OGL_CHECK_GL_ERROR_HEADER

#include <stdexcept>
#include <string>

#include "ogl/defines.h"
#include "ogl/opengl.h"

OGL_NAMESPACE_BEGIN

inline void
check_gl_error()
{
    GLenum err = glGetError();
    if (err != GL_NO_ERROR)
        throw std::runtime_error("GL error: " + std::to_string(err));
}

OGL_NAMESPACE_END

#endif
