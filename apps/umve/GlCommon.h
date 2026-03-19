// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef GL_COMMON_HEADER
#define GL_COMMON_HEADER

#include <stdexcept>
#include <string>

/* --- OpenGL headers --- */

#if defined(OGL_USE_OSMESA)
#  define GL_GLEXT_PROTOTYPES
#  include <GL/osmesa.h>
#elif defined(__APPLE__)
#  include <OpenGL/gl3.h>
#elif defined(_WIN32)
#  include <GL/glew.h>
#else
#  define GL_GLEXT_PROTOTYPES
#  include <GL/gl.h>
#  include <GL/glext.h>
#endif

/* --- Namespace macros --- */

#define GL_NAMESPACE_BEGIN namespace gl {
#define GL_NAMESPACE_END }

GL_NAMESPACE_BEGIN

/* --- GL error checking --- */

inline void
check_gl_error()
{
    GLenum err = glGetError();
    if (err != GL_NO_ERROR)
        throw std::runtime_error("GL error: " + std::to_string(err));
}

/* --- Mouse events --- */

enum MouseEventType
{
    MOUSE_EVENT_PRESS,
    MOUSE_EVENT_RELEASE,
    MOUSE_EVENT_MOVE,
    MOUSE_EVENT_WHEEL_UP,
    MOUSE_EVENT_WHEEL_DOWN
};

enum MouseButton
{
    MOUSE_BUTTON_NONE   = 0,
    MOUSE_BUTTON_LEFT   = 1 << 0,
    MOUSE_BUTTON_RIGHT  = 1 << 1,
    MOUSE_BUTTON_MIDDLE = 1 << 2
};

struct MouseEvent
{
    MouseEventType type;
    MouseButton button;
    int button_mask;
    int x;
    int y;
};

GL_NAMESPACE_END

#endif /* GL_COMMON_HEADER */
