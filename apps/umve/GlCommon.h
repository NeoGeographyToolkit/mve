// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef GL_COMMON_HEADER
#define GL_COMMON_HEADER

#include <stdexcept>
#include <string>

/* OpenGL via Qt */
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLContext>

/* --- Namespace macros --- */

#define GL_NAMESPACE_BEGIN namespace gl {
#define GL_NAMESPACE_END }

// Get GL 3.3 functions from the current Qt GL context.
inline QOpenGLFunctions_3_3_Core* glFunctions()
{
    return QOpenGLContext::currentContext()
        ->versionFunctions<QOpenGLFunctions_3_3_Core>();
}

GL_NAMESPACE_BEGIN

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
