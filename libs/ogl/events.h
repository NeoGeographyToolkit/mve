/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_EVENTS_HEADER
#define OGL_EVENTS_HEADER

#include "ogl/defines.h"

OGL_NAMESPACE_BEGIN

/** Mouse event types. */
enum MouseEventType
{
    MOUSE_EVENT_PRESS,
    MOUSE_EVENT_RELEASE,
    MOUSE_EVENT_MOVE,
    MOUSE_EVENT_WHEEL_UP,
    MOUSE_EVENT_WHEEL_DOWN
};

/** Mouse button types. */
enum MouseButton
{
    MOUSE_BUTTON_NONE   = 0,
    MOUSE_BUTTON_LEFT   = 1 << 0,
    MOUSE_BUTTON_RIGHT  = 1 << 1,
    MOUSE_BUTTON_MIDDLE = 1 << 2,
    MOUSE_BUTTON_X1     = 1 << 3,
    MOUSE_BUTTON_X2     = 1 << 4
};

/** Mouse event. */
struct MouseEvent
{
    MouseEventType type; ///< Type of event
    MouseButton button; ///< Button that caused the event
    int button_mask; ///< Button state when event was generated
    int x; ///< Mouse X-position
    int y; ///< Mouse Y-position
};

OGL_NAMESPACE_END

#endif /* OGL_EVENTS_HEADER */
