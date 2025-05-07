/********************************************************************
*   X11 Keyboard Handler with Focus Management
*   Guaranteed to work with GUI or terminal focus
********************************************************************/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <X11/Xatom.h>
#include <unistd.h>
#include "emcpose.h"
#include "scurve.h"
#include "tp_scurve.h"
#include "common.h"

// Global state
static Display *display = NULL;
static Window target_window = 0;
static int window_initialized = 0;
static int halui_initialized = 0;

// Scurve motions.
static EmcPose pausepos = {};
static struct scurve_data sc_pause_x = {};
static struct scurve_data sc_pause_y = {};
static struct scurve_data sc_pause_z = {};

void initialize_keyboard() {
    display = XOpenDisplay(NULL);
    if (!display) {
        fprintf(stderr, "Error: Cannot open X display\n");
        return;
    }

    Window root = DefaultRootWindow(display);
    Atom net_active = XInternAtom(display, "_NET_ACTIVE_WINDOW", False);
    Atom actual_type;
    int actual_format;
    unsigned long nitems, bytes_after;
    unsigned char *data = NULL;

    if (XGetWindowProperty(display, root, net_active, 0, 1, False, XA_WINDOW,
                           &actual_type, &actual_format, &nitems, &bytes_after, &data) == Success && data) {
        target_window = *((Window *)data);
        XFree(data);

        printf("Active window ID: 0x%lx\n", target_window);

        // Try to grab the keyboard to get key events
        int grab_status = XGrabKeyboard(display, target_window, True, GrabModeAsync, GrabModeAsync, CurrentTime);
        if (grab_status != GrabSuccess) {
            fprintf(stderr, "Error: Failed to grab keyboard\n");
            return;
        }

        XSelectInput(display, target_window, KeyPressMask);
        XFlush(display);
        window_initialized = 1;
    } else {
        fprintf(stderr, "Error: Could not get active window\n");
    }
}

static int jog_x(TP_STRUCT *const tp, struct path_data *path, double tarpos, int pause) {

    scurve_init(&sc_pause_x,
                path->max_jerk,
                tp->aMax,
                path->maxvel,
                tp->cycleTime);

    scurve_set_target_state(&sc_pause_x, 0, 0, tarpos, pause);
    int res = scurve_update(&sc_pause_x);
    tp->currentPos.tran.x = pausepos.tran.x+sc_pause_x.curpos;

    if(res == RETURN_FINISHED){
        return 1;
    }
    return 0;
}

static int jog_y(TP_STRUCT *const tp, struct path_data *path, double tarpos, int pause) {

    scurve_init(&sc_pause_y,
                path->max_jerk,
                tp->aMax,
                path->maxvel,
                tp->cycleTime);

    scurve_set_target_state(&sc_pause_y, 0, 0, tarpos, pause);
    int res = scurve_update(&sc_pause_y);
    tp->currentPos.tran.y = pausepos.tran.y+sc_pause_y.curpos;

    if(res == RETURN_FINISHED){
        return 1;
    }
    return 0;
}

static int jog_z(TP_STRUCT *const tp, struct path_data *path, double tarpos, int pause) {

    scurve_init(&sc_pause_z,
                path->max_jerk,
                tp->aMax,
                path->maxvel,
                tp->cycleTime);

    scurve_set_target_state(&sc_pause_z, 0, 0, tarpos, pause);
    int res = scurve_update(&sc_pause_z);
    tp->currentPos.tran.z = pausepos.tran.z+sc_pause_z.curpos;

    if(res == RETURN_FINISHED){
        return 1;
    }
    return 0;
}

static void keyboard_cleanup(struct path_data *path) {
    if (display) {
        XUngrabKeyboard(display, CurrentTime); // Release the keyboard
        XCloseDisplay(display);
        display = NULL;
    }
    window_initialized = 0;
    path->jog_x_min = 0;
    path->jog_x_plus = 0;
    path->jog_y_min = 0;
    path->jog_y_plus = 0;
    path->jog_z_min = 0;
    path->jog_z_plus = 0;
}

int tpUpdateKeyPressJog(TP_STRUCT *const tp, struct path_data *path) {

    // No pause anymore, remove keyboard capture.
    if(!tp->pausing && window_initialized) {
        keyboard_cleanup(path);
    }

    // No pause anymore. But we have to move back to start position to resume program.
    if(!tp->pausing && path->must_jog_back){
        // Run back to pausepos before resume program.
        // First run the x, y axis back into program position.
        int res_x = jog_x(tp, path, 0, 0);
        int res_y = jog_y(tp, path, 0, 0);

        if(res_x==RETURN_FINISHED && res_y==RETURN_FINISHED){
            // At last run the z axis into program position.
            int res_z = jog_z(tp, path, 0, 0);
            if(res_z==RETURN_FINISHED){
                path->must_jog_back=0; // Reset.
                return 0;
            }
        }
        return 1; // Busy.
    }

    // Initialize move in pause.
    if(tp->pausing && path->curvel == 0 && !window_initialized) {
        pausepos = tp->currentPos;
        scurve_reset_data(&sc_pause_x);
        scurve_reset_data(&sc_pause_y);
        scurve_reset_data(&sc_pause_z);
        initialize_keyboard();
        path->must_jog_back=1;
    }

    // Nothing to do, return.
    if(!window_initialized || !display || !tp->pausing) {
        return 0;
    }

    // Process key events but do not block
    while (XPending(display)) {
        XEvent ev;
        XNextEvent(display, &ev);

        if (ev.type == KeyPress || ev.type == KeyRelease) {
            KeySym keysym = XLookupKeysym(&ev.xkey, 0);
            int is_pressed = (ev.type == KeyPress); // 1 for press, 0 for release

            switch (keysym) {
            case XK_Up:
                // Action for Up arrow key
                printf("Y+ %s\n", is_pressed ? "Start" : "Stop");
                path->jog_y_plus = is_pressed;
                break;

            case XK_Down:
                // Action for Down arrow key
                printf("Y- %s\n", is_pressed ? "Start" : "Stop");
                path->jog_y_min= is_pressed;
                break;

            case XK_Left:
                // Action for Left arrow key
                printf("X- %s\n", is_pressed ? "Start" : "Stop");
                path->jog_x_min = is_pressed;
                break;

            case XK_Right:
                // Action for Right arrow key
                printf("X+ %s\n", is_pressed ? "Start" : "Stop");
                 path->jog_x_plus = is_pressed;
                break;

            case XK_Page_Up:
                // Action for Page Up key
                printf("Z+ %s\n", is_pressed ? "Start" : "Stop");
                 path->jog_z_plus = is_pressed;
                break;

            case XK_Page_Down:
                // Action for Page Down key
                printf("Z- %s\n", is_pressed ? "Start" : "Stop");
                 path->jog_z_min = is_pressed;
                break;

            default:
                // Action for other keys
                printf("0x%lx (%s)\n", keysym, XKeysymToString(keysym));
            }
            fflush(stdout);
        }
    }

    // If jogging is active, call `run_x_plus` every 1ms
    if (path->jog_x_plus) {
        jog_x(tp, path, INFINITY, 0);
    } else {
        jog_x(tp, path, INFINITY, 1);
    }
    if (path->jog_x_min) {
        jog_x(tp, path, -INFINITY, 0);
    } else {
        jog_x(tp, path, -INFINITY, 1);
    }

    if (path->jog_y_plus) {
        jog_y(tp, path, INFINITY, 0);
    } else {
        jog_y(tp, path, INFINITY, 1);
    }
    if (path->jog_y_min) {
        jog_y(tp, path, -INFINITY, 0);
    } else {
        jog_y(tp, path, -INFINITY, 1);
    }

    if (path->jog_z_plus) {
        jog_z(tp, path, INFINITY, 0);
    } else {
        jog_z(tp, path, INFINITY, 1);
    }
    if (path->jog_z_min) {
        jog_z(tp, path, -INFINITY, 0);
    } else {
        jog_z(tp, path, -INFINITY, 1);
    }

    return 1; // Busy.
}

int tpUpdateHalPinJog(TP_STRUCT *const tp, struct path_data *path) {

    // No pause anymore, remove keyboard capture.
    if(!tp->pausing && halui_initialized) {
        halui_initialized=0;
    }

    // No pause anymore. But we have to move back to start position to resume program.
    if(!tp->pausing && path->must_jog_back){
        // Run back to pausepos before resume program.
        // First run the x, y axis back into program position.
        int res_x = jog_x(tp, path, 0, 0);
        int res_y = jog_y(tp, path, 0, 0);

        if(res_x==RETURN_FINISHED && res_y==RETURN_FINISHED){
            // At last run the z axis into program position.
            int res_z = jog_z(tp, path, 0, 0);
            if(res_z==RETURN_FINISHED){
                path->must_jog_back=0; // Reset.
                return 0;
            }
        }
        return 1; // Busy.
    }

    // Initialize move in pause.
    if(tp->pausing && path->curvel == 0 && !halui_initialized) {
        pausepos = tp->currentPos;
        scurve_reset_data(&sc_pause_x);
        scurve_reset_data(&sc_pause_y);
        scurve_reset_data(&sc_pause_z);
        path->must_jog_back=1;
        halui_initialized=1;
    }

    // Nothing to do, return.
    if(!halui_initialized || !tp->pausing) {
        return 0;
    }

    // If jogging is active, call `run_x_plus` every 1ms
    if (path->jog_x_plus) {
        jog_x(tp, path, INFINITY, 0);
    } else {
        jog_x(tp, path, INFINITY, 1);
    }
    if (path->jog_x_min) {
        jog_x(tp, path, -INFINITY, 0);
    } else {
        jog_x(tp, path, -INFINITY, 1);
    }

    if (path->jog_y_plus) {
        jog_y(tp, path, INFINITY, 0);
    } else {
        jog_y(tp, path, INFINITY, 1);
    }
    if (path->jog_y_min) {
        jog_y(tp, path, -INFINITY, 0);
    } else {
        jog_y(tp, path, -INFINITY, 1);
    }

    if (path->jog_z_plus) {
        jog_z(tp, path, INFINITY, 0);
    } else {
        jog_z(tp, path, INFINITY, 1);
    }
    if (path->jog_z_min) {
        jog_z(tp, path, -INFINITY, 0);
    } else {
        jog_z(tp, path, -INFINITY, 1);
    }

    return 1; // Busy.
}

#endif // KEYBOARD_H
