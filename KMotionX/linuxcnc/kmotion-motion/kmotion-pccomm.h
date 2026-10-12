/*
 * kmotion-pccomm.h - the board's commands to the PC (KMotionCNC's PC_COMM mechanism) carried
 * out against LinuxCNC. See kmotion-pccomm.cc.
 */
#pragma once

#include <map>
#include <string>

class KmBackend;

struct KmPcCommConfig {
    bool enabled = true;                  // [KMOTION] PC_COMM
    std::string nml_file;                 // [EMC] NML_FILE, "" = LinuxCNC's default
    std::string var_file;                 // [RS274NGC] PARAMETER_FILE (GET_VARS reads it)
    bool machine_mm = false;              // [TRAJ] LINEAR_UNITS (positions in LinuxCNC's status are machine units)
    double max_feed_override = 1.2;       // [DISPLAY] MAX_FEED_OVERRIDE, MAX/MIN_SPINDLE_OVERRIDE
    double max_spindle_override = 1.0;
    double min_spindle_override = 0.0;
    std::map<int, std::string> user_button;   // [KMOTION] USER_BUTTON_<n> = an MDI line
    double mdi_timeout = 600;             // seconds an MDI line from the board may take
};

// start the dispatcher thread (board mode only); log gets what the -l log gets
void pccomm_start(KmBackend *km, const KmPcCommConfig &cfg, void (*log)(const char *fmt, ...));
void pccomm_stop();
// a message for the operator from a board command (PC_COMM_MSG), if one is waiting: the
// protocol loop reports it through motion's error ring, like the backend's own messages
bool pccomm_message(std::string &out);
