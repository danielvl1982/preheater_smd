/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * motion.cpp
 */

#include "motion.h"
#include "endstops.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "../gcode/gcode.h"
#include "../lcd/marlinui.h"
#include "../inc/MarlinConfig.h"

#if IS_SCARA
  #include "../libs/buzzer.h"
  #include "../lcd/marlinui.h"
#endif

#if HAS_BED_PROBE
  #include "probe.h"
#endif

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(BLTOUCH)
  #include "../feature/bltouch.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../feature/runout.h"
#endif

#if ENABLED(SENSORLESS_HOMING)
  #include "../feature/tmc_util.h"
#endif

#if ENABLED(FWRETRACT)
  #include "../feature/fwretract.h"
#endif

#if ENABLED(BABYSTEP_DISPLAY_TOTAL)
  #include "../feature/babystep.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

// Relative Mode. Enable with G91, disable with G90.
bool relative_mode; // = false;

/**
 * Cartesian Current Position
 *   Used to track the native machine position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'sync_plan_position' to update 'planner.position'.
 */
#ifdef Z_IDLE_HEIGHT
  #define Z_INIT_POS Z_IDLE_HEIGHT
#else
  #define Z_INIT_POS Z_HOME_POS
#endif

/**
 * Cartesian Destination
 *   The destination for a move, filled in by G-code movement commands,
 *   and expected by functions like 'prepare_line_to_destination'.
 *   G-codes can set destination using 'get_destination_from_command'
 */
xyze_pos_t destination; // {0}

// G60/G61 Position Save and Return
#if SAVED_POSITIONS
  uint8_t saved_slots[(SAVED_POSITIONS + 7) >> 3];
  xyze_pos_t stored_position[SAVED_POSITIONS];
#endif

// The active extruder (tool). Set with T<extruder> command.
#if HAS_MULTI_EXTRUDER
  uint8_t active_extruder = 0; // = 0
#endif

#if ENABLED(LCD_SHOW_E_TOTAL)
  float e_move_accumulator; // = 0
#endif

// Extruder offsets
#if HAS_HOTEND_OFFSET
  xyz_pos_t hotend_offset[HOTENDS]; // Initialized by settings.load
  void reset_hotend_offsets() {
    constexpr float tmp[XYZ][HOTENDS] = { HOTEND_OFFSET_X, HOTEND_OFFSET_Y, HOTEND_OFFSET_Z };
    static_assert(
      !tmp[X_AXIS][0] && !tmp[Y_AXIS][0] && !tmp[Z_AXIS][0],
      "Offsets for the first hotend must be 0.0."
    );
    // Transpose from [XYZ][HOTENDS] to [HOTENDS][XYZ]
    HOTEND_LOOP() LOOP_ABC(a) hotend_offset[e][a] = tmp[a][e];
    TERN_(DUAL_X_CARRIAGE, hotend_offset[1].x = _MAX(X2_HOME_POS, X2_MAX_POS));
  }
#endif

// The feedrate for the current move, often used as the default if
// no other feedrate is specified. Overridden for special moves.
// Set by the last G0 through G5 command's "F" parameter.
// Functions that override this for custom moves *must always* restore it!
#ifndef DEFAULT_FEEDRATE_MM_M
  #define DEFAULT_FEEDRATE_MM_M 4000
#endif
feedRate_t feedrate_mm_s = MMM_TO_MMS(DEFAULT_FEEDRATE_MM_M);
int16_t feedrate_percentage = 100;

// Cartesian conversion result goes here:
xyz_pos_t cartes;

#if IS_KINEMATIC

  abce_pos_t delta;

  #if HAS_SCARA_OFFSET
    abc_pos_t scara_home_offset;
  #endif

  #if HAS_SOFTWARE_ENDSTOPS
    float delta_max_radius, delta_max_radius_2;
  #elif IS_SCARA
    constexpr float delta_max_radius = SCARA_PRINTABLE_RADIUS,
                    delta_max_radius_2 = sq(SCARA_PRINTABLE_RADIUS);
  #else // DELTA
    constexpr float delta_max_radius = DELTA_PRINTABLE_RADIUS,
                    delta_max_radius_2 = sq(DELTA_PRINTABLE_RADIUS);
  #endif

#endif

/**
 * The workspace can be offset by some commands, or
 * these offsets may be omitted to save on computation.
 */
#if HAS_POSITION_SHIFT
  // The distance that XYZ has been offset by G92. Reset by G28.
  xyz_pos_t position_shift{0};
#endif
#if HAS_HOME_OFFSET
  // This offset is added to the configured home position.
  // Set by M206, M428, or menu item. Saved to EEPROM.
  xyz_pos_t home_offset{0};
#endif
#if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
  // The above two are combined to save on computes
  xyz_pos_t workspace_offset{0};
#endif

#if HAS_ABL_NOT_UBL
  feedRate_t xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_FEEDRATE);
#endif

/**
 * Output the current position to serial
 */

inline void report_more_positions() {
  stepper.report_positions();
  TERN_(IS_SCARA, scara_report_positions());
}

// Report the logical position for a given machine position
inline void report_logical_position(const xyze_pos_t &rpos) {
  const xyze_pos_t lpos = rpos.asLogical();
  #if NUM_AXES
    SERIAL_ECHOPGM_P(
      LIST_N(DOUBLE(NUM_AXES),
           X_LBL, lpos.x,
        SP_Y_LBL, lpos.y,
        SP_Z_LBL, lpos.z,
        SP_I_LBL, lpos.i,
        SP_J_LBL, lpos.j,
        SP_K_LBL, lpos.k,
        SP_U_LBL, lpos.u,
        SP_V_LBL, lpos.v,
        SP_W_LBL, lpos.w
      )
    );
  #endif
  #if HAS_EXTRUDERS
    SERIAL_ECHOPGM_P(SP_E_LBL, lpos.e);
  #endif
}

/**
 * Report the logical current position according to the most recent G-code command.
 * The planner.position always corresponds to the last G-code too. This makes M114
 * suitable for debugging kinematics and leveling while avoiding planner sync that
 * definitively interrupts the printing flow.
 */
#if ENABLED(AUTO_REPORT_POSITION)
  AutoReporter<PositionReport> position_auto_reporter;
#endif

#if ANY(FULL_REPORT_TO_HOST_FEATURE, REALTIME_REPORTING_COMMANDS)

  M_StateEnum M_State_grbl = M_INIT;

  /**
   * Output the current grbl compatible state to serial while moving
   */
  void report_current_grblstate_moving() { SERIAL_ECHOLNPGM("S_XYZ:", int(M_State_grbl)); }

  /**
   * Output the current position (processed) to serial while moving
   */
  void report_current_position_moving() {
    get_cartesian_from_steppers();
    const xyz_pos_t lpos = cartes.asLogical();

    SERIAL_ECHOPGM_P(
      LIST_N(DOUBLE(NUM_AXES),
           X_LBL, lpos.x,
        SP_Y_LBL, lpos.y,
        SP_Z_LBL, lpos.z,
        SP_I_LBL, lpos.i,
        SP_J_LBL, lpos.j,
        SP_K_LBL, lpos.k,
        SP_U_LBL, lpos.u,
        SP_V_LBL, lpos.v,
        SP_W_LBL, lpos.w
      )
      #if HAS_EXTRUDERS
        , SP_E_LBL, current_position.e
      #endif
    );

    stepper.report_positions();
    TERN_(IS_SCARA, scara_report_positions());
    report_current_grblstate_moving();
  }

  /**
   * Set a Grbl-compatible state from the current marlin_state
   */
  M_StateEnum grbl_state_for_marlin_state() {
    switch (marlin_state) {
      case MF_INITIALIZING: return M_INIT;
      case MF_SD_COMPLETE:  return M_ALARM;
      case MF_WAITING:      return M_IDLE;
      case MF_STOPPED:      return M_END;
      case MF_RUNNING:      return M_RUNNING;
      case MF_PAUSED:       return M_HOLD;
      case MF_KILLED:       return M_ERROR;
      default:              return M_IDLE;
    }
  }

#endif

#if IS_KINEMATIC

  bool position_is_reachable(const_float_t rx, const_float_t ry, const float inset/*=0*/) {

    bool can_reach;

    #if ENABLED(DELTA)

      can_reach = HYPOT2(rx, ry) <= sq(DELTA_PRINTABLE_RADIUS - inset + fslop);

    #elif ENABLED(AXEL_TPARA)

      const float R2 = HYPOT2(rx - TPARA_OFFSET_X, ry - TPARA_OFFSET_Y);
      can_reach = (
        R2 <= sq(L1 + L2) - inset
        #if MIDDLE_DEAD_ZONE_R > 0
          && R2 >= sq(float(MIDDLE_DEAD_ZONE_R))
        #endif
      );

    #elif IS_SCARA

      const float R2 = HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y);
      can_reach = (
        R2 <= sq(L1 + L2) - inset
        #if MIDDLE_DEAD_ZONE_R > 0
          && R2 >= sq(float(MIDDLE_DEAD_ZONE_R))
        #endif
      );

    #elif ENABLED(POLARGRAPH)

      const float d1 = rx - (draw_area_min.x),
                  d2 = (draw_area_max.x) - rx,
                   y = ry - (draw_area_max.y),
                   a = HYPOT(d1, y),
                   b = HYPOT(d2, y);

      can_reach = (
           a < polargraph_max_belt_len + 1
        && b < polargraph_max_belt_len + 1
      );

    #endif

    return can_reach;
  }

#else // CARTESIAN

  // Return true if the given position is within the machine bounds.
  bool position_is_reachable(TERN_(HAS_X_AXIS, const_float_t rx) OPTARG(HAS_Y_AXIS, const_float_t ry)) {
    if (TERN0(HAS_Y_AXIS, !COORDINATE_OKAY(ry, Y_MIN_POS - fslop, Y_MAX_POS + fslop))) return false;
    #if ENABLED(DUAL_X_CARRIAGE)
      if (active_extruder)
        return COORDINATE_OKAY(rx, X2_MIN_POS - fslop, X2_MAX_POS + fslop);
      else
        return COORDINATE_OKAY(rx, X1_MIN_POS - fslop, X1_MAX_POS + fslop);
    #else
      if (TERN0(HAS_X_AXIS, !COORDINATE_OKAY(rx, X_MIN_POS - fslop, X_MAX_POS + fslop))) return false;
      return true;
    #endif
  }

#endif // CARTESIAN

void home_if_needed(const bool keeplev/*=false*/) {
  if (!all_axes_trusted()) gcode.home_all_axes(keeplev);
}

/**
 * Run out the planner buffer and re-sync the current
 * position from the last-updated stepper positions.
 */
void quickstop_stepper() {
  planner.quick_stop();
  planner.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES_ENUM);
  sync_plan_position();
}

#if ENABLED(REALTIME_REPORTING_COMMANDS)

  void quickpause_stepper() {
    planner.quick_pause();
    //planner.synchronize();
  }

  void quickresume_stepper() {
    planner.quick_resume();
    //planner.synchronize();
  }

#endif

/**
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position() {
  if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position", current_position);
  planner.set_position_mm(current_position);
}

#if HAS_EXTRUDERS
  void sync_plan_position_e() { planner.set_e_position_mm(current_position.e); }
#endif

/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
  #if ENABLED(DELTA)
    forward_kinematics(planner.get_axis_positions_mm());
  #elif IS_SCARA
    forward_kinematics(
      planner.get_axis_position_degrees(A_AXIS), planner.get_axis_position_degrees(B_AXIS)
      OPTARG(AXEL_TPARA, planner.get_axis_position_degrees(C_AXIS))
    );
    cartes.z = planner.get_axis_position_mm(Z_AXIS);
  #else
    NUM_AXIS_CODE(
      cartes.x = planner.get_axis_position_mm(X_AXIS),
      cartes.y = planner.get_axis_position_mm(Y_AXIS),
      cartes.z = planner.get_axis_position_mm(Z_AXIS),
      cartes.i = planner.get_axis_position_mm(I_AXIS),
      cartes.j = planner.get_axis_position_mm(J_AXIS),
      cartes.k = planner.get_axis_position_mm(K_AXIS),
      cartes.u = planner.get_axis_position_mm(U_AXIS),
      cartes.v = planner.get_axis_position_mm(V_AXIS),
      cartes.w = planner.get_axis_position_mm(W_AXIS)
    );
  #endif
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * sync_plan_position after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  xyze_pos_t pos = cartes;

  TERN_(HAS_EXTRUDERS, pos.e = planner.get_axis_position_mm(E_AXIS));

  TERN_(HAS_POSITION_MODIFIERS, planner.unapply_modifiers(pos, true));

  if (axis == ALL_AXES_ENUM)
    current_position = pos;
  else
    current_position[axis] = pos[axis];
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void line_to_current_position(const_feedRate_t fr_mm_s/*=feedrate_mm_s*/) {
  planner.buffer_line(current_position, fr_mm_s);
}

#if HAS_EXTRUDERS
  void unscaled_e_move(const_float_t length, const_feedRate_t fr_mm_s) {
    TERN_(HAS_FILAMENT_SENSOR, runout.reset());
    current_position.e += length / planner.e_factor[active_extruder];
    line_to_current_position(fr_mm_s);
    planner.synchronize();
  }
#endif

#if IS_KINEMATIC

  /**
   * Buffer a fast move without interpolation. Set current_position to destination
   */
  void prepare_fast_move_to_destination(const_feedRate_t scaled_fr_mm_s/*=MMS_SCALED(feedrate_mm_s)*/) {
    if (DEBUGGING(LEVELING)) DEBUG_POS("prepare_fast_move_to_destination", destination);

    #if UBL_SEGMENTED
      // UBL segmented line will do Z-only moves in single segment
      bedlevel.line_to_destination_segmented(scaled_fr_mm_s);
    #else
      if (current_position == destination) return;

      planner.buffer_line(destination, scaled_fr_mm_s);
    #endif

    current_position = destination;
  }

#endif // IS_KINEMATIC

/**
 * Do a fast or normal move to 'destination' with an optional FR.
 *  - Move at normal speed regardless of feedrate percentage.
 *  - Extrude the specified length regardless of flow percentage.
 */
void _internal_move_to_destination(const_feedRate_t fr_mm_s/*=0.0f*/
  OPTARG(IS_KINEMATIC, const bool is_fast/*=false*/)
) {
  REMEMBER(fr, feedrate_mm_s);
  REMEMBER(pct, feedrate_percentage, 100);
  TERN_(HAS_EXTRUDERS, REMEMBER(fac, planner.e_factor[active_extruder], 1.0f));

  if (fr_mm_s) feedrate_mm_s = fr_mm_s;
  if (TERN0(IS_KINEMATIC, is_fast))
    TERN(IS_KINEMATIC, prepare_fast_move_to_destination(), NOOP);
  else
    prepare_line_to_destination();
}

#if SECONDARY_AXES

  void secondary_axis_moves(SECONDARY_AXIS_ARGS_LC(const_float_t), const_feedRate_t fr_mm_s) {
    auto move_one = [&](const AxisEnum a, const_float_t p) {
      const feedRate_t fr = fr_mm_s ?: homing_feedrate(a);
      current_position[a] = p; line_to_current_position(fr);
    };
    SECONDARY_AXIS_CODE(
      move_one(I_AXIS, i), move_one(J_AXIS, j), move_one(K_AXIS, k),
      move_one(U_AXIS, u), move_one(V_AXIS, v), move_one(W_AXIS, w)
    );
  }

#endif

/**
 * Plan a move to (X, Y, Z, [I, [J, [K...]]]) and set the current_position
 * Plan a move to (X, Y, Z, [I, [J, [K...]]]) with separation of Z from other components.
 *
 * - If Z is moving up, the Z move is done before XY, etc.
 * - If Z is moving down, the Z move is done after XY, etc.
 * - Delta may lower Z first to get into the free motion zone.
 * - Before returning, wait for the planner buffer to empty.
 */
void do_blocking_move_to(NUM_AXIS_ARGS_(const_float_t) const_feedRate_t fr_mm_s/*=0.0f*/) {
  DEBUG_SECTION(log_move, "do_blocking_move_to", DEBUGGING(LEVELING));
  #if NUM_AXES
    if (DEBUGGING(LEVELING)) DEBUG_XYZ("> ", NUM_AXIS_ARGS_LC());
  #endif

  const feedRate_t xy_feedrate = fr_mm_s ?: feedRate_t(XY_PROBE_FEEDRATE_MM_S);

  #if HAS_Z_AXIS
    const feedRate_t z_feedrate = fr_mm_s ?: homing_feedrate(Z_AXIS);
  #endif

  #if IS_KINEMATIC && DISABLED(POLARGRAPH)
    // kinematic machines are expected to home to a point 1.5x their range? never reachable.
    if (!position_is_reachable(x, y)) return;
    destination = current_position;          // sync destination at the start
  #endif

  #if ENABLED(DELTA)

    REMEMBER(fr, feedrate_mm_s, xy_feedrate);

    if (DEBUGGING(LEVELING)) DEBUG_POS("destination = current_position", destination);

    // when in the danger zone
    if (current_position.z > delta_clip_start_height) {
      if (z > delta_clip_start_height) {                      // staying in the danger zone
        destination.set(x, y, z);                             // move directly (uninterpolated)
        prepare_internal_fast_move_to_destination();          // set current_position from destination
        if (DEBUGGING(LEVELING)) DEBUG_POS("danger zone move", current_position);
        return;
      }
      destination.z = delta_clip_start_height;
      prepare_internal_fast_move_to_destination();            // set current_position from destination
      if (DEBUGGING(LEVELING)) DEBUG_POS("zone border move", current_position);
    }

    if (z > current_position.z) {                             // raising?
      destination.z = z;
      prepare_internal_fast_move_to_destination(z_feedrate);  // set current_position from destination
      if (DEBUGGING(LEVELING)) DEBUG_POS("z raise move", current_position);
    }

    destination.set(x, y);
    prepare_internal_move_to_destination();                   // set current_position from destination
    if (DEBUGGING(LEVELING)) DEBUG_POS("xy move", current_position);

    if (z < current_position.z) {                             // lowering?
      destination.z = z;
      prepare_internal_fast_move_to_destination(z_feedrate);  // set current_position from destination
      if (DEBUGGING(LEVELING)) DEBUG_POS("z lower move", current_position);
    }

    #if SECONDARY_AXES
      secondary_axis_moves(SECONDARY_AXIS_LIST(i, j, k, u, v, w), fr_mm_s);
    #endif

  #elif IS_SCARA

    // If Z needs to raise, do it before moving XY
    if (destination.z < z) { destination.z = z; prepare_internal_fast_move_to_destination(z_feedrate); }

    destination.set(x, y); prepare_internal_fast_move_to_destination(xy_feedrate);

    #if SECONDARY_AXES
      secondary_axis_moves(SECONDARY_AXIS_LIST(i, j, k, u, v, w), fr_mm_s);
    #endif

    // If Z needs to lower, do it after moving XY
    if (destination.z > z) { destination.z = z; prepare_internal_fast_move_to_destination(z_feedrate); }

  #else

    #if HAS_Z_AXIS  // If Z needs to raise, do it before moving XY
      if (current_position.z < z) { current_position.z = z; line_to_current_position(z_feedrate); }
    #endif

    current_position.set(TERN_(HAS_X_AXIS, x) OPTARG(HAS_Y_AXIS, y)); line_to_current_position(xy_feedrate);

    #if SECONDARY_AXES
      secondary_axis_moves(SECONDARY_AXIS_LIST(i, j, k, u, v, w), fr_mm_s);
    #endif

    #if HAS_Z_AXIS
      // If Z needs to lower, do it after moving XY
      if (current_position.z > z) { current_position.z = z; line_to_current_position(z_feedrate); }
    #endif

  #endif

  planner.synchronize();
}

void do_blocking_move_to(const xy_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(NUM_AXIS_LIST_(raw.x, raw.y, current_position.z, current_position.i, current_position.j, current_position.k,
                                    current_position.u, current_position.v, current_position.w) fr_mm_s);
}
void do_blocking_move_to(const xyz_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(NUM_AXIS_ELEM_(raw) fr_mm_s);
}
void do_blocking_move_to(const xyze_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(NUM_AXIS_ELEM_(raw) fr_mm_s);
}

#if HAS_X_AXIS
  void do_blocking_move_to_x(const_float_t rx, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_x(", rx, ", ", fr_mm_s, ")");
    do_blocking_move_to(
      NUM_AXIS_LIST_(rx, current_position.y, current_position.z, current_position.i, current_position.j, current_position.k,
                     current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
#endif

#if HAS_Y_AXIS
  void do_blocking_move_to_y(const_float_t ry, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_y(", ry, ", ", fr_mm_s, ")");
    do_blocking_move_to(
      NUM_AXIS_LIST_(current_position.x, ry, current_position.z, current_position.i, current_position.j, current_position.k,
                    current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
#endif

#if HAS_Z_AXIS
  void do_blocking_move_to_z(const_float_t rz, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_z(", rz, ", ", fr_mm_s, ")");
    do_blocking_move_to_xy_z(current_position, rz, fr_mm_s);
  }
#endif

#if HAS_I_AXIS
  void do_blocking_move_to_i(const_float_t ri, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyz_i(current_position, ri, fr_mm_s);
  }
  void do_blocking_move_to_xyz_i(const xyze_pos_t &raw, const_float_t i, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, i, raw.j, raw.k, raw.u, raw.v, raw.w)
      fr_mm_s
    );
  }
#endif

#if HAS_J_AXIS
  void do_blocking_move_to_j(const_float_t rj, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzi_j(current_position, rj, fr_mm_s);
  }
  void do_blocking_move_to_xyzi_j(const xyze_pos_t &raw, const_float_t j, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, j, raw.k, raw.u, raw.v, raw.w)
      fr_mm_s
    );
  }
#endif

#if HAS_K_AXIS
  void do_blocking_move_to_k(const_float_t rk, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzij_k(current_position, rk, fr_mm_s);
  }
  void do_blocking_move_to_xyzij_k(const xyze_pos_t &raw, const_float_t k, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, k, raw.u, raw.v, raw.w)
      fr_mm_s
    );
  }
#endif

#if HAS_U_AXIS
  void do_blocking_move_to_u(const_float_t ru, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzijk_u(current_position, ru, fr_mm_s);
  }
  void do_blocking_move_to_xyzijk_u(const xyze_pos_t &raw, const_float_t u, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, raw.k, u, raw.v, raw.w)
      fr_mm_s
    );
  }
#endif

#if HAS_V_AXIS
  void do_blocking_move_to_v(const_float_t rv, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzijku_v(current_position, rv, fr_mm_s);
  }
  void do_blocking_move_to_xyzijku_v(const xyze_pos_t &raw, const_float_t v, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, raw.k, raw.u, v, raw.w)
      fr_mm_s
    );
  }
#endif

#if HAS_W_AXIS
  void do_blocking_move_to_w(const_float_t rw, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzijkuv_w(current_position, rw, fr_mm_s);
  }
  void do_blocking_move_to_xyzijkuv_w(const xyze_pos_t &raw, const_float_t w, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, raw.k, raw.u, raw.v, w)
      fr_mm_s
    );
  }
#endif

#if HAS_Y_AXIS
  void do_blocking_move_to_xy(const_float_t rx, const_float_t ry, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_xy(", rx, ", ", ry, ", ", fr_mm_s, ")");
    do_blocking_move_to(
      NUM_AXIS_LIST_(rx, ry, current_position.z, current_position.i, current_position.j, current_position.k,
                    current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_xy(const xy_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to_xy(raw.x, raw.y, fr_mm_s);
  }
#endif

#if HAS_Z_AXIS
  void do_blocking_move_to_xy_z(const xy_pos_t &raw, const_float_t z, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, z, current_position.i, current_position.j, current_position.k,
                    current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
  void do_z_clearance(const_float_t zclear, const bool lower_allowed/*=false*/) {
    float zdest = zclear;
    if (!lower_allowed) NOLESS(zdest, current_position.z);
    do_blocking_move_to_z(_MIN(zdest, Z_MAX_POS), TERN(HAS_BED_PROBE, z_probe_fast_mm_s, homing_feedrate(Z_AXIS)));
  }
  void do_z_clearance_by(const_float_t zclear) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_z_clearance_by(", zclear, ")");
    do_z_clearance(current_position.z + zclear);
  }
#endif

//
// Prepare to do endstop or probe moves with custom feedrates.
//  - Save / restore current feedrate and multiplier
//
static float saved_feedrate_mm_s;
static int16_t saved_feedrate_percentage;
void remember_feedrate_and_scaling() {
  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
}
void remember_feedrate_scaling_off() {
  remember_feedrate_and_scaling();
  feedrate_percentage = 100;
}
void restore_feedrate_and_scaling() {
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
}

#if HAS_SOFTWARE_ENDSTOPS

  // Software Endstops are based on the configured limits.
  #define _AMIN(A) A##_MIN_POS
  #define _AMAX(A) A##_MAX_POS
  soft_endstops_t soft_endstop = {
    true, false,
    { MAPLIST(_AMIN, MAIN_AXIS_NAMES) },
    { MAPLIST(_AMAX, MAIN_AXIS_NAMES) },
  };

  /**
   * Software endstops can be used to monitor the open end of
   * an axis that has a hardware endstop on the other end. Or
   * they can prevent axes from moving past endstops and grinding.
   *
   * To keep doing their job as the coordinate system changes,
   * the software endstop positions must be refreshed to remain
   * at the same positions relative to the machine.
   */
  void update_software_endstops(const AxisEnum axis
    OPTARG(HAS_HOTEND_OFFSET, const uint8_t old_tool_index/*=0*/, const uint8_t new_tool_index/*=0*/)
  ) {

    #if ENABLED(DUAL_X_CARRIAGE)

      if (axis == X_AXIS) {

        // In Dual X mode hotend_offset[X] is T1's home position
        const float dual_max_x = _MAX(hotend_offset[1].x, X2_MAX_POS);

        if (new_tool_index != 0) {
          // T1 can move from X2_MIN_POS to X2_MAX_POS or X2 home position (whichever is larger)
          soft_endstop.min.x = X2_MIN_POS;
          soft_endstop.max.x = dual_max_x;
        }
        else if (idex_is_duplicating()) {
          // In Duplication Mode, T0 can move as far left as X1_MIN_POS
          // but not so far to the right that T1 would move past the end
          soft_endstop.min.x = X1_MIN_POS;
          soft_endstop.max.x = _MIN(X1_MAX_POS, dual_max_x - duplicate_extruder_x_offset);
        }
        else {
          // In other modes, T0 can move from X1_MIN_POS to X1_MAX_POS
          soft_endstop.min.x = X1_MIN_POS;
          soft_endstop.max.x = X1_MAX_POS;
        }

      }

    #elif ENABLED(DELTA)

      soft_endstop.min[axis] = base_min_pos(axis);
      soft_endstop.max[axis] = (axis == Z_AXIS) ? DIFF_TERN(HAS_BED_PROBE, delta_height, probe.offset.z) : base_max_pos(axis);

      switch (axis) {
        case X_AXIS:
        case Y_AXIS:
          // Get a minimum radius for clamping
          delta_max_radius = _MIN(ABS(_MAX(soft_endstop.min.x, soft_endstop.min.y)), soft_endstop.max.x, soft_endstop.max.y);
          delta_max_radius_2 = sq(delta_max_radius);
          break;
        case Z_AXIS:
          refresh_delta_clip_start_height();
        default: break;
      }

    #elif HAS_HOTEND_OFFSET

      // Software endstops are relative to the tool 0 workspace, so
      // the movement limits must be shifted by the tool offset to
      // retain the same physical limit when other tools are selected.

      if (new_tool_index == old_tool_index || axis == Z_AXIS) { // The Z axis is "special" and shouldn't be modified
        const float offs = (axis == Z_AXIS) ? 0 : hotend_offset[active_extruder][axis];
        soft_endstop.min[axis] = base_min_pos(axis) + offs;
        soft_endstop.max[axis] = base_max_pos(axis) + offs;
      }
      else {
        const float diff = hotend_offset[new_tool_index][axis] - hotend_offset[old_tool_index][axis];
        soft_endstop.min[axis] += diff;
        soft_endstop.max[axis] += diff;
      }

    #else

      soft_endstop.min[axis] = base_min_pos(axis);
      soft_endstop.max[axis] = base_max_pos(axis);

    #endif

    if (DEBUGGING(LEVELING))
      SERIAL_ECHOLNPGM("Axis ", C(AXIS_CHAR(axis)), " min:", soft_endstop.min[axis], " max:", soft_endstop.max[axis]);
  }

  /**
   * Constrain the given coordinates to the software endstops.
   *
   * For DELTA/SCARA the XY constraint is based on the smallest
   * radius within the set software endstops.
   */
  void apply_motion_limits(xyz_pos_t &target) {

    if (!soft_endstop._enabled) return;

    #if IS_KINEMATIC

      if (TERN0(DELTA, !all_axes_homed())) return;

      #if ALL(HAS_HOTEND_OFFSET, DELTA)
        // The effector center position will be the target minus the hotend offset.
        const xy_pos_t offs = hotend_offset[active_extruder];
      #else
        // SCARA needs to consider the angle of the arm through the entire move, so for now use no tool offset.
        constexpr xy_pos_t offs{0};
      #endif

      #if ENABLED(POLARGRAPH)
        LIMIT(target.x, draw_area_min.x, draw_area_max.x);
        LIMIT(target.y, draw_area_min.y, draw_area_max.y);
      #else
        if (TERN1(IS_SCARA, axis_was_homed(X_AXIS) && axis_was_homed(Y_AXIS))) {
          const float dist_2 = HYPOT2(target.x - offs.x, target.y - offs.y);
          if (dist_2 > delta_max_radius_2)
            target *= float(delta_max_radius / SQRT(dist_2)); // 200 / 300 = 0.66
        }
      #endif

    #else

      #if HAS_X_AXIS
        if (axis_was_homed(X_AXIS)) {
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_X)
            NOLESS(target.x, soft_endstop.min.x);
          #endif
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_X)
            NOMORE(target.x, soft_endstop.max.x);
          #endif
        }
      #endif

      #if HAS_Y_AXIS
        if (axis_was_homed(Y_AXIS)) {
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_Y)
            NOLESS(target.y, soft_endstop.min.y);
          #endif
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_Y)
            NOMORE(target.y, soft_endstop.max.y);
          #endif
        }
      #endif

    #endif

    #if HAS_Z_AXIS
      if (axis_was_homed(Z_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_Z)
          NOLESS(target.z, soft_endstop.min.z);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_Z)
          NOMORE(target.z, soft_endstop.max.z);
        #endif
      }
    #endif
    #if HAS_I_AXIS
      if (axis_was_homed(I_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_I)
          NOLESS(target.i, soft_endstop.min.i);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_I)
          NOMORE(target.i, soft_endstop.max.i);
        #endif
      }
    #endif
    #if HAS_J_AXIS
      if (axis_was_homed(J_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_J)
          NOLESS(target.j, soft_endstop.min.j);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_J)
          NOMORE(target.j, soft_endstop.max.j);
        #endif
      }
    #endif
    #if HAS_K_AXIS
      if (axis_was_homed(K_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_K)
          NOLESS(target.k, soft_endstop.min.k);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_K)
          NOMORE(target.k, soft_endstop.max.k);
        #endif
      }
    #endif
    #if HAS_U_AXIS
      if (axis_was_homed(U_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_U)
          NOLESS(target.u, soft_endstop.min.u);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_U)
          NOMORE(target.u, soft_endstop.max.u);
        #endif
      }
    #endif
    #if HAS_V_AXIS
      if (axis_was_homed(V_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_V)
          NOLESS(target.v, soft_endstop.min.v);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_V)
          NOMORE(target.v, soft_endstop.max.v);
        #endif
      }
    #endif
    #if HAS_W_AXIS
      if (axis_was_homed(W_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_W)
          NOLESS(target.w, soft_endstop.min.w);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_W)
          NOMORE(target.w, soft_endstop.max.w);
        #endif
      }
    #endif
  }

#else // !HAS_SOFTWARE_ENDSTOPS

  soft_endstops_t soft_endstop;

#endif // !HAS_SOFTWARE_ENDSTOPS

FORCE_INLINE void segment_idle(millis_t &next_idle_ms) {
  const millis_t ms = millis();
  if (ELAPSED(ms, next_idle_ms)) {
    next_idle_ms = ms + 200UL;
    return idle();
  }
  thermalManager.task();  // Returns immediately on most calls
}

#if IS_KINEMATIC

  #if IS_SCARA
    /**
     * Before raising this value, use M665 S[seg_per_sec] to decrease
     * the number of segments-per-second. Default is 200. Some deltas
     * do better with 160 or lower. It would be good to know how many
     * segments-per-second are actually possible for SCARA on AVR.
     *
     * Longer segments result in less kinematic overhead
     * but may produce jagged lines. Try 0.5mm, 1.0mm, and 2.0mm
     * and compare the difference.
     */
    #define SCARA_MIN_SEGMENT_LENGTH 0.5f
  #endif

  /**
   * Prepare a linear move in a DELTA or SCARA setup.
   *
   * Called from prepare_line_to_destination as the
   * default Delta/SCARA segmenter.
   *
   * This calls planner.buffer_line several times, adding
   * small incremental moves for DELTA or SCARA.
   *
   * For Unified Bed Leveling (Delta or Segmented Cartesian)
   * the bedlevel.line_to_destination_segmented method replaces this.
   *
   * For Auto Bed Leveling (Bilinear) with SEGMENT_LEVELED_MOVES
   * this is replaced by segmented_line_to_destination below.
   */
  inline bool line_to_destination_kinematic() {

    // Get the top feedrate of the move in the XY plane
    const float scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);

    const xyze_float_t diff = destination - current_position;

    // If the move is only in Z/E don't split up the move
    if (!diff.x && !diff.y) {
      planner.buffer_line(destination, scaled_fr_mm_s);
      return false; // caller will update current_position
    }

    // Fail if attempting move outside printable radius
    if (!position_is_reachable(destination)) return true;

    // Get the linear distance in XYZ
    float cartesian_mm = xyz_float_t(diff).magnitude();

    // If the move is very short, check the E move distance
    TERN_(HAS_EXTRUDERS, if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(diff.e));

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return true;

    // Minimum number of seconds to move the given distance
    const float seconds = cartesian_mm / scaled_fr_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments
    uint16_t segments = segments_per_second * seconds;

    // For SCARA enforce a minimum segment size
    #if IS_SCARA
      NOMORE(segments, cartesian_mm * RECIPROCAL(SCARA_MIN_SEGMENT_LENGTH));
    #endif

    // At least one segment is required
    NOLESS(segments, 1U);

    // The approximate length of each segment
    const float inv_segments = 1.0f / float(segments);
    const xyze_float_t segment_distance = diff * inv_segments;

    // Add hints to help optimize the move
    PlannerHints hints(cartesian_mm * inv_segments);
    TERN_(SCARA_FEEDRATE_SCALING, hints.inv_duration = scaled_fr_mm_s / hints.millimeters);

    /*
    SERIAL_ECHOPGM("mm=", cartesian_mm);
    SERIAL_ECHOPGM(" seconds=", seconds);
    SERIAL_ECHOPGM(" segments=", segments);
    SERIAL_ECHOPGM(" segment_mm=", hints.millimeters);
    SERIAL_EOL();
    //*/

    // Get the current position as starting point
    xyze_pos_t raw = current_position;

    // Calculate and execute the segments
    millis_t next_idle_ms = millis() + 200UL;
    while (--segments) {
      segment_idle(next_idle_ms);
      raw += segment_distance;
      if (!planner.buffer_line(raw, scaled_fr_mm_s, active_extruder, hints))
        break;
    }

    // Ensure last segment arrives at target location.
    planner.buffer_line(destination, scaled_fr_mm_s, active_extruder, hints);

    return false; // caller will update current_position
  }

#else // !IS_KINEMATIC

  #if ENABLED(SEGMENT_LEVELED_MOVES) && DISABLED(AUTO_BED_LEVELING_UBL)

    /**
     * Prepare a segmented move on a CARTESIAN setup.
     *
     * This calls planner.buffer_line several times, adding
     * small incremental moves. This allows the planner to
     * apply more detailed bed leveling to the full move.
     */
    inline void segmented_line_to_destination(const_feedRate_t fr_mm_s, const float segment_size=LEVELED_SEGMENT_LENGTH) {

      const xyze_float_t diff = destination - current_position;

      // If the move is only in Z/E don't split up the move
      if (!diff.x && !diff.y) {
        planner.buffer_line(destination, fr_mm_s);
        return;
      }

      // Get the linear distance in XYZ
      // If the move is very short, check the E move distance
      // No E move either? Game over.
      float cartesian_mm = diff.magnitude();
      TERN_(HAS_EXTRUDERS, if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(diff.e));
      if (UNEAR_ZERO(cartesian_mm)) return;

      // The length divided by the segment size
      // At least one segment is required
      uint16_t segments = cartesian_mm / segment_size;
      NOLESS(segments, 1U);

      // The approximate length of each segment
      const float inv_segments = 1.0f / float(segments);
      const xyze_float_t segment_distance = diff * inv_segments;

      // Add hints to help optimize the move
      PlannerHints hints(cartesian_mm * inv_segments);
      TERN_(SCARA_FEEDRATE_SCALING, hints.inv_duration = scaled_fr_mm_s / hints.millimeters);

      //SERIAL_ECHOPGM("mm=", cartesian_mm);
      //SERIAL_ECHOLNPGM(" segments=", segments);
      //SERIAL_ECHOLNPGM(" segment_mm=", hints.millimeters);

      // Get the raw current position as starting point
      xyze_pos_t raw = current_position;

      // Calculate and execute the segments
      millis_t next_idle_ms = millis() + 200UL;
      while (--segments) {
        segment_idle(next_idle_ms);
        raw += segment_distance;
        if (!planner.buffer_line(raw, fr_mm_s, active_extruder, hints))
          break;
      }

      // Since segment_distance is only approximate,
      // the final move must be to the exact destination.
      planner.buffer_line(destination, fr_mm_s, active_extruder, hints);
    }

  #endif // SEGMENT_LEVELED_MOVES && !AUTO_BED_LEVELING_UBL

  /**
   * Prepare a linear move in a Cartesian setup.
   *
   * When a mesh-based leveling system is active, moves are segmented
   * according to the configuration of the leveling system.
   *
   * Return true if 'current_position' was set to 'destination'
   */
  inline bool line_to_destination_cartesian() {
    const float scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);
    #if HAS_MESH
      if (planner.leveling_active && planner.leveling_active_at_z(destination.z)) {
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          #if UBL_SEGMENTED
            return bedlevel.line_to_destination_segmented(scaled_fr_mm_s);
          #else
            bedlevel.line_to_destination_cartesian(scaled_fr_mm_s, active_extruder); // UBL's motion routine needs to know about
            return true;                                                             // all moves, including Z-only moves.
          #endif
        #elif ENABLED(SEGMENT_LEVELED_MOVES)
          segmented_line_to_destination(scaled_fr_mm_s);
          return false; // caller will update current_position
        #else
          /**
           * For MBL and ABL-BILINEAR only segment moves when X or Y are involved.
           * Otherwise fall through to do a direct single move.
           */
          if (xy_pos_t(current_position) != xy_pos_t(destination)) {
            #if ENABLED(MESH_BED_LEVELING)
              bedlevel.line_to_destination(scaled_fr_mm_s);
            #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
              bedlevel.line_to_destination(scaled_fr_mm_s);
            #endif
            return true;
          }
        #endif
      }
    #endif // HAS_MESH

    planner.buffer_line(destination, scaled_fr_mm_s);
    return false; // caller will update current_position
  }

#endif // !IS_KINEMATIC

#if HAS_DUPLICATION_MODE
  bool extruder_duplication_enabled;
  #if ENABLED(MULTI_NOZZLE_DUPLICATION)
    uint8_t duplication_e_mask; // = 0
  #endif
#endif

#if ENABLED(DUAL_X_CARRIAGE)

  DualXMode dual_x_carriage_mode         = DEFAULT_DUAL_X_CARRIAGE_MODE;
  float inactive_extruder_x              = X2_MAX_POS,                    // Used in mode 0 & 1
        duplicate_extruder_x_offset      = DEFAULT_DUPLICATION_X_OFFSET;  // Used in mode 2 & 3
  xyz_pos_t raised_parked_position;                                       // Used in mode 1
  bool active_extruder_parked            = false;                         // Used in mode 1, 2 & 3
  millis_t delayed_move_time             = 0;                             // Used in mode 1
  celsius_t duplicate_extruder_temp_offset = 0;                           // Used in mode 2 & 3
  bool idex_mirrored_mode                = false;                         // Used in mode 3

  float x_home_pos(const uint8_t extruder) {
    if (extruder == 0) return X_HOME_POS;

    /**
     * In dual carriage mode the extruder offset provides an override of the
     * second X-carriage position when homed - otherwise X2_HOME_POS is used.
     * This allows soft recalibration of the second extruder home position
     * (with M218 T1 Xn) without firmware reflash.
     */
    return hotend_offset[1].x > 0 ? hotend_offset[1].x : X2_HOME_POS;
  }

  void idex_set_mirrored_mode(const bool mirr) {
    idex_mirrored_mode = mirr;
    stepper.apply_directions();
  }

  void set_duplication_enabled(const bool dupe, const int8_t tool_index/*=-1*/) {
    extruder_duplication_enabled = dupe;
    if (tool_index >= 0) active_extruder = tool_index;
    stepper.apply_directions();
  }

  void idex_set_parked(const bool park/*=true*/) {
    delayed_move_time = 0;
    active_extruder_parked = park;
    if (park) raised_parked_position = current_position;  // Remember current raised toolhead position for use by unpark
  }

  /**
   * Prepare a linear move in a dual X axis setup
   *
   * Return true if current_position[] was set to destination[]
   */
  inline bool dual_x_carriage_unpark() {
    if (active_extruder_parked) {
      switch (dual_x_carriage_mode) {

        case DXC_FULL_CONTROL_MODE: break;

        case DXC_AUTO_PARK_MODE: {
          if (current_position.e == destination.e) {
            // This is a travel move (with no extrusion)
            // Skip it, but keep track of the current position
            // (so it can be used as the start of the next non-travel move)
            if (delayed_move_time != 0xFFFFFFFFUL) {
              current_position = destination;
              NOLESS(raised_parked_position.z, destination.z);
              delayed_move_time = millis() + 1000UL;
              return true;
            }
          }
          //
          // Un-park the active extruder
          //
          const feedRate_t fr_zfast = planner.settings.max_feedrate_mm_s[Z_AXIS];
          //  1. Move to the raised parked XYZ. Presumably the tool is already at XY.
          xyze_pos_t raised = raised_parked_position; raised.e = current_position.e;
          if (planner.buffer_line(raised, fr_zfast)) {
            //  2. Move to the current native XY and raised Z. Presumably this is a null move.
            xyze_pos_t curpos = current_position; curpos.z = raised_parked_position.z;
            if (planner.buffer_line(curpos, PLANNER_XY_FEEDRATE())) {
              //  3. Lower Z back down
              line_to_current_position(fr_zfast);
            }
          }
          stepper.apply_directions();

          idex_set_parked(false);
          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("idex_set_parked(false)");
        } break;

        case DXC_MIRRORED_MODE:
        case DXC_DUPLICATION_MODE:
          if (active_extruder == 0) {
            set_duplication_enabled(false); // Clear stale duplication state
            // Restore planner to parked head (T1) X position
            float x0_pos = current_position.x;
            xyze_pos_t pos_now = current_position;
            pos_now.x = inactive_extruder_x;
            planner.set_position_mm(pos_now);

            // Keep the same X or add the duplication X offset
            xyze_pos_t new_pos = pos_now;
            if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
              new_pos.x = x0_pos + duplicate_extruder_x_offset;
            else
              new_pos.x = _MIN(X_BED_SIZE - x0_pos, X_MAX_POS);

            // Move duplicate extruder into the correct position
            if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Set planner X", inactive_extruder_x, " ... Line to X", new_pos.x);
            if (!planner.buffer_line(new_pos, planner.settings.max_feedrate_mm_s[X_AXIS], 1)) break;
            planner.synchronize();

            sync_plan_position();             // Extra sync for good measure
            set_duplication_enabled(true);    // Enable Duplication
            idex_set_parked(false);           // No longer parked
            if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("set_duplication_enabled(true)\nidex_set_parked(false)");
          }
          else if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Active extruder not 0");
          break;
      }
    }
    return false;
  }

#endif // DUAL_X_CARRIAGE

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 *
 * Make sure current_position.e and destination.e are good
 * before calling or cold/lengthy extrusion may get missed.
 *
 * Before exit, current_position is set to destination.
 */
void prepare_line_to_destination() {
  apply_motion_limits(destination);

  #if ANY(PREVENT_COLD_EXTRUSION, PREVENT_LENGTHY_EXTRUDE)

    if (!DEBUGGING(DRYRUN) && destination.e != current_position.e) {
      bool ignore_e = thermalManager.tooColdToExtrude(active_extruder);
      if (ignore_e) SERIAL_ECHO_MSG(STR_ERR_COLD_EXTRUDE_STOP);

      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        const float e_delta = ABS(destination.e - current_position.e) * planner.e_factor[active_extruder];
        if (e_delta > (EXTRUDE_MAXLENGTH)) {
          #if ENABLED(MIXING_EXTRUDER)
            float collector[MIXING_STEPPERS];
            mixer.refresh_collector(1.0, mixer.get_current_vtool(), collector);
            MIXER_STEPPER_LOOP(e) {
              if (e_delta * collector[e] > (EXTRUDE_MAXLENGTH)) {
                ignore_e = true;
                SERIAL_ECHO_MSG(STR_ERR_LONG_EXTRUDE_STOP);
                break;
              }
            }
          #else
            ignore_e = true;
            SERIAL_ECHO_MSG(STR_ERR_LONG_EXTRUDE_STOP);
          #endif
        }
      #endif

      if (ignore_e) {
        current_position.e = destination.e;       // Behave as if the E move really took place
        planner.set_e_position_mm(destination.e); // Prevent the planner from complaining too
      }
    }

  #endif // PREVENT_COLD_EXTRUSION || PREVENT_LENGTHY_EXTRUDE

  if (TERN0(DUAL_X_CARRIAGE, dual_x_carriage_unpark())) return;

  if (
    #if UBL_SEGMENTED
      #if IS_KINEMATIC // UBL using Kinematic / Cartesian cases as a workaround for now.
        bedlevel.line_to_destination_segmented(MMS_SCALED(feedrate_mm_s))
      #else
        line_to_destination_cartesian()
      #endif
    #elif IS_KINEMATIC
      line_to_destination_kinematic()
    #else
      line_to_destination_cartesian()
    #endif
  ) return;

  current_position = destination;
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are trusted and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
void set_axis_is_at_home(const AxisEnum axis) {
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM(">>> set_axis_is_at_home(", C(AXIS_CHAR(axis)), ")");

  set_axis_trusted(axis);
  set_axis_homed(axis);

  #if ENABLED(DUAL_X_CARRIAGE)
    if (axis == X_AXIS && (active_extruder == 1 || dual_x_carriage_mode == DXC_DUPLICATION_MODE)) {
      current_position.x = x_home_pos(active_extruder);
      return;
    }
  #endif

  #if ANY(MORGAN_SCARA, AXEL_TPARA)
    scara_set_axis_is_at_home(axis);
  #elif ENABLED(DELTA)
    current_position[axis] = (axis == Z_AXIS) ? DIFF_TERN(HAS_BED_PROBE, delta_height, probe.offset.z) : base_home_pos(axis);
  #else
    current_position[axis] = base_home_pos(axis);
  #endif

  /**
   * Z Probe Z Homing? Account for the probe's Z offset.
   */
  #if HAS_BED_PROBE && Z_HOME_TO_MIN
    if (axis == Z_AXIS) {
      #if HOMING_Z_WITH_PROBE
        current_position.z -= probe.offset.z;
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("*** Z HOMED WITH PROBE (Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) ***\n> probe.offset.z = ", probe.offset.z);
      #else
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("*** Z HOMED TO ENDSTOP ***");
      #endif
    }
  #endif

  TERN_(I2C_POSITION_ENCODERS, I2CPEM.homed(axis));

  TERN_(BABYSTEP_DISPLAY_TOTAL, babystep.reset_total(axis));

  #if HAS_POSITION_SHIFT
    position_shift[axis] = 0;
    update_workspace_offset(axis);
  #endif

  if (DEBUGGING(LEVELING)) {
    #if HAS_HOME_OFFSET
      DEBUG_ECHOLNPGM("> home_offset[", C(AXIS_CHAR(axis)), "] = ", home_offset[axis]);
    #endif
    DEBUG_POS("", current_position);
    DEBUG_ECHOLNPGM("<<< set_axis_is_at_home(", C(AXIS_CHAR(axis)), ")");
  }
}

#if HAS_WORKSPACE_OFFSET
  void update_workspace_offset(const AxisEnum axis) {
    workspace_offset[axis] = home_offset[axis] + position_shift[axis];
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Axis ", C(AXIS_CHAR(axis)), " home_offset = ", home_offset[axis], " position_shift = ", position_shift[axis]);
  }
#endif

#if HAS_M206_COMMAND
  /**
   * Change the home offset for an axis.
   * Also refreshes the workspace offset.
   */
  void set_home_offset(const AxisEnum axis, const_float_t v) {
    home_offset[axis] = v;
    update_workspace_offset(axis);
  }
#endif
