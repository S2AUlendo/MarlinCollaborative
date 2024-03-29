/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#pragma once

#include "../inc/MarlinConfigPre.h" // Access the top level configurations.
#include "../module/planner.h"      // Access block type from planner.

#include "ft_types.h"


typedef struct FTConfig {
  ftMotionMode_t mode = FTM_DEFAULT_MODE;                 // Mode / active compensation mode configuration.

  #if HAS_X_AXIS
    ftMotionCmpnstr_t cmpnstr[1 + ENABLED(HAS_Y_AXIS)] =               // Compensation mode.
      { FTM_DEFAULT_X_COMPENSATOR OPTARG(HAS_Y_AXIS, FTM_DEFAULT_Y_COMPENSATOR) };
    float baseFreq[1 + ENABLED(HAS_Y_AXIS)] =             // Base frequency. [Hz]
      { FTM_SHAPING_DEFAULT_X_FREQ OPTARG(HAS_Y_AXIS, FTM_SHAPING_DEFAULT_Y_FREQ) };
    float zeta[1 + ENABLED(HAS_Y_AXIS)] =                 // Damping factor
        { FTM_SHAPING_DEFAULT_ZETA_X OPTARG(HAS_Y_AXIS, FTM_SHAPING_DEFAULT_ZETA_Y) };
    float vtol[1 + ENABLED(HAS_Y_AXIS)] =                 // Vibration Level
        { FTM_SHAPING_DEFAULT_V_TOL_X OPTARG(HAS_Y_AXIS, FTM_SHAPING_DEFAULT_V_TOL_Y) };
  #endif

  #if HAS_EXTRUDERS
    bool linearAdvEna = FTM_LINEAR_ADV_DEFAULT_ENA;       // Linear advance enable configuration.
    float linearAdvK = FTM_LINEAR_ADV_DEFAULT_K;          // Linear advance gain.
  #endif
} ft_config_t;

class FTMotion {

  public:

    // Public variables
    static ft_config_t cfg;
    static ftMotionTrajGenConfig_t traj_gen_cfg;
    static bool busy;

    static void set_defaults() {
      cfg.mode = FTM_DEFAULT_MODE;

      TERN_(HAS_X_AXIS, cfg.cmpnstr[X_AXIS] = FTM_DEFAULT_X_COMPENSATOR);
      TERN_(HAS_Y_AXIS, cfg.cmpnstr[Y_AXIS] = FTM_DEFAULT_Y_COMPENSATOR);

      TERN_(HAS_X_AXIS, cfg.baseFreq[X_AXIS] = FTM_SHAPING_DEFAULT_X_FREQ);
      TERN_(HAS_Y_AXIS, cfg.baseFreq[Y_AXIS] = FTM_SHAPING_DEFAULT_Y_FREQ);

      TERN_(HAS_X_AXIS, cfg.zeta[X_AXIS] = FTM_SHAPING_DEFAULT_ZETA_X);
      TERN_(HAS_Y_AXIS, cfg.zeta[Y_AXIS] = FTM_SHAPING_DEFAULT_ZETA_Y);

      TERN_(HAS_X_AXIS, cfg.vtol[X_AXIS] = FTM_SHAPING_DEFAULT_V_TOL_X);
      TERN_(HAS_Y_AXIS, cfg.vtol[Y_AXIS] = FTM_SHAPING_DEFAULT_V_TOL_Y);

      #if HAS_EXTRUDERS
        cfg.linearAdvEna = FTM_LINEAR_ADV_DEFAULT_ENA;
        cfg.linearAdvK = FTM_LINEAR_ADV_DEFAULT_K;
      #endif

      #if HAS_X_AXIS
        update_shaping_params();
      #endif

      reset();
    }

    static ft_command_t stepperCmdBuff[FTM_STEPPERCMD_BUFF_SIZE]; // Buffer of stepper commands.
    static int32_t stepperCmdBuff_produceIdx,             // Index of next stepper command write to the buffer.
                   stepperCmdBuff_consumeIdx;             // Index of next stepper command read from the buffer.

    static bool sts_stepperBusy;                          // The stepper buffer has items and is in use.

    static millis_t axis_pos_move_end_ti[NUM_AXIS_ENUMS],
                    axis_neg_move_end_ti[NUM_AXIS_ENUMS];

    // Public methods
    static void init();
    static void loop();                                   // Controller main, to be invoked from non-isr task.

    #if HAS_X_AXIS
      // Refreshes the gains and indices used by shaping functions.
      static void update_shaping_params(void);
    #endif

    static void reset();                                  // Reset all states of the fixed time conversion to defaults.

    static void setup_traj_gen(uint32_t intervals);

    static bool axis_moving_pos(const AxisEnum axis) { return !ELAPSED(millis(), axis_pos_move_end_ti[axis]); }
    static bool axis_moving_neg(const AxisEnum axis) { return !ELAPSED(millis(), axis_neg_move_end_ti[axis]); }

  private:

    static xyze_trajectory_t traj;
    static xyze_trajectoryMod_t trajMod;

    static bool blockProcRdy;
    static bool batchRdy, batchRdyForInterp;

    // Trapezoid data variables.
    static xyze_pos_t   startPosn,          // (mm) Start position of block
                        endPosn_prevBlock;  // (mm) End position of previous block
    static xyze_float_t ratio;              // (ratio) Axis move ratio of block
    static float accel_P, decel_P,
                 F_P,
                 f_s,
                 s_1e,
                 s_2e;

    static uint32_t N1, N2, N3;
    static uint32_t max_intervals;

    static constexpr uint32_t PROP_BATCHES = CEIL(FTM_WINDOW_SIZE/FTM_BATCH_SIZE) - 1; // Number of batches needed to propagate the current trajectory to the stepper.

    // Make vector variables.
    static uint32_t makeVector_idx,
                    makeVector_batchIdx;

    // Interpolation variables.
    static uint32_t interpIdx;

    static xyze_long_t steps;

    // Shaping variables.
    #if HAS_X_AXIS

      typedef struct AxisShaping {
        bool ena;                         // Enabled indication.
        float d_zi[FTM_ZMAX];             // Data point delay vector.
        float Ai[5];                      // Shaping gain vector.
        uint32_t Ni[5];                   // Shaping time index vector.
        uint32_t max_i;                   // Vector length for the selected shaper.

        void set_axis_shaping_N(const ftMotionCmpnstr_t shaper, const_float_t f, const_float_t zeta);    // Sets the gains used by shaping functions.
        void set_axis_shaping_A(const ftMotionCmpnstr_t shaper, const_float_t zeta, const_float_t vtol); // Sets the indices used by shaping functions.

      } axis_shaping_t;

      typedef struct Shaping {
        uint32_t zi_idx;           // Index of storage in the data point delay vectors.   
        axis_shaping_t x;
        #if HAS_Y_AXIS
          axis_shaping_t y;
        #endif

      } shaping_t;

      static shaping_t shaping; // Shaping data

    #endif // HAS_X_AXIS

    // Linear advance variables.
    #if HAS_EXTRUDERS
      static float e_raw_z1, e_advanced_z1;
    #endif

    // Private methods
    static void discard_planner_block_protected();
    static void runoutBlock();
    static int32_t stepperCmdBuffItems();
    static void loadBlockData(block_t *const current_block);
    static void makeVector();
    static void convertToSteps(const uint32_t idx);

    FORCE_INLINE static int32_t num_samples_cmpnstr_settle() { return ( shaping.x.ena || shaping.y.ena ) ? FTM_ZMAX : 0; }


}; // class FTMotion

extern FTMotion ftMotion;
