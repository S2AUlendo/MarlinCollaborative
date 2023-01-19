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

#pragma once

#include "../inc/MarlinConfigPre.h" // Access the top level configurations.
#include "../module/planner.h"      // Access block type from planner.


#ifdef FXDTICTRL

    enum fxdTiCtrlMode_t {
        fxdTiCtrlMode_t_DISABLED = 0U,
        fxdTiCtrlMode_t_ENABLED = 1U,
        fxdTiCtrlMode_t_ULENDO_FBS = 2U,
        fxdTiCtrlMode_t_ZV = 10U,
        fxdTiCtrlMode_t_ZVD = 11U,
        fxdTiCtrlMode_t_EI = 12U,
        fxdTiCtrlMode_t_2HEI = 13U,
        fxdTiCtrlMode_t_3HEI = 14U,
        fxdTiCtrlMode_t_MZV = 15U,
        fxdTiCtrlMode_t_DISCTF = 20U
    };

    enum dynFreqMode_t {
        dynFreqMode_t_DISABLED = 0U,
        dynFreqMode_t_Z_POSN_BASED = 1U,
        dynFreqMode_t_MASS_BASED = 2U
    };

    enum stepDirState_t {
        stepDirState_t_NOT_SET = 0U,
        stepDirState_t_POS = 1U,
        stepDirState_t_NEG = 2U
    };

    class FxdTiCtrl {

        public:
        
            // Public variables.
            static fxdTiCtrlMode_t cfg_mode;                        // Mode / active compensation mode configuration.
            static bool cfg_linearAdvEna;                           // Linear advance enable configuration.
            static float cfg_linearAdvK;                            // Linear advance gain.
            static dynFreqMode_t cfg_dynFreqMode;                   // Dynamic frequency mode configuration.
            static float cfg_baseFreq[2];                           // Base frequency. [Hz]
            static float cfg_dynFreqK[2];                           // Scaling / gain for dynamic frequency. [Hz/mm] or [Hz/g]

            static uint8_t stepperCmdBuff[FXDTICTRL_STEPPERCMD_BUFF_SIZE];                      // Buffer of stepper commands.
            static hal_timer_t stepperCmdBuff_StepRelativeTi[FXDTICTRL_STEPPERCMD_BUFF_SIZE];   // Buffer of the stepper command timing.
            static bool stepperCmdBuff_ApplyDir[FXDTICTRL_STEPPERCMD_BUFF_SIZE];                // Buffer of whether DIR needs to be updated.
            static uint32_t stepperCmdBuff_produceIdx;                      // Index of next stepper command write to the buffer.
            static uint32_t stepperCmdBuff_consumeIdx;                      // Index of next stepper command read from the buffer.

            static bool sts_stepperBusy;        // The stepper buffer has items and is in use.

            
            // Public functions.
            static void startBlockProc(block_t * current_block);    // Sets controller states to begin processing a block.
            static bool getBlockProcDn();                           // Returns true if the controller no longer needs the current block.
            static void runoutBlock();                              // Moves any free data points to the stepper buffer even if a full
                                                                    // batch isn't ready.
            static void loop();                                     // Controller main, to be invoked from non-isr task.
            
            static void updateShapingA(float zeta, float vtol);     // Refreshes the gains used by shaping functions.
                                                                    // To be called on init or mode or zeta change.
            static void updateShapingN(float xf, float yf, float zeta); // Refreshes the indices used by shaping functions.
                                                                        // To be called when frequencies change.
            static void reset();                                        // Resets all states of the fixed time conversion to defaults.

        private:

            // Private variables.
            static float xd[(FXDTICTRL_BATCH_SIZE*2)];
            static float yd[(FXDTICTRL_BATCH_SIZE*2)];
            static float zd[(FXDTICTRL_BATCH_SIZE*2)];
            static float ed[(FXDTICTRL_BATCH_SIZE*2)];
            static float xm[FXDTICTRL_BATCH_SIZE];
            static float ym[FXDTICTRL_BATCH_SIZE];
            static float zm[FXDTICTRL_BATCH_SIZE];
            static float em[FXDTICTRL_BATCH_SIZE];

            static block_t * current_block_cpy;
            static bool blockProcRdy;
            static bool blockProcRdy_z1;
            static bool blockProcDn;
            static bool batchRdy;
            static bool batchRdyForInterp;
            static bool runoutEna;
            
            // Trapezoid data variables.
            static float totalLength;
            static float x_startPosn;
            static float y_startPosn;
            static float z_startPosn;
            static float e_startPosn;
            static float x_endPosn_prevBlock;
            static float y_endPosn_prevBlock;
            static float z_endPosn_prevBlock;
            static float e_endPosn_prevBlock;
            static float x_Ratio;
            static float y_Ratio;
            static float z_Ratio;
            static float e_Ratio;
            static float accel_P, decel_P;
            static float F_P;
            static float f_s;
            static float s_1e;
            static float s_2e;
            
            static uint32_t N1, N2, N3;
            static uint32_t max_intervals;

            // Make vector variables.
            static uint32_t makeVector_idx;
            static uint32_t makeVector_idx_z1;
            static uint32_t makeVector_batchIdx;

            // Interpolation variables.
            static int32_t x_steps;
            static int32_t y_steps;
            static int32_t z_steps;
            static int32_t e_steps;
            static uint32_t interpIdx;
            static uint32_t interpIdx_z1;
            static stepDirState_t x_dirState;
            static stepDirState_t y_dirState;
            static stepDirState_t z_dirState;
            static stepDirState_t e_dirState;
            static hal_timer_t nextStepTicks;

            // Shaping variables.
            static float xd_zi[FXDTICTRL_ZMAX];
            static float yd_zi[FXDTICTRL_ZMAX];
            static uint32_t xy_zi_idx;
            static float x_Ai[5];
            static float y_Ai[5];
            static uint32_t x_Ni[5];
            static uint32_t y_Ni[5];
            static uint32_t xy_max_i;
            
            // Linear advance variables.
            static float e_raw_z1;
            static float e_advanced_z1;


            // Private functions.
            static uint32_t stepperCmdBuffItems();
            static void init();
            static void loadBlockData(block_t * current_block);
            static void makeVector();
            static void convertToSteps(uint32_t idx);

    }; /* class fxdTiCtrl */

    extern FxdTiCtrl fxdTiCtrl;

#endif
