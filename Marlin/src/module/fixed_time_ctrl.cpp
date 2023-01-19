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

#include "fixed_time_ctrl.h"
#include "stepper.h" // Access stepper block queue function and abort status.


#ifdef FXDTICTRL

    FxdTiCtrl fxdTiCtrl;

    //-----------------------------------------------------------------//
    // Variables.
    //-----------------------------------------------------------------//

    // Public variables.
    fxdTiCtrlMode_t FxdTiCtrl::cfg_mode = FXDTICTRL_DEFAULT_MODE;               // Mode / active compensation mode configuration.
    bool FxdTiCtrl::cfg_linearAdvEna = FXDTICTRL_LINEAR_ADV_DEFAULT_ENA;        // Linear advance enable configuration.
    float FxdTiCtrl::cfg_linearAdvK = FXDTICTRL_LINEAR_ADV_DEFAULT_K;           // Linear advance gain.
    dynFreqMode_t FxdTiCtrl::cfg_dynFreqMode = FXDTICTRL_DEFAULT_DYNFREQ_MODE;  // Dynamic frequency mode configuration.
    float FxdTiCtrl::cfg_baseFreq[2] = {FXDTICTRL_SHAPING_DEFAULT_X_FREQ,       // Base frequency. [Hz]
                                        FXDTICTRL_SHAPING_DEFAULT_Y_FREQ};
    float FxdTiCtrl::cfg_dynFreqK[2] = {0.0f, 0.0f};                            // Scaling / gain for dynamic frequency. [Hz/mm] or [Hz/g]

    uint8_t FxdTiCtrl::stepperCmdBuff[FXDTICTRL_STEPPERCMD_BUFF_SIZE] = {0U};                       // Buffer of stepper commands.
    hal_timer_t FxdTiCtrl::stepperCmdBuff_StepRelativeTi[FXDTICTRL_STEPPERCMD_BUFF_SIZE] = {0U};    // Buffer of the stepper command timing.
    bool FxdTiCtrl::stepperCmdBuff_ApplyDir[FXDTICTRL_STEPPERCMD_BUFF_SIZE] = {false};              // Buffer of whether DIR needs to be updated.
    uint32_t FxdTiCtrl::stepperCmdBuff_produceIdx = 0U;         // Index of next stepper command write to the buffer.
    uint32_t FxdTiCtrl::stepperCmdBuff_consumeIdx = 0U;         // Index of next stepper command read from the buffer.

    bool FxdTiCtrl::sts_stepperBusy = false;                // The stepper buffer has items and is in use.


    // Private variables.
                                                                // NOTE: These are sized for Ulendo FBS use.
    float FxdTiCtrl::xd[(FXDTICTRL_BATCH_SIZE*2)] = {0.0f};     // Storage for fixed-time-based trajectory.
    float FxdTiCtrl::yd[(FXDTICTRL_BATCH_SIZE*2)] = {0.0f};     // Storage for fixed-time-based trajectory.
    float FxdTiCtrl::zd[(FXDTICTRL_BATCH_SIZE*2)] = {0.0f};     // Storage for fixed-time-based trajectory.
    float FxdTiCtrl::ed[(FXDTICTRL_BATCH_SIZE*2)] = {0.0f};     // Storage for fixed-time-based trajectory.
    float FxdTiCtrl::xm[FXDTICTRL_BATCH_SIZE] = {0.0f};         // Storage for modified fixed-time-based trajectory.
    float FxdTiCtrl::ym[FXDTICTRL_BATCH_SIZE] = {0.0f};         // Storage for modified fixed-time-based trajectory.
    float FxdTiCtrl::zm[FXDTICTRL_BATCH_SIZE] = {0.0f};         // Storage for modified fixed-time-based trajectory.
    float FxdTiCtrl::em[FXDTICTRL_BATCH_SIZE] = {0.0f};         // Storage for modified fixed-time-based trajectory.

    block_t * FxdTiCtrl::current_block_cpy = nullptr; // Pointer to current block being processed.
    bool FxdTiCtrl::blockProcRdy = false;       // Indicates a block is ready to be processed.
    bool FxdTiCtrl::blockProcRdy_z1 = false;    // Storage for the previous indicator.
    bool FxdTiCtrl::blockProcDn = false;        // Indicates current block is done being processed.
    bool FxdTiCtrl::batchRdy = false;           // Indicates a batch of the fixed time trajectory
                                                // has been generated, is now available in the upper -
                                                // half of xd, yd, zd, ed vectors, and is ready to be
                                                // post processed, if applicable, then interpolated.
    bool FxdTiCtrl::batchRdyForInterp = false;  // Indicates the batch is done being post processed,
                                                // if applicable, and is ready to be converted to step commands.
    bool FxdTiCtrl::runoutEna = false;          // True if runout of the block hasn't been done and is allowed.

    // Trapezoid data variables.
    float FxdTiCtrl::totalLength;                   // Total movement length of block. [mm]
    float FxdTiCtrl::x_startPosn;                   // Start position of block. [mm]
    float FxdTiCtrl::y_startPosn;                   // Start position of block. [mm]
    float FxdTiCtrl::z_startPosn;                   // Start position of block. [mm]
    float FxdTiCtrl::e_startPosn;                   // Start position of block. [mm]
    float FxdTiCtrl::x_endPosn_prevBlock = 0.0f;    // Start position of block. [mm]
    float FxdTiCtrl::y_endPosn_prevBlock = 0.0f;    // Start position of block. [mm]
    float FxdTiCtrl::z_endPosn_prevBlock = 0.0f;    // Start position of block. [mm]
    float FxdTiCtrl::e_endPosn_prevBlock = 0.0f;    // Start position of block. [mm]
    float FxdTiCtrl::x_Ratio;                       // Axis move ratio of block. [unitless]
    float FxdTiCtrl::y_Ratio;                       // Axis move ratio of block. [unitless]
    float FxdTiCtrl::z_Ratio;                       // Axis move ratio of block. [unitless]
    float FxdTiCtrl::e_Ratio;                       // Axis move ratio of block. [unitless]
    float FxdTiCtrl::accel_P;                       // Acceleration prime of block. [mm/sec/sec]
    float FxdTiCtrl::decel_P;                       // Deceleration prime of block. [mm/sec/sec]
    float FxdTiCtrl::F_P;                           // Feedrate prime of block. [mm/sec]
    float FxdTiCtrl::f_s;                           // Starting feedrate of block. [mm/sec]
    float FxdTiCtrl::s_1e;                          // Position after acceleration phase of block.
    float FxdTiCtrl::s_2e;                          // Position after acceleration and coasting phase of block.

    uint32_t FxdTiCtrl::N1;             // Number of data points in the acceleration phase.
    uint32_t FxdTiCtrl::N2;             // Number of data points in the coasting phase.
    uint32_t FxdTiCtrl::N3;             // Number of data points in the deceleration phase.

    uint32_t FxdTiCtrl::max_intervals;  // Total number of data points that will be generated from block.

    // Make vector variables.
    uint32_t FxdTiCtrl::makeVector_idx = 0U;                            // Index of fixed time trajectory generation of the overall block.
    uint32_t FxdTiCtrl::makeVector_idx_z1 = 0U;                         // Storage for the previously calculated index above.
    uint32_t FxdTiCtrl::makeVector_batchIdx = FXDTICTRL_BATCH_SIZE;     // Index of fixed time trajectory generation within the batch.

    // Interpolation variables.
    int32_t FxdTiCtrl::x_steps = 0U;                // Step count accumulator.
    int32_t FxdTiCtrl::y_steps = 0U;                // Step count accumulator.
    int32_t FxdTiCtrl::z_steps = 0U;                // Step count accumulator.
    int32_t FxdTiCtrl::e_steps = 0U;                // Step count accumulator.
    uint32_t FxdTiCtrl::interpIdx = 0U;             // Index of current data point being interpolated.
    uint32_t FxdTiCtrl::interpIdx_z1 = 0U;          // Storage for the previously calculated index above.
    stepDirState_t FxdTiCtrl::x_dirState = stepDirState_t_NOT_SET;  // Memory of the currently set step direction of the axis.
    stepDirState_t FxdTiCtrl::y_dirState = stepDirState_t_NOT_SET;  // Memory of the currently set step direction of the axis.
    stepDirState_t FxdTiCtrl::z_dirState = stepDirState_t_NOT_SET;  // Memory of the currently set step direction of the axis.
    stepDirState_t FxdTiCtrl::e_dirState = stepDirState_t_NOT_SET;  // Memory of the currently set step direction of the axis.
    hal_timer_t FxdTiCtrl::nextStepTicks = FXDTICTRL_MIN_TICKS;     // Accumulator for the next step time (in ticks).

    // Shaping variables.
    float FxdTiCtrl::xd_zi[FXDTICTRL_ZMAX] = {0.0f};    // Data point delay vector.
    float FxdTiCtrl::yd_zi[FXDTICTRL_ZMAX] = {0.0f};    // Data point delay vector.
    uint32_t FxdTiCtrl::xy_zi_idx = 0U;                 // Index of storage in the data point delay vectors.
    float FxdTiCtrl::x_Ai[5];                           // Shaping gain vector.
    float FxdTiCtrl::y_Ai[5];                           // Shaping gain vector.
    uint32_t FxdTiCtrl::x_Ni[5];                        // Shaping time index vector.
    uint32_t FxdTiCtrl::y_Ni[5];                        // Shaping time index vector.
    uint32_t FxdTiCtrl::xy_max_i = 0U;                  // Vector length for the selected shaper.

    // Linear advance variables.
    float FxdTiCtrl::e_raw_z1 = 0.0f;              // Unit delay of raw extruder position.
    float FxdTiCtrl::e_advanced_z1 = 0.0f;         // Unit delay of advanced extruder position.


    //-----------------------------------------------------------------//
    // Function definitions.
    //-----------------------------------------------------------------//

    // Public functions.

    // Sets controller states to begin processing a block.
    void FxdTiCtrl::startBlockProc(block_t * current_block){
        current_block_cpy = current_block;
        blockProcRdy = true;
        blockProcDn = false;
        runoutEna = true;
        return;
    }

    // Returns true if the controller no longer needs the current block.
    bool FxdTiCtrl::getBlockProcDn(){ return blockProcDn; }

    // Moves any free data points to the stepper buffer even if a full
    // batch isn't ready.
    void FxdTiCtrl::runoutBlock(){

        if(runoutEna && !batchRdy) {    // If the window is full already (block intervals was
                                        // a multiple of the batch size), or runout is not
                                        // enabled, no runout is needed.
            // Fill out the trajectory window with the last position calculated.
            if(makeVector_batchIdx > FXDTICTRL_BATCH_SIZE){
                for(uint32_t i = makeVector_batchIdx; i < (2*FXDTICTRL_BATCH_SIZE); i++){
                    xd[i] = xd[makeVector_batchIdx-1];
                    yd[i] = yd[makeVector_batchIdx-1];
                    zd[i] = zd[makeVector_batchIdx-1];
                    ed[i] = ed[makeVector_batchIdx-1];
                }
            }
            makeVector_batchIdx = FXDTICTRL_BATCH_SIZE;
            batchRdy = true;
        }
        runoutEna = false;
        return;
    }

    // Controller main, to be invoked from non-isr task.
    void FxdTiCtrl::loop(){

        if (!cfg_mode) { return; }

        static bool initd = false;

        if (!initd) { init(); initd = true; }

        // Handle block abort with the following sequence:
        // 1. Zero out commands in stepper ISR.
        // 2. Drain the motion buffer, stop processing until they are emptied.
        // 3. Reset all the states / memory.
        // 4. Signal ready for new block.
        if(stepper.abort_current_block){
            if (sts_stepperBusy) { return; }        // Wait until motion buffers are emptied
            reset();
            blockProcDn = true;                     // Set queueing to look for next block.
            runoutEna = false;                      // Disabling running out this block, since we want to halt the motion.
            stepper.abort_current_block = false;    // Abort finished.
        }

        // Planner processing and block conversion.
        if (!blockProcRdy){ stepper.fxdTiCtrl_BlockQueueUpdate(); }
        if (blockProcRdy){
            if (!blockProcRdy_z1){ // One-shot.
            loadBlockData(current_block_cpy);
            }
            while ( !blockProcDn && !batchRdy && \
                    ((makeVector_idx - makeVector_idx_z1) < FXDTICTRL_POINTS_PER_LOOP) ){ makeVector(); }
        }

        // FBS / post processing.
        if(batchRdy && !batchRdyForInterp){

            // Call Ulendo FBS here.

            memcpy(xm, &xd[FXDTICTRL_BATCH_SIZE], sizeof(xm));
            memcpy(ym, &yd[FXDTICTRL_BATCH_SIZE], sizeof(ym));
            
            // Done compensating ...

            // Copy the uncompensated vectors.
            memcpy(zm, &zd[FXDTICTRL_BATCH_SIZE], sizeof(zm));
            memcpy(em, &ed[FXDTICTRL_BATCH_SIZE], sizeof(em));
            
            // Shift the time series back in the window.
            memcpy(xd, &xd[FXDTICTRL_BATCH_SIZE], (sizeof(xd) / 2));
            memcpy(yd, &yd[FXDTICTRL_BATCH_SIZE], (sizeof(yd) / 2));
            // Disabled by comment as these are uncompensated, the lower half is not used.
            // memcpy(zd, &zd[FXDTICTRL_BATCH_SIZE], (sizeof(zd) / 2));
            // memcpy(ed, &ed[FXDTICTRL_BATCH_SIZE], (sizeof(ed) / 2));

            // ... data is ready in xm, ym, zm, em.
            batchRdyForInterp = true; 

            batchRdy = false; // Clear so that makeVector() may resume generating points.

        } /* if(batchRdy && !batchRdyForInterp) */

        // Interpolation.
        while( batchRdyForInterp && \
                ( stepperCmdBuffItems() < (FXDTICTRL_STEPPERCMD_BUFF_SIZE - FXDTICTRL_STEPS_PER_UNIT_TIME) ) && \
                ( (interpIdx - interpIdx_z1) < FXDTICTRL_STEPS_PER_LOOP )
            ) {

            convertToSteps(interpIdx++);

            if(interpIdx == FXDTICTRL_BATCH_SIZE){
                batchRdyForInterp = false;
                interpIdx = 0U;
            }
        }

        // Report busy status to planner.
        planner.fxdTiCtrl_busy = (sts_stepperBusy || \
            ((!blockProcDn && blockProcRdy) || batchRdy || batchRdyForInterp || runoutEna) );

        blockProcRdy_z1 = blockProcRdy;
        makeVector_idx_z1 = makeVector_idx;
        interpIdx_z1 = interpIdx;

        return;
    }

    // Refreshes the gains used by shaping functions.
    // To be called on init or mode or zeta change.
    void FxdTiCtrl::updateShapingA(float zeta, float vtol){

        float K = exp( -zeta * PI / sqrt( 1.0f - zeta*zeta ) );

        switch(cfg_mode){
            case fxdTiCtrlMode_t_ZV:
                xy_max_i = 1U;
                x_Ai[0] = 1.0f / (1.0f + K);
                x_Ai[1] = x_Ai[0] * K;
                break;
            case fxdTiCtrlMode_t_ZVD:
                xy_max_i = 2U;
                x_Ai[0] = 1.0f / ( 1.0f + 2.0f*K + K*K );
                x_Ai[1] = x_Ai[0] * 2.0f*K;
                x_Ai[2] = x_Ai[0] * K*K;
                break;
            case fxdTiCtrlMode_t_EI:
                {
                    xy_max_i = 2U;
                    x_Ai[0] = 0.25f * (1.0f + vtol);
                    x_Ai[1] = 0.50f * (1.0f - vtol) * K;
                    x_Ai[2] = x_Ai[0] * K*K;
                    float A_adj = 1.0f / (x_Ai[0] + x_Ai[1] + x_Ai[2]);
                    for ( uint32_t i = 0U; i < 3U; i++ ) { x_Ai[i] = x_Ai[i] * A_adj; }
                }
                break;
            case fxdTiCtrlMode_t_2HEI:
                {
                    xy_max_i = 3U;
                    float vtol2 = vtol * vtol;
                    float X = pow(vtol2 * (sqrt(1.0f - vtol2) + 1.0f), 1.0f/3.0f);
                    x_Ai[0] = ( 3.0f*X*X + 2.0f*X + 3.0f*vtol2 ) / ( 16.0f*X );
                    x_Ai[1] = ( 0.5f - x_Ai[0] ) * K;
                    x_Ai[2] = x_Ai[1] * K;
                    x_Ai[3] = x_Ai[0] * K*K*K;
                    float A_adj = 1.0f / (x_Ai[0] + x_Ai[1] + x_Ai[2] + x_Ai[3]);
                    for ( uint32_t i = 0U; i < 4U; i++ ) { x_Ai[i] = x_Ai[i] * A_adj; }
                }
                break;
            case fxdTiCtrlMode_t_3HEI:
                {
                    xy_max_i = 4U;
                    x_Ai[0] = 0.0625f * ( 1.0f + 3.0f * vtol + 2.0f * sqrt( 2.0f * ( vtol + 1.0f ) * vtol ) );
                    x_Ai[1] = 0.25f * ( 1.0f - vtol ) * K;
                    x_Ai[2] = ( 0.5f * ( 1.0f + vtol ) - 2.0f * x_Ai[0] ) * K*K;
                    x_Ai[3] = x_Ai[1] * K*K;
                    x_Ai[4] = x_Ai[0] * K*K * K*K;
                    float A_adj = 1.0f / (x_Ai[0] + x_Ai[1] + x_Ai[2] + x_Ai[3] + x_Ai[4]);
                    for ( uint32_t i = 0U; i < 5U; i++ ) { x_Ai[i] = x_Ai[i] * A_adj; }
                }
                break;
            case fxdTiCtrlMode_t_MZV:
                {
                    xy_max_i = 2U;
                    float B = 1.4142135623730950488016887242097f * K;
                    x_Ai[0] = 1.0f / ( 1.0f + B + K*K );
                    x_Ai[1] = x_Ai[0] * B;
                    x_Ai[2] = x_Ai[0] * K*K;
                }
                break;
            default:
                for ( uint32_t i = 0U; i < 5U; i++ ) { x_Ai[i] = 0.0f; }
                xy_max_i = 0U;
        }
        memcpy(y_Ai, x_Ai, sizeof(x_Ai)); // For now, zeta and vtol are shared across x and y.
        return;
    }

    // Refreshes the indices used by shaping functions.
    // To be called when frequencies change.
    void FxdTiCtrl::updateShapingN(float xf, float yf, float zeta){

        // Protections omitted for DBZ and for index exceeding array length.

        float df = sqrt ( 1.0f - zeta*zeta );

        switch(cfg_mode){
            case fxdTiCtrlMode_t_ZV:
                x_Ni[1] = round( 0.5f / xf / df * FXDTICTRL_FS );
                y_Ni[1] = round( 0.5f / yf / df * FXDTICTRL_FS );
                break;
            case fxdTiCtrlMode_t_ZVD:
            case fxdTiCtrlMode_t_EI:
                x_Ni[1] = round( 0.5f / xf / df * FXDTICTRL_FS );
                y_Ni[1] = round( 0.5f / yf / df * FXDTICTRL_FS );
                x_Ni[2] = 2*x_Ni[1];
                y_Ni[2] = 2*y_Ni[1];
                break;
            case fxdTiCtrlMode_t_2HEI:
                x_Ni[1] = round( 0.5f / xf / df * FXDTICTRL_FS );
                y_Ni[1] = round( 0.5f / yf / df * FXDTICTRL_FS );
                x_Ni[2] = 2*x_Ni[1];
                y_Ni[2] = 2*y_Ni[1];
                x_Ni[3] = 3*x_Ni[1];
                y_Ni[3] = 3*y_Ni[1];
                break;
            case fxdTiCtrlMode_t_3HEI:
                x_Ni[1] = round( 0.5f / xf / df * FXDTICTRL_FS );
                y_Ni[1] = round( 0.5f / yf / df * FXDTICTRL_FS );
                x_Ni[2] = 2*x_Ni[1];
                y_Ni[2] = 2*y_Ni[1];
                x_Ni[3] = 3*x_Ni[1];
                y_Ni[3] = 3*y_Ni[1];
                x_Ni[4] = 4*x_Ni[1];
                y_Ni[4] = 4*y_Ni[1];
                break;
            case fxdTiCtrlMode_t_MZV:
                x_Ni[1] = round( 0.375f / xf / df * FXDTICTRL_FS );
                y_Ni[1] = round( 0.375f / yf / df * FXDTICTRL_FS );
                x_Ni[2] = 2*x_Ni[1];
                y_Ni[2] = 2*y_Ni[1];
                break;
            default:
                for ( uint32_t i = 0U; i < 5U; i++ ) { x_Ni[i] = 0U; y_Ni[i] = 0U; }
        }
        return;
    }

    // Resets all trajectory processing variables.
    void FxdTiCtrl::reset(){
        
        stepperCmdBuff_produceIdx = 0U; stepperCmdBuff_consumeIdx = 0U;

        for (uint32_t i = 0U; i < FXDTICTRL_BATCH_SIZE; i++){ // Reset trajectory history
            xd[i] = 0.0f; yd[i] = 0.0f; zd[i] = 0.0f; ed[i] = 0.0f;
        }

        blockProcRdy = false;
        blockProcRdy_z1 = false;
        blockProcDn = false;
        batchRdy = false;
        batchRdyForInterp = false;
        runoutEna = false;

        x_endPosn_prevBlock = 0.0f; y_endPosn_prevBlock = 0.0f;
        z_endPosn_prevBlock = 0.0f; e_endPosn_prevBlock = 0.0f;

        makeVector_idx = 0U;
        makeVector_idx_z1 = 0U;
        makeVector_batchIdx = FXDTICTRL_BATCH_SIZE;

        x_steps = 0U; y_steps = 0U; z_steps = 0U; e_steps = 0U;
        interpIdx = 0U;
        interpIdx_z1 = 0U;
        x_dirState = stepDirState_t_NOT_SET;
        y_dirState = stepDirState_t_NOT_SET;
        z_dirState = stepDirState_t_NOT_SET;
        e_dirState = stepDirState_t_NOT_SET;
        nextStepTicks = FXDTICTRL_MIN_TICKS;

        for(uint32_t i = 0U; i < FXDTICTRL_ZMAX; i++) xd_zi[i] = 0.0f;
        for(uint32_t i = 0U; i < FXDTICTRL_ZMAX; i++) yd_zi[i] = 0.0f;
        xy_zi_idx = 0U;

        e_raw_z1 = 0.0f; e_advanced_z1 = 0.0f;
        
        return;
    }

    // Private functions.
    // Auxilliary function to get number of step commands in the buffer.
    uint32_t FxdTiCtrl::stepperCmdBuffItems(){
        if (stepperCmdBuff_produceIdx < stepperCmdBuff_consumeIdx) {
            return (FXDTICTRL_STEPPERCMD_BUFF_SIZE + stepperCmdBuff_produceIdx - stepperCmdBuff_consumeIdx);
        } else {
            return (stepperCmdBuff_produceIdx - stepperCmdBuff_consumeIdx);
        }
    }

    // Initializes storage variables before startup.
    void FxdTiCtrl::init(){
        updateShapingN( cfg_baseFreq[0], cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
        updateShapingA( FXDTICTRL_SHAPING_ZETA, FXDTICTRL_SHAPING_V_TOL );
        reset(); // Precautionary.
        return;
    }

    // Loads / converts block data from planner to fixed-time control variables.
    void FxdTiCtrl::loadBlockData(block_t * current_block){

        x_startPosn = x_endPosn_prevBlock;
        y_startPosn = y_endPosn_prevBlock;
        z_startPosn = z_endPosn_prevBlock;
        e_startPosn = e_endPosn_prevBlock;

        totalLength = current_block->millimeters;

        uint8_t direction = current_block->direction_bits;

        float x_moveDist = current_block->steps.a / planner.settings.axis_steps_per_mm[X_AXIS];
        x_moveDist *= ((direction & (1 << X_AXIS)) == 0U) ? 1.0f : -1.0f;
        float y_moveDist = current_block->steps.b / planner.settings.axis_steps_per_mm[Y_AXIS];
        y_moveDist *= ((direction & (1 << Y_AXIS)) == 0U) ? 1.0f : -1.0f;
        float z_moveDist = current_block->steps.c / planner.settings.axis_steps_per_mm[Z_AXIS];
        z_moveDist *= ((direction & (1 << Z_AXIS)) == 0U) ? 1.0f : -1.0f;
        float extrusion = current_block->steps.e / planner.settings.axis_steps_per_mm[E_AXIS_N(current_block->extruder)];
        extrusion *= ((direction & (1 << E_AXIS_N(current_block->extruder))) == 0U) ? 1.0f : -1.0f;

        float oneByTotalLength = 1.0f / totalLength;
        x_Ratio = x_moveDist * oneByTotalLength;
        y_Ratio = y_moveDist * oneByTotalLength;
        z_Ratio = z_moveDist * oneByTotalLength;
        e_Ratio = extrusion * oneByTotalLength;

        float F_n = current_block->nominal_speed;
        f_s = (current_block->initial_rate) * (current_block->millimeters / current_block->step_event_count);
        float f_e = (current_block->final_rate) * (current_block->millimeters / current_block->step_event_count);
        
        float a = current_block->acceleration;
        float d = -1.0f * a;

        float oneby2a = 1.0f / (2.0f*a);
        float oneby2d = 1.0f / (2.0f*d);
        float feSqByTwoD = f_e * f_e * oneby2d;
        float fsSqByTwoA = f_s * f_s * oneby2a;

        float T2 = (1.0f/F_n) * (totalLength - (oneby2a-oneby2d)*(F_n * F_n) - (feSqByTwoD-fsSqByTwoA));

        if (T2 < 0.0f)  {
            T2 = 0.0f;
            // Clip by intersection if speed can't be reached.
            F_n = SQRT((totalLength - (feSqByTwoD-fsSqByTwoA)) / (oneby2a-oneby2d));
        }

        float T1 = (F_n - f_s)/a;
        float T3 = (f_e - F_n)/d;

        N1 = ceil(T1*FXDTICTRL_FS);
        N2 = ceil(T2*FXDTICTRL_FS);
        N3 = ceil(T3*FXDTICTRL_FS);

        float T1_P = N1*FXDTICTRL_TS;
        float T2_P = N2*FXDTICTRL_TS;
        float T3_P = N3*FXDTICTRL_TS;

        F_P = (2.0f * totalLength - f_s * T1_P - f_e * T3_P) / (T1_P + 2.0f*T2_P + T3_P);

        accel_P = ( (N1 == 0U) ? 0.0f : ((F_P - f_s)/T1_P) );

        decel_P = (f_e - F_P)/T3_P;

        s_1e = f_s * T1_P + 0.5f * accel_P * T1_P * T1_P;

        s_2e = s_1e + F_P*T2_P;

        max_intervals = N1 + N2 + N3 - 1U;
        
        x_endPosn_prevBlock += x_moveDist;
        y_endPosn_prevBlock += y_moveDist;
        z_endPosn_prevBlock += z_moveDist;
        e_endPosn_prevBlock += extrusion;

        return;
    }

    // Generate data points of the trajectory.
    void FxdTiCtrl::makeVector(){

        float accel_k = 0.0f;
        float tau = (makeVector_idx + 1) * FXDTICTRL_TS;
        float dist = 0.0f;
        if ( makeVector_idx < N1 ){
            dist = f_s * tau + 0.5f * accel_P * tau * tau;
            accel_k = accel_P;
        }
        else if( makeVector_idx >= N1 && makeVector_idx < (N1 + N2) ){
            dist = s_1e + F_P * (tau - N1*FXDTICTRL_TS);
            accel_k = 0.0f;
        }
        else {
            float tau_ = tau - (N1 + N2)*FXDTICTRL_TS;
            dist = s_2e + F_P * tau_ + 0.5f * decel_P * tau_ * tau_;
            accel_k = decel_P;
        }

        xd[makeVector_batchIdx] = x_startPosn + x_Ratio * dist;
        yd[makeVector_batchIdx] = y_startPosn + y_Ratio * dist;
        zd[makeVector_batchIdx] = z_startPosn + z_Ratio * dist;

        if (cfg_linearAdvEna) {
            float dedt_tar = (e_startPosn + e_Ratio * dist - e_raw_z1) * FXDTICTRL_FS;
            
            float dedt_adj = 0.0f;
            if ( e_Ratio == 0.0f ) { dedt_adj = dedt_tar; }
            else if ( e_Ratio < 0.0f ) { dedt_adj = dedt_tar; }
            else { dedt_adj = dedt_tar + accel_k * cfg_linearAdvK; }

            ed[makeVector_batchIdx] = e_advanced_z1 + dedt_adj * FXDTICTRL_TS;

            e_raw_z1 = e_startPosn + e_Ratio * dist;
            e_advanced_z1 = ed[makeVector_batchIdx];
        } else {
            ed[makeVector_batchIdx] = e_startPosn + e_Ratio * dist;
            // Alternatively: coordArray_e[makeVector_batchIdx] = e_startDist + extrusion / (N1 + N2 + N3);
        }

        // Update shaping parameters if needed.
        static float zd_z1 = 0.0f;
        switch(cfg_dynFreqMode){
            case dynFreqMode_t_Z_POSN_BASED:
                if (zd[makeVector_batchIdx] != zd_z1){ // Only update if z value changed.
                    float xf = cfg_baseFreq[0] + cfg_dynFreqK[0]*zd[makeVector_batchIdx];
                    xf = (xf > FXDTICTRL_MIN_SHAPE_FREQ) ? xf : FXDTICTRL_MIN_SHAPE_FREQ;
                    float yf = cfg_baseFreq[1] + cfg_dynFreqK[1]*zd[makeVector_batchIdx];
                    yf = (yf > FXDTICTRL_MIN_SHAPE_FREQ) ? yf : FXDTICTRL_MIN_SHAPE_FREQ;
                    updateShapingN( xf, yf, FXDTICTRL_SHAPING_ZETA );
                    zd_z1 = zd[makeVector_batchIdx];
                }
                break;
            case dynFreqMode_t_MASS_BASED:
                // Update constantly; the optimization done for z value makes less
                // sense for e, as e is expected to constantly change.
                updateShapingN( (cfg_baseFreq[0] + cfg_dynFreqK[0]*ed[makeVector_batchIdx]), \
                                (cfg_baseFreq[1] + cfg_dynFreqK[1]*ed[makeVector_batchIdx]), \
                                FXDTICTRL_SHAPING_ZETA );
                break;
            default: break;
        }

        // Apply shaping if in mode.
        if ( cfg_mode >= 10U && cfg_mode < 20U ){
            xd_zi[xy_zi_idx] = xd[makeVector_batchIdx];
            yd_zi[xy_zi_idx] = yd[makeVector_batchIdx];
            xd[makeVector_batchIdx] = x_Ai[0]*xd[makeVector_batchIdx];
            yd[makeVector_batchIdx] = y_Ai[0]*yd[makeVector_batchIdx];
            for ( uint32_t i = 1U; i <= xy_max_i; i++ ){
                xd[makeVector_batchIdx] += x_Ai[i] * xd_zi[ (x_Ni[i] > xy_zi_idx) ? (FXDTICTRL_ZMAX + xy_zi_idx - x_Ni[i]) : (xy_zi_idx - x_Ni[i]) ];
                yd[makeVector_batchIdx] += y_Ai[i] * yd_zi[ (y_Ni[i] > xy_zi_idx) ? (FXDTICTRL_ZMAX + xy_zi_idx - y_Ni[i]) : (xy_zi_idx - y_Ni[i]) ];
            }
            xy_zi_idx++;
            if (xy_zi_idx == FXDTICTRL_ZMAX) { xy_zi_idx = 0U; }
        }
        
        makeVector_batchIdx++;

        if(makeVector_batchIdx == (2*FXDTICTRL_BATCH_SIZE)){
            makeVector_batchIdx = FXDTICTRL_BATCH_SIZE;
            batchRdy = true;
        }

        if(makeVector_idx == max_intervals){
            blockProcDn = true;
            blockProcRdy = false;
            makeVector_idx = 0U;
        } else {
            makeVector_idx++;
        }

        return;
    }

    // Interpolates single data point to stepper commands.
    void FxdTiCtrl::convertToSteps(uint32_t idx){
        int32_t x_err_P = 0;
        int32_t y_err_P = 0;
        int32_t z_err_P = 0;
        int32_t e_err_P = 0;

        // Calculate target steps using truncation (fastest).
        //* <-- add a slash to enable 
        int32_t x_delta = (int32_t)(xm[idx]*planner.settings.axis_steps_per_mm[X_AXIS]) - x_steps;
        int32_t y_delta = (int32_t)(ym[idx]*planner.settings.axis_steps_per_mm[Y_AXIS]) - y_steps;
        int32_t z_delta = (int32_t)(zm[idx]*planner.settings.axis_steps_per_mm[Z_AXIS]) - z_steps;
        int32_t e_delta = (int32_t)(em[idx]*planner.settings.axis_steps_per_mm[E_AXIS]) - e_steps;
        //*/

        // Calculate target steps using rounding (slower).
        /* <-- add a slash to enable 
        float x_steps_tar = xm[idx]*planner.settings.axis_steps_per_mm[X_AXIS];
        x_steps_tar += (x_steps_tar < 0.0f) ? -0.5f : 0.5f; // May be eliminated if guaranteed positive.
        int32_t x_delta = (int32_t)(x_steps_tar) - x_steps;
        float y_steps_tar = ym[idx]*planner.settings.axis_steps_per_mm[Y_AXIS];
        y_steps_tar += (y_steps_tar < 0.0f) ? -0.5f : 0.5f;
        int32_t y_delta = (int32_t)(y_steps_tar) - y_steps;
        float z_steps_tar = zm[idx]*planner.settings.axis_steps_per_mm[Z_AXIS];
        z_steps_tar += (z_steps_tar < 0.0f) ? -0.5f : 0.5f;
        int32_t z_delta = (int32_t)(z_steps_tar) - z_steps;
        float e_steps_tar = em[idx]*planner.settings.axis_steps_per_mm[E_AXIS];
        e_steps_tar += (e_steps_tar < 0.0f) ? -0.5f : 0.5f;
        int32_t e_delta = (int32_t)(e_steps_tar) - e_steps;
        //*/

        bool any_dirChange = (  ( (x_delta > 0) && x_dirState != stepDirState_t_POS ) ||
                                ( (x_delta < 0) && x_dirState != stepDirState_t_NEG ) ||
                                ( (y_delta > 0) && y_dirState != stepDirState_t_POS ) ||
                                ( (y_delta < 0) && y_dirState != stepDirState_t_NEG ) ||
                                ( (z_delta > 0) && z_dirState != stepDirState_t_POS ) ||
                                ( (z_delta < 0) && z_dirState != stepDirState_t_NEG ) ||
                                ( (e_delta > 0) && e_dirState != stepDirState_t_POS ) ||
                                ( (e_delta < 0) && e_dirState != stepDirState_t_NEG ) );

        for (uint32_t i = 0U; i < FXDTICTRL_STEPS_PER_UNIT_TIME; i++){

            // TODO: (?) Since the *delta variables will not change,
            // the comparison may be done once before iterating at
            // expense of storage and lines of code.

            bool anyStep = false;

            stepperCmdBuff[stepperCmdBuff_produceIdx] = 0U;

            // Commands are written in the format:
            // |X_step|X_direction|Y_step|Y_direction|Z_step|Z_direction|E_step|E_direction|
            if (x_delta >= 0){
                if ( (x_err_P + x_delta) < FXDTICTRL_CTS_COMPARE_VAL ){
                    x_err_P += x_delta;
                } else {
                    x_steps++;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b11000000;
                    x_err_P += x_delta - FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            } else {
                if ( (x_err_P + x_delta) > -FXDTICTRL_CTS_COMPARE_VAL ){
                    x_err_P += x_delta;
                } else {
                    x_steps--;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b10000000;
                    x_err_P += x_delta + FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            }

            if (y_delta >= 0){
                if ( (y_err_P + y_delta) < FXDTICTRL_CTS_COMPARE_VAL ){
                    y_err_P += y_delta;
                } else {
                    y_steps++;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00110000;
                    y_err_P += y_delta - FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            } else {
                if ( (y_err_P + y_delta) > -FXDTICTRL_CTS_COMPARE_VAL ){
                    y_err_P += y_delta;
                } else {
                    y_steps--;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00100000;
                    y_err_P += y_delta + FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            }

            if (z_delta >= 0){
                if ( (z_err_P + z_delta) < FXDTICTRL_CTS_COMPARE_VAL ){
                    z_err_P += z_delta;
                } else {
                    z_steps++;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00001100;
                    z_err_P += z_delta - FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            } else {
                if ( (z_err_P + z_delta) > -FXDTICTRL_CTS_COMPARE_VAL ){
                    z_err_P += z_delta;
                } else {
                    z_steps--;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00001000;
                    z_err_P += z_delta + FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            }

            if (e_delta >= 0){
                if ( (e_err_P + e_delta) < FXDTICTRL_CTS_COMPARE_VAL ){
                    e_err_P += e_delta;
                } else {
                    e_steps++;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00000011;
                    e_err_P += e_delta - FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            } else {
                if ( (e_err_P + e_delta) > -FXDTICTRL_CTS_COMPARE_VAL ){
                    e_err_P += e_delta;
                } else {
                    e_steps--;
                    stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00000010;
                    e_err_P += e_delta + FXDTICTRL_STEPS_PER_UNIT_TIME;
                    anyStep = true;
                }
            }

            if (!anyStep){
                nextStepTicks += FXDTICTRL_MIN_TICKS;
            } else {
                stepperCmdBuff_StepRelativeTi[stepperCmdBuff_produceIdx] = nextStepTicks;

                if (any_dirChange) {
                    stepperCmdBuff_ApplyDir[stepperCmdBuff_produceIdx] = true;
                    if ( x_delta > 0 ) {
                        stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b01000000;
                        x_dirState = stepDirState_t_POS;
                    } else {
                        x_dirState = stepDirState_t_NEG;
                    }

                    if ( y_delta > 0 ) {
                        stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00010000;
                        y_dirState = stepDirState_t_POS;
                    } else {
                        y_dirState = stepDirState_t_NEG;
                    }

                    if ( z_delta > 0 ) {
                        stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00000100;
                        z_dirState = stepDirState_t_POS;
                    } else {
                        z_dirState = stepDirState_t_NEG;
                    }

                    if ( e_delta > 0 ) {
                        stepperCmdBuff[stepperCmdBuff_produceIdx] |= 0b00000001;
                        e_dirState = stepDirState_t_POS;
                    } else {
                        e_dirState = stepDirState_t_NEG;
                    }

                    any_dirChange = false;
                } else { // ...no direction change.
                    stepperCmdBuff_ApplyDir[stepperCmdBuff_produceIdx] = false;
                }

                if ( (stepperCmdBuff_produceIdx + 1U) == FXDTICTRL_STEPPERCMD_BUFF_SIZE ) {
                    stepperCmdBuff_produceIdx = 0U;
                } else {
                    stepperCmdBuff_produceIdx++;
                }

                nextStepTicks = FXDTICTRL_MIN_TICKS;
            }
        } /* for (uint32_t i = 0U; i < FXDTICTRL_STEPS_PER_UNIT_TIME; i++) */
        return;
    }
#endif
