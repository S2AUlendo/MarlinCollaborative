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

#include "../gcode.h"
#include "../../module/fixed_time_ctrl.h"


#ifdef FXDTICTRL

    void GcodeSuite::M950() {
        // Parse mode parameter.
        if (parser.seenval('M')) {
            const uint8_t val = parser.value_byte();
            switch (val) {
                case 0U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_DISABLED;
                    SERIAL_ECHOLN("Fixed time controller disabled.");
                    break;
                case 1U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_ENABLED;
                    SERIAL_ECHOLN("Fixed time controller enabled.");
                    fxdTiCtrl.reset();
                    break;
                // case 2U:
                //     fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_ULENDO_FBS;
                //     SERIAL_ECHOLN("Fixed time controller enabled w/ Ulendo FBS.");
                //     break;
                case 10U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_ZV;
                    SERIAL_ECHOLN("Fixed time controller enabled w/ ZV shaping.");
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.updateShapingA( FXDTICTRL_SHAPING_ZETA, FXDTICTRL_SHAPING_V_TOL );
                    fxdTiCtrl.reset();
                    break;
                case 11U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_ZVD;
                    SERIAL_ECHOLN("Fixed time controller enabled w/ ZVD shaping.");
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.updateShapingA( FXDTICTRL_SHAPING_ZETA, FXDTICTRL_SHAPING_V_TOL );
                    fxdTiCtrl.reset();
                    break;
                case 12U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_EI;
                    SERIAL_ECHOLN("Fixed time controller enabled w/ EI shaping.");
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.updateShapingA( FXDTICTRL_SHAPING_ZETA, FXDTICTRL_SHAPING_V_TOL );
                    fxdTiCtrl.reset();
                    break;
                case 13U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_2HEI;
                    SERIAL_ECHOLN("Fixed time controller enabled w/ 2 Hump EI shaping.");
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.updateShapingA( FXDTICTRL_SHAPING_ZETA, FXDTICTRL_SHAPING_V_TOL );
                    fxdTiCtrl.reset();
                    break;
                case 14U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_3HEI;
                    SERIAL_ECHOLN("Fixed time controller enabled w/ 3 Hump EI shaping.");
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.updateShapingA( FXDTICTRL_SHAPING_ZETA, FXDTICTRL_SHAPING_V_TOL );
                    fxdTiCtrl.reset();
                    break;
                case 15U:
                    fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_MZV;
                    SERIAL_ECHOLN("Fixed time controller enabled w/ MZV shaping.");
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.updateShapingA( FXDTICTRL_SHAPING_ZETA, FXDTICTRL_SHAPING_V_TOL );
                    fxdTiCtrl.reset();
                    break;
                // case 20U:
                //     fxdTiCtrl.cfg_mode = fxdTiCtrlMode_t_DISCTF;
                //     SERIAL_ECHOLN("Fixed time controller enabled w/ discrete transfer functions.");
                //     break;
                default:
                    SERIAL_ECHOLN("Invalid control mode [M] value.");
            }
        }

        // Parse pressure control (linear advance) parameter.
        if (parser.seenval('P')) {
            const uint8_t val = parser.value_byte();
            if (val == 0U || val == 1U) {
                fxdTiCtrl.cfg_linearAdvEna = bool(val);
                if (fxdTiCtrl.cfg_linearAdvEna){
                    SERIAL_ECHOLN("Pressure control: linear advance enabled.");
                } else {
                    SERIAL_ECHOLN("Pressure control: linear advance disabled.");
                }
            } else {
                SERIAL_ECHOLN("Invalid pressure control [P] value.");
            }
        }

        // Parse pressure control (linear advance) gain parameter.
        if (parser.seenval('K')) {
            const float val = parser.value_float();
            if (fxdTiCtrl.cfg_linearAdvEna && val >= 0.0f){
                fxdTiCtrl.cfg_linearAdvK = val;
                SERIAL_ECHO("Pressure control: linear advance gain set to: ");
                SERIAL_ECHO_F(fxdTiCtrl.cfg_linearAdvK, 5);
                SERIAL_ECHO(".\n");
            } else if (fxdTiCtrl.cfg_linearAdvEna) { // Value out of range.
                SERIAL_ECHOLN("Pressure control: linear advance gain out of range.");
            } else { // linearAdvEna is false.
                SERIAL_ECHOLN("Pressure control: cannot set linear advance gain because it is disabled.");
            }
        }

        // Parse dynamic frequency mode parameter.
        if (parser.seenval('D')) {
            const uint8_t val = parser.value_byte();
            if (fxdTiCtrl.cfg_mode >= 10U && fxdTiCtrl.cfg_mode < 20U) {
                switch (val){
                    case 0U:
                        fxdTiCtrl.cfg_dynFreqMode = dynFreqMode_t_DISABLED;
                        SERIAL_ECHOLN("Dynamic frequency mode disabled.");
                        break;
                    case 1U:
                        fxdTiCtrl.cfg_dynFreqMode = dynFreqMode_t_Z_POSN_BASED;
                        SERIAL_ECHOLN("Dynamic frequency mode enabled: Z position based.");
                        break;
                    case 2U:
                        fxdTiCtrl.cfg_dynFreqMode = dynFreqMode_t_MASS_BASED;
                        SERIAL_ECHOLN("Dynamic frequency mode enabled: mass based.");
                        break;
                    default:
                        SERIAL_ECHOLN("Invalid dynamic frequency mode [D] value.");
                        break;
                }
            } else {
                SERIAL_ECHOLN("Dynamic frequency mode: cannot enable because a compatible shaper is not selected.");
            }
        }

        // Parse frequency parameter (X axis).
        if (parser.seenval('A')) {
            const float val = parser.value_float();

            bool modeUsesFrequency = (fxdTiCtrl.cfg_mode >= 10U && fxdTiCtrl.cfg_mode < 20U);
            bool frequencyInRange = ( (val >= FXDTICTRL_MIN_SHAPE_FREQ) && (val <= (FXDTICTRL_FS/2)) );
            // TODO: Frequency minimum is dependent on the shaper used; the above check isn't always correct.

            if (modeUsesFrequency) {
                if (frequencyInRange){
                    fxdTiCtrl.cfg_baseFreq[0] = val;
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.reset();
                    if (fxdTiCtrl.cfg_dynFreqMode){ SERIAL_ECHO("Compensator base dynamic frequency (X/A axis) set to:"); }
                    else { SERIAL_ECHO("Compensator static frequency (X/A axis) set to: "); }
                    SERIAL_ECHO_F( fxdTiCtrl.cfg_baseFreq[0], 2 );
                    SERIAL_ECHO(".\n");
                } else { // Frequency out of range.
                    SERIAL_ECHOLN("Invalid frequency [A] value.");
                }
            } else { // Mode doesn't use frequency.
                SERIAL_ECHOLN("Cannot set frequency [A] value because a compatible mode is not selected.");
            }
        }

        // Parse frequency parameter (Y axis).
        if (parser.seenval('B')) {
            const float val = parser.value_float();

            bool modeUsesFrequency = (fxdTiCtrl.cfg_mode >= 10U && fxdTiCtrl.cfg_mode < 20U);
            bool frequencyInRange = ( (val >= FXDTICTRL_MIN_SHAPE_FREQ) && (val <= (FXDTICTRL_FS/2)) );

            if (modeUsesFrequency) {
                if (frequencyInRange){
                    fxdTiCtrl.cfg_baseFreq[1] = val;
                    fxdTiCtrl.updateShapingN( fxdTiCtrl.cfg_baseFreq[0], fxdTiCtrl.cfg_baseFreq[1], FXDTICTRL_SHAPING_ZETA );
                    fxdTiCtrl.reset();
                    if (fxdTiCtrl.cfg_dynFreqMode){ SERIAL_ECHO("Compensator base dynamic frequency (Y/B axis) set to:"); }
                    else { SERIAL_ECHO("Compensator static frequency (Y/B axis) set to: "); }
                    SERIAL_ECHO_F( fxdTiCtrl.cfg_baseFreq[1], 2 );
                    SERIAL_ECHO(".\n");
                } else { // Frequency out of range.
                    SERIAL_ECHOLN("Invalid frequency [B] value.");
                }
            } else { // Mode doesn't use frequency.
                SERIAL_ECHOLN("Cannot set frequency [B] value because a compatible mode is not selected.");
            }
        }

        // Parse frequency scaling parameter (X axis).
        if (parser.seenval('G')) {
            const float val = parser.value_float();

            bool modeUsesDynFreq = (fxdTiCtrl.cfg_dynFreqMode == dynFreqMode_t_Z_POSN_BASED || \
                                    fxdTiCtrl.cfg_dynFreqMode == dynFreqMode_t_MASS_BASED );
            
            if (modeUsesDynFreq) {
                fxdTiCtrl.cfg_dynFreqK[0] = val;
                SERIAL_ECHO("Frequency scaling (X/A axis) set to: ");
                SERIAL_ECHO_F(fxdTiCtrl.cfg_dynFreqK[0], 8);
                SERIAL_ECHO(".\n");
            } else {
                SERIAL_ECHOLN("Cannot set frequency scaling [G] value because a compatible mode is not selected.");
            }
        }

        // Parse frequency scaling parameter (Y axis).
        if (parser.seenval('H')) {
            const float val = parser.value_float();

            bool modeUsesDynFreq = (fxdTiCtrl.cfg_dynFreqMode == dynFreqMode_t_Z_POSN_BASED || \
                                    fxdTiCtrl.cfg_dynFreqMode == dynFreqMode_t_MASS_BASED );
            
            if (modeUsesDynFreq) {
                fxdTiCtrl.cfg_dynFreqK[1] = val;
                SERIAL_ECHO("Frequency scaling (Y/B axis) set to: ");
                SERIAL_ECHO_F(fxdTiCtrl.cfg_dynFreqK[1], 8);
                SERIAL_ECHO(".\n");
            } else {
                SERIAL_ECHOLN("Cannot set frequency scaling [H] value because a compatible mode is not selected.");
            }
        }


        return;
    }

#endif