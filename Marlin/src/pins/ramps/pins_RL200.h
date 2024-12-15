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

/**
 * Rapide Lite 200 v1 (RUMBA clone) pin assignments. Has slightly different assignment for
 * extruder motors due to dual Z motors. Pinout therefore based on pins_RUMBA.h.
 * ATmega2560
 */

#define BOARD_INFO_NAME "RL200"
#define DEFAULT_MACHINE_NAME "Rapide Lite 200"

#define E0_STEP_PIN                           26  // (RUMBA E1 pins)
#define E0_DIR_PIN                            25
#define E0_ENABLE_PIN                         27

#define E1_STEP_PIN                           29  // (RUMBA E2 pins)
#define E1_DIR_PIN                            28
#define E1_ENABLE_PIN                         39

#define Z2_STEP_PIN                           23  // (RUMBA E0 pins)
#define Z2_DIR_PIN                            22
#define Z2_ENABLE_PIN                         24

#include "pins_RUMBA.h"
