# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      __init__.py
# @brief     Target ECU implementations (role 1)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Target ECU implementations. Bridge dbc signals <-> physical models and transmit periodically.

Import each ECU directly (e.g. `from ioniq5_vecu.ecus.steering import SteeringEcu`).
We don't eagerly re-export here: when running `python -m ioniq5_vecu.ecus.steering`,
importing the module from the package __init__ first would raise a RuntimeWarning
(double import).
"""
