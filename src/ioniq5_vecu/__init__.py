# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      __init__.py
# @brief     ioniq5_vecu virtual Target ECU (vECU) package
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""ioniq5_vecu — virtual Target ECU (vECU) package.

Receives command frames defined in the CAN matrix (dbc), updates the actuator
models and transmits the status frames periodically.
"""

from .bus import CanBus, PeriodicTx, load_database

__all__ = ["CanBus", "PeriodicTx", "load_database"]
