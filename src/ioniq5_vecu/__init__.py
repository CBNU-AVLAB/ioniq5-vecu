# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      __init__.py
# @brief     ioniq5_vecu virtual Target ECU (vECU) package
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""ioniq5_vecu - virtual Target ECU (vECU) package (containerized/headless).

This package reads only the official CAN matrix (dbc): it receives commands (RX),
updates actuator physical state, and periodically transmits state (TX). No GUI.
"""
from .bus import CanBus, PeriodicTx, load_database

__all__ = ["CanBus", "PeriodicTx", "load_database"]
