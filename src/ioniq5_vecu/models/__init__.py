# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      __init__.py
# @brief     Actuator physical/dynamics models
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Actuator physical/dynamics models (unit-agnostic, dbc/CAN-independent)."""
from .servo import ServoModel

__all__ = ["ServoModel"]
