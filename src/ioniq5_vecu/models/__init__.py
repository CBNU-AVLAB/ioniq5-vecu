# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      __init__.py
# @brief     Actuator physical/dynamics models
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""Actuator physical/dynamics models (unit-agnostic, dbc/CAN-independent)."""
from .servo import ServoModel

__all__ = ["ServoModel"]
