# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      __init__.py
# @brief     vECU side-channel I/O (non-CAN)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
vECU side-channel I/O (non-CAN). The manual-control UDP channel, etc.

Manual input has no corresponding message in the official CAN matrix, so it goes over
a separate localhost UDP between host console and container vECU, never on vcan0.
"""
