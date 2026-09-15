# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      __init__.py
# @brief     vECU side-channel I/O (non-CAN)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
vECU side-channel I/O (non-CAN): the manual-control UDP channel.

Manual input travels over localhost UDP between the host console and the vECU, never on vcan0.
"""
