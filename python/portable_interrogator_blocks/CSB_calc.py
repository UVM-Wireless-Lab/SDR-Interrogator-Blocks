#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2025 gr-portable_interrogator_blocks author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy as np
from gnuradio import gr

class CSB_calc(gr.sync_block):
    """
    Block to calculate the carrier to sideband ratio for a tone AM signal. The block takes a fixed
    length vector of input powers [LSB, C, USB] and outputs the C/SB ratio, as well as LSB, C, and USB values. 
    """
    def __init__(self):
        gr.sync_block.__init__(self,
            name="CSB Calculator",
            in_sig=[(np.float32,3)],
            out_sig=[np.float32,np.float32,np.float32,np.float32]
        )

    def work(self, input_items, output_items):
        LSB = input_items[0][0][0]
        C = input_items[0][0][1]
        RSB = input_items[0][0][2]

        SB_avg = (LSB+RSB)/2
        CSB = C-SB_avg

        output_items[0][:] = CSB
        output_items[1][:] = LSB
        output_items[2][:] = C
        output_items[3][:] = RSB

        return len(output_items[0])
