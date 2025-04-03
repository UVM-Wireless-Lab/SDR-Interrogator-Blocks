#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2025 gr-portable_interrogator_blocks author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy as np
from gnuradio import gr

class get_peaks(gr.sync_block):
    """
    This block extracts the three peaks from an FFT of a tone AM signal, lower sideband (LSB), carrier
    (C), and upper sideband (USB). The output is the vector [LSB, C, USB].

    \nParameters:
    \n  Input Length - Length of the input FFT
    \n  LSB Index - Index of LSB in the FFT. If the signal is centered, this can be calculated as 
    int((-1*Fm)*Length/(sample_rate/Decimation))+int(Length/2), where Fm is the modulating frequency
    \n  Carrier Index - int(Length/2) for centered signal
    \n  USB Index - int((Fm)*Length/(sample_rate/Decimation))+int(Length/2) for centered signal
    \n  Search Range - To isolate each peak, the maximum value is taken in a small region around the 
    index of interest, allowing for small variations in frequency. This parameter sets the number of 
    indices to search around each peak. The search range can be equated to a bandwidth per 
    Search_BW = (sample_rate/Length)*Search_Range    
    """
    def __init__(self, inVecSize = 1024,f1_ind = 512,f2_ind=512,f3_ind = 512,searchSize = 10):
        gr.sync_block.__init__(
            self,
            name='Get AM Peaks',
            in_sig=[(np.float32,inVecSize)],#(gr.sizeof_float*1)],
            out_sig=[(np.float32,3)]    
        )

        self.f1_ind= f1_ind
        self.f2_ind= f2_ind
        self.f3_ind= f3_ind
        self.inVecSize = inVecSize
        self.searchSize = searchSize
 
    ############ Callbacks ###########
    def set_f1(self,f1_ind):
        self.f1_ind = f1_ind

    def set_f2(self,f2_ind):
        self.f2_ind = f2_ind

    def set_f3(self,f3_ind):
        self.f3_ind = f3_ind
    ###################################
    def work(self, input_items, output_items):

        P1 = np.max( input_items[0][0][(self.f1_ind-self.searchSize):(self.f1_ind+self.searchSize)])
        P2 = np.max( input_items[0][0][(self.f2_ind-self.searchSize):(self.f2_ind+self.searchSize)])
        P3 = np.max( input_items[0][0][(self.f3_ind-self.searchSize):(self.f3_ind+self.searchSize)])

        top3 =np.array([P1,P2,P3])
        output_items[0][:]= top3

        return len(output_items[0])