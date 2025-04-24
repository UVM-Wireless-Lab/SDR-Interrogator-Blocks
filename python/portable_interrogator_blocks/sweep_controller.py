#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# 
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy as np
import pmt
from gnuradio import gr
import datetime as dt

class sweep_controller(gr.sync_block):
    """
Block to sweep a variable and measure up to 4 quantities. Designed for use with C/SB Calculator and Find Peaks blocks 
to sweep the output gain and record the  lower and upper sideband power, carrier power, and C/SB
for a tone AM signal. The block functions by publishing a message with the variable to be swept 
which should be connected to a "Message Pair to Var" block. The 4 inputs are recorded for each 
sweep value and saved to a .csv file. The first and second inputs are also collected into vectors which are streamed at
the block ouputs. These outputs facilitate plotting with the QT GUI Vector Sink native to GNU Radio.

    \n Inputs:
    \n Sweep - Boolean, when true, sweep is initiated
    \n Start - Starting Value
    \n Stop - Ending Value
    \n Step - Resolution of sweep
    \n Sample Buffer - This sets a number of data-buffers to be discarded in between measurements
    \n Averages - Number of measurements to average when saving data
    \n Path - Folder to save file to
    \n Output Length - output vector length for plotting [1+(Stop-Start)/Step]. Should be padded to the next power of two for efficiency
    \n File Name - Name of file to save
    \n Add Time - Boolean to append date and time to file name (Y-m-d-HMS). WARNING: If false file may be overwritten if the 
    file name is not changed between sweeps"""

    def __init__(self, Sweep = False,Start = 0,Stop =10,Step = 0.5,sample_buffer=10, Average = 1,
        Prefix = "sweep_out", OutLen = 32,FileName = "power_sweep",appendDT = True):
        gr.sync_block.__init__(self,
            name ="CSB Gain Controller",
            in_sig=[np.float32,np.float32,np.float32,np.float32],
            out_sig=[(np.float32,OutLen)]
            )

        self.message_port_register_out(pmt.intern('param_out'))

        self.inputs = 4   # Used in construction only to define array sizes

        self.Sweep = Sweep
        self.prevSweep = Sweep
        self.Start = Start
        self.Stop = Stop
        self.Step = Step
        self.Average = Average
        self.avgVec = np.empty((self.inputs,Average))
        self.Path = Prefix
        self.FileName = FileName
        self.appendDT = appendDT

        self.state = 0
        self.xAxe = np.arange(self.Start,self.Stop+self.Step,self.Step)
        self.data = np.zeros((self.inputs,len(self.xAxe)))
        self.plotOut = np.zeros(OutLen)
        self.sample_buffer = sample_buffer
        self.counter  = sample_buffer
        self.index = 0


# Callbacks: 
####################################################
    def set_sweep(self, Sweep):
        self.Sweep = Sweep

    def set_path(self, Path):
        self.Path = Path

    def set_start(self, Start):
        self.Start = Start
    
    def set_average(self,Average):
        self.Average = Average
        self.avgVec = np.empty((self.inputs,Average))
    
    
    def set_buffer(self,sample_buffer):
        self.sample_buffer = sample_buffer

    def set_name(self,Filename):
        self.FileName = Filename
    
    def set_append(self,appendDT):
        self.appendDT = appendDT


    
########################################################

    def work(self, input_items, output_items):


        if ((self.prevSweep^self.Sweep) and (not self.Sweep)): # Sweep has been disabled, save data and exit
            self.state = 4

        self.prevSweep = self.Sweep
        match self.state:
            case 0:
                if self.Sweep == True: #Log initial value
                    print("\nDatapoints: ",len(self.xAxe))
                    print("\nRunning Sweep...")
                    self.state = 1  


            case 1: # Update sweep variable
                    self.message_port_pub(pmt.intern("param_out"), pmt.cons(pmt.intern("gain"),pmt.to_pmt(self.xAxe[self.index])))
                    self.counter = self.sample_buffer
                    self.state = 2


            case 2: #Wait for buffer time
                self.counter -= 1
                if self.counter < 0:
                    self.state = 5
                    self.counter = self.Average


            case 5: #Average values
                self.counter += -1
                # For each input, average all the items in the data buffers
                in0 = np.sum(input_items[0])/len(input_items[0]) 
                in1 = np.sum(input_items[1])/len(input_items[1])
                in2 = np.sum(input_items[2])/len(input_items[2])
                in3 = np.sum(input_items[3])/len(input_items[3])
                self.avgVec[:,self.counter] = np.array([[in0],[in1],[in2],[in3]])[:,0]

                if self.counter < 0:
                    self.state = 3

            case 3: #Log value
                avg = (np.divide(np.sum(self.avgVec,axis=1),len(self.avgVec[0,:])))
                
                self.data[:,self.index] = avg.reshape(4,1)[:,0]
                self.plotOut[self.index] = self.data[0,self.index]
                
                self.index+=1

                if self.index>=len(self.xAxe):
                    self.state = 4
                else:
                    self.state = 1

                
            case 4: # Write data to file
                self.message_port_pub(pmt.intern("param_out"), pmt.cons(pmt.intern("gain"),pmt.to_pmt(0))) #"Turn off" Tx (set gain to 0dB)
                
                out = np.concatenate((self.xAxe.reshape(1,-1),self.data),axis=0)

                if self.appendDT:
                    time = dt.datetime.now().strftime("%Y-%m-%d-%H%M%S")+("" if self.FileName=="" else "_")
                else:
                   time = ""
                
                print(f"{self.Path}{time}{self.FileName}")


                np.savetxt(f"{self.Path}{time}{self.FileName}.csv",out.T,delimiter =',',fmt='%f')

                print("Data Saved! Sweep Complete \n")

                self.index = 0
                print("Reset! Ready to sweep again\n")
                self.Sweep = False
                self.prevSweep = False
                self.state = 0
                
        
        output_items[0][:] = self.plotOut # Output Vector
        return len(input_items[0])