#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2025 UVM.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy as np
import pmt
from gnuradio import gr
import datetime as dt

class csb_sweep_gain_freq(gr.sync_block):
    """
Block to sweep gain and measure 4 quantities. Used with C/SB Calculator and Find Peaks blocks 
to sweep the output gain and record the  lower and upper sideband power, carrier power, and C/SB
for a tone AM signal. The block functions by publishing a message with the variable to be swept 
which should be connected to a "Message Pair to Var" block. The 4 inputs are recorded for each 
sweep value and saved as a 4xn numpy matrix and as a .csv file

    \n Inputs:
    \n Sweep - Boolean, when true, sweep is initiated
    \n Start - Starting Value
    \n Stop - Ending Value
    \n Step - Resolution of sweep
    \n sample_buffer - This sets a number of data-buffers to be discarded in between measurements
    \n Average - Number of measurements to average when saving data
    \n Path - Path to save file to
    \n OutLen - output vector length for plotting [1+(Stop-Start)/Step]   """

    def __init__(self, Sweep = False,Start = 0,Stop =10,Step = 0.5,sample_buffer=10, Average = 1,
        Prefix = "sweep_out", OutLen = 32,appendDT = True,FileName = "",fstart = 900, fstop = 1000,fstep = 100,gsweep = False,fsweep = False,freq = 1000e6):
        gr.sync_block.__init__(self,
            name ="CSB Gain Controller",
            in_sig=[np.float32,np.float32,np.float32,np.float32],
            out_sig=[(np.float32,OutLen)]
            )

        
        self.message_port_register_out(pmt.intern("gain_out"))
        self.message_port_register_out(pmt.intern("gnat"))
        self.inputs = 4   # Used in construction only to define array sizes

        self.Sweep = Sweep
        self.prevSweep = Sweep
        self.Start = Start
        self.Stop = Stop
        self.Step = Step
        self.Average = Average
        self.avgVec = np.zeros(self.Average)
        self.Path = Prefix
        self.FileName = FileName
        self.appendDT = appendDT

        self.SwpVar = self.Start
        self.state = 0
        self.xAxe = np.arange(self.Start,self.Stop+self.Step,self.Step)
        self.data = np.zeros((self.inputs,len(self.xAxe)))
        self.plotOut = np.zeros(OutLen)
        self.sample_buffer = sample_buffer
        self.counter  = sample_buffer
        self.index = 0
        
        self.fstart = fstart
        self.fstop = fstop
        self.fstep = fstep
        self.gsweep = gsweep
        self.fsweep = fsweep
        self.fvec = np.arange(self.fstart,self.fstop+self.fstep,self.fstep)
        self.fInd = 0
        self.freq = freq




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
        #self.avgVec = np.empty((self.inputs,Average))
        self.avgVec = np.zeros(Average)
    
    
    def set_buffer(self,sample_buffer):
        self.sample_buffer = sample_buffer

    def set_name(self,name):
        self.FileName = name
    
    def set_append(self,appendDT):
        self.appendDT = appendDT

    def set_fstart(self,fstart):
        self.fstart = fstart
        self.fvec = np.arange(self.fstart,self.fstop+self.fstep,self.fstep)
    
    def set_fstop(self,fstop):
        self.fstop = fstop
        self.fvec = np.arange(self.fstart,self.fstop+self.fstep,self.fstep)
    
    def set_fstep(self,fstep):
        self.fstep = fstep
        self.fvec = np.arange(self.fstart,self.fstop+self.fstep,self.fstep)

    def set_gsweep(self,gsweep):
        self.gsweep = gsweep

    def set_fsweep(self,fsweep):
        self.fsweep = fsweep

    def set_freq(self,freq):
        self.freq = freq
    
########################################################

    def work(self, input_items, output_items):


        if ((self.prevSweep^self.Sweep) and (not self.Sweep)): # Sweep has been disabled
            # Save data and exit sweep
            self.state = 4
        self.prevSweep = self.Sweep

        # if self.fsweep and self.Sweep and self.gsweep:
        #     self.message_port_pub(pmt.intern("freq_out"), pmt.cons(pmt.intern("freq"),pmt.to_pmt(self.fvec[self.fInd])))
        
        match self.state:
            case 0:
                if self.Sweep == True: 
                    #print("\nDatapoints: ",len(self.xAxe))
                    #print("Size of Output Data: ",np.shape(self.data))
                    print("\nRunning Sweep...")
                    # self.message_port_pub(pmt.intern("gain_out"), pmt.cons(pmt.intern("gain"),pmt.to_pmt(self.SwpVar)))
                    # self.counter = self.sample_buffer
                    # self.state = 2  

                    if self.fsweep:
                        if not self.gsweep:
                            self.f_data = np.zeros((self.inputs,len(self.fvec)))
                        self.state = 6
                    elif self.gsweep:
                        self.state = 1
                    else:
                        self.sweep = False
                        print("\nNo sweep variable selected. Please re-run")


            case 1: # Update sweep variable
                    #print("Update")
                    self.SwpVar = self.xAxe[self.index]
                    self.message_port_pub(pmt.intern("gain_out"), pmt.cons(pmt.intern("gain"),pmt.to_pmt(self.SwpVar)))
                    self.counter = self.sample_buffer
                    self.state = 2
                    #self.SwpVar += self.Step


            case 2: #Wait for buffer time
                #print("Wait")
                self.counter -= 1
                if self.counter < 0:
                    self.state = 5
                    self.counter = self.Average

            case 3: #Log gain value
                #print("Log")
                #avg = (np.divide(np.sum(self.avgVec,axis=1),len(self.avgVec[0,:])))
                
                self.data[:,self.index] = self.avgVec.reshape(4,1)
                self.plotOut[self.index] = self.data[0,self.index]
                
                self.index+=1

                if self.index>=len(self.xAxe):
                    self.state = 4
                    if self.fsweep:
                        self.fInd+=1
                else:
                    self.state = 1

                
            case 4: # Write data to file
                #print("Write")
                self.message_port_pub(pmt.intern("gain_out"), pmt.cons(pmt.intern("gain"),pmt.to_pmt(0))) #"Turn off" Tx (set gain to 0dB)
                

                if self.appendDT:
                    time = dt.datetime.now().strftime("%Y-%m-%d-%H%M%S_")
                else:
                   time = ""
                
                fstr = str(self.freq/1e6)+"MHz_"
                #np.save(f"{self.Path}{time}{self.FileName}.npy",out)
                if self.fsweep and not self.gsweep:
                    out = np.concatenate((self.fvec.reshape(1,-1),self.f_data),axis = 0)
                    np.savetxt(f"{self.Path}{time}{self.FileName}.csv",out.T,delimiter =',',fmt='%f')
                else:
                    out = np.concatenate((self.xAxe.reshape(1,-1),self.data),axis=0)
                    np.savetxt(f"{self.Path}{time}{fstr}{self.FileName}.csv",out.T,delimiter =',',fmt='%f')

                print("Data Saved! \n")

                self.index = 0
                if self.fsweep:
                    if self.fInd>=len(self.fvec):
                        self.Sweep = False
                        self.prevSweep = False
                        self.state = 0
                        self.fInd = 0   
                    else:
                        self.state = 6                     
                
                #self.SwpVar = self.Start
                else:
                    print("Reset! Ready to sweep again\n")
                    self.Sweep = False
                    self.prevSweep = False
                    self.state = 0
    


            case 5: #Average values
                #print("Average")
                self.counter += -1
                # Average values in input buffers
                self.avgVec[0]= (np.sum(input_items[0])/len(input_items[0]))/self.Average
                self.avgVec[1]= (np.sum(input_items[1])/len(input_items[1]))/self.Average
                self.avgVec[2]= (np.sum(input_items[2])/len(input_items[2]))/self.Average
                self.avgVec[3]= (np.sum(input_items[3])/len(input_items[3]))/self.Average

                # in1 = np.sum(input_items[1])/len(input_items[1])
                # in2 = np.sum(input_items[2])/len(input_items[2])
                # in3 = np.sum(input_items[3])/len(input_items[3])
                # self.avgVec[:,self.counter] = np.array([[in0],[in1],[in2],[in3]])[:,0]

                if self.counter < 0:
                    if not self.gsweep:
                        self.state = 7
                    else:
                        self.state = 3

            case 6: # Update Freq
                
                f = self.fvec[self.fInd]*1e6
                self.message_port_pub(pmt.intern("gain_out"), pmt.cons(pmt.intern("freq"),pmt.to_pmt(f)))
               
                if self.gsweep:
                    self.state = 1
                else:
                    self.counter = self.sample_buffer
                    self.state = 2

            case 7: #log f value
                self.fdata[:,self.fInd] = self.avgVec.reshape(4,1)
                self.fInd += 1
                if self.fInd >= len(self.fvec):
                    self.state = 4
                else:
                    self.state = 6

        output_items[0][:] = self.plotOut # Output Vector
        return len(input_items[0])