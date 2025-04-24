# SDR Interrogator Blocks
## Overview
This repository contains a GNU radio module with the Python blocks used in the portable interrogator developed at University of Vermont for the Burned Area Monitoring System (BAMS) project. The blocks in this repository are used to conduct automated measurements of device conversion loss and measure the carrier to sideband distortion for an amplitude modulated interrogation signal. To see implementations in GNU radio flowgraphs check [SDR-Interrogator-Flowgraphs](https://github.com/UVM-Wireless-Lab/SDR-Interrogator-Flowgraphs) 

## Structure
There are four blocks defined in this repository:
- Get AM Peaks  -  Block that takes an FFT as input and measures the three peaks of an AM signal
- CSB Calculator  -  Takes a vector of AM peaks (i.e. from "Get AM Peaks") and calculates the C/SB ratio
- Sweep Controller  -  Controlling block to sweep the transmit power and record four values. Output is a vector of measured values at the first input.
- CL Sweep Controller  -  Very similar to Sweep controller, but with only one input and with an additional parameter (Transmit Gain). Records the receive power and calculates the relative conversion loss (Transmit Gain - Rx Power). The output is a vector of relative conversion loss
  
## Notes
- When using the sweep controllers with a GNU Radio QT GUI Vector sink for plotting, the "Output Length" parameter must be set to a power of two for the program to run efficiently. This length is only used for plotting, so it can take any value greater than the number of datapoints.
- The Get AM Peaks takes the expected peak locations (as indices of the FFT) as inputs. See the flowgraphs for an example.

## Build Instructions
Detailed instructions on working with OOT modules can be found at https://wiki.gnuradio.org/index.php/OutOfTreeModules. To build the module follow the following instructions. You will need to have cmake installed.
- Clone or copy the repository
- Create a build directory
- Generate and build cmake files
- Update GNU radio block definitions
  
To do this, execute the following commands:

`cd SDR-Interrogator-Blocks` Ensure you are in the module directory. Replace SDR-Interrogator-Blocks with your exact module name/path

`mkdir build`

`cd build`

`cmake ..`

`make`

`sudo make install`

`sudo ldconfig`

After building, for the changes to appear in GNU Radio Companion you will either need to relaunch GRC or refresh using the refresh button

## Contact
Please reach out to sfought@uvm.edu with any questions
