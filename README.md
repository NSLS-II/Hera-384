# Hera-384
EPICS code for the new multi-element detectors at NSLS-II

# Introduction
This code provides EPICS support for a range of detectors developed at NSLS-II, based on a low-noise ASIC (MARS) designed by BNL's Instrumentation Division. The ASIC has 32 channels, each providing a low-noise charge-sensitive preamplifier, a shaping amplifier and peak-detection and timing circuitry. A set of up to 12 of these ASICs can be assembled to instrument a multi-element x-ray detector based on silicon (either simple PIN pad diode arrays or SDD arrays) or germanium strip or pad arrays. We have made 64-, 96-, 192- and 384-element versions. This code supports all of them.
The key feature of all of these systems is that they are spectroscopic. Each of their many channels is a fully instrumented spectrometer, producing x-ray spectra with good energy resolution. The details depend on the sensor type and configuration.

# EPICS IOC
The code provides full control of all of the many features provided by the ASIC underlying the system. In order to provide fast and flexible controls, the code is implemented as an embedded IOC, residing in the hard cpu cores of a Zynq FPGA system. THis system makes it possible to have close control of the hardware, and straightforward programming under the Linux system running on the dual-core Arm CPU. Programs and data are kept on a micro-SD card, and a complete Debian Linux system is installed, together with an EPICs distribution.

# ASIC
In order to fully understand the system operation, it is necessary to describe the underlying ASIC which forms the core of the detector system. Each ASIC has 32 channels. It can handle positive or negative charge signals, and has a range of gains, from 12.5keV full scale to 75keV (silicon) full scale, covering the most useful x-ray energies in silicon or germanium sensors. It has four shaping times, from 250 ns to 2 us. THe ASIC also has a built-in pileup rejection system which can be optionally enabled, and a test pulse generator input with a DAC to control its amplitude.
The operation proceeds as follows:
An x-ray is absorbed by the sensor and a charge pulse is delivered to the ASIC input. THis pulse is converted to a voltage step by a charge-sensitive preamplifier, and then filtered to provide a pseudo-Gaussian voltage pulse whose peak value is proportional to the x-ray energy. When the pulse voltage crosses a threshold value, a peak-detector circuit is triggered, and when the pulse changes slope (i.e. at the peak), that value is stored in a capacitor, and a signal sent to the DAQ system that a photon has been detected. The external DAQ system converts this voltage into a digital value and passes it on. The ASIC also has a time-to-analog converter which can be configured in two ways: either time-of-arrival (ToA)or time-over threshold (ToT). Both modes can be useful, depending on the nature of the measurement.

# IOC
The IOC attempts to provide access to all of the features of the ASIC in as simple a manner as possible. The record which controls the ASIC has 70 process variables (PVs), in addition to the standard PVs offered by all EPICS records. There are other records relating to facilities for setting the sensor bias voltage and Peltier cooler current, measuring various voltages, currents and temperatures on the system, but they are standard ai and ao records.

# zDDM record PVs

Below is an alphabetic list of PVs provided by the zDDM record. THis list does not include values which are part of any EPICS record.

CHAN: Which channel within chip CHIP is being monitored             
CHEN: Array of enable flags for each channel in the detector
CHIP: Which chip contains the channel being monitored            
CNT:  Flag for counting activity          
CONT: Flag to indicate single-shot or repeated count cycles       
DESC: Description of IOC.             
DLY: 0              
DTYP: NSLS detector 
EBLK: Enable internal leakage current generator to ensure ASIC input is properly biased.           
EGU: ENgineering units:counts         
EXSIZE: 4096 X-axis size for energy array (MCA)       
EYSIZE: 384  Y-axis size for energy array. Equals nmber of channels        
FNAM: File name for data collection              
FREQ: 1000000: Internal timebase clock frequency       
FVER: Firmware version for FPGA functions             
GAIN: 75keV         
GMON: Global monitor function switch.           
LOAO: Pulse: Choose to monitor pulse shape or leakage current from ASIC.    
MCA: Array containing 384 spectra, each with 4096 elements.          
MFS: Off: Multiple firing control for threshold discriminator.            
MODE: Framing: Choose either framing operation or continuous operation.       
MONCH: Channel to be monitored 
NAME: det1          
NCH: 384: number of channels            
NCHIPS: 12: number of chips
NELM: 384           
OFFS: Energy calibration array of offsets
PPN: (nil)          
PPNR: (nil)         
PR1: 1000000: timer value in clock ticks        
PUEN: Disable: enable pileup rejection       
PUTF: 0             
PUTR: Trim DAC for pileup rejection        
RATE: 2: display update frequency, Hz.             
RUNNO: 0: Running counter of frames
SHPT: 0.5us: Shaping time         
SLP: Energy calibration slope
SPCT: Spectrum of currently monitored channel
SPCTX: Calibrated X-axis for plot
TDC: Spectrum of timing signal          
TDM: Timing signal function, ToT or ToA          
TDS: Slope of timing ramp
THRSH: Per-chip threshold 
THTR: Per-channel threshold trim
TP: 1: Counting time per frame               
TP1: 1: Auto-count time
TPAMP: 0: Test pulse generator amplitude
TPCNT: 0: Test pulse generator number of pulses to generate
TPENB: Off: Test pulse enable
TPFRQ: 0: Test pulse frequency
TSEN: Per-channel test pulse enable         
TXSIZE: 1024 : X-axis size for timing signal display       
TYSIZE: 384 : Y-axis size for timing signla display
VERS: 1 IOC code version             
