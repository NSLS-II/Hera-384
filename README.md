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
The IOC attempts to provide access to all of the features of the ASIC in as simple a manner as possible. The record which controls the ASIC has 53 process variables (PVs), in addition to the standard PVs offered by all EPICS records. There are other records relating to facilities for setting the sensor bias voltage and Peltier cooler current, measuring various voltages, currents and temperatures on the system, but they are standard ai and ao records.

# zDDM record PVs

Below is an alphabetic list of PVs provided by the zDDM record. This list does not include values which are part of all EPICS records.

CALF:   Name of file containing calibration constants
CHEN: 	Array of flags to enable or disable any channel         
CNT: 	1= start count frame          
CONT:	Select one-shot or auto-count       
DESC:   Description of detector                        
DLY:	Pre-count delay (single-shot mode)    
DLY1:	Pre-count delay (auto-count mode)
EBLK:	Enable internal current source to bias amplifier
EXSIZE:	X-axis size for energy spectra plot   
EYSIZE:	Y-axis size for energy spectra plot
FNAM:	Filename for fast data storage
FREQ:	Clock frequency for frame timer
FVER:	Firmware version
GAIN:	Gain
GMON:	Selector for various debug monitors                 
INTENS:	Total intensity in each channel, i.e. events.
IPADDR:	IP address of fast data interface (set by user).
LOAO:	Chose to monitor analog pulse or value of detector leakage   
MCA:	Array of spectra for each channel
MFS:	Multi-fire suppression enable and time choice
MODE:	Either time framed or continuous counting.
MONCH:	Channel to be monitored
NAME:	Record name
NCH:	NUmber of channels
NCHIPS:	Number of chips which supply those channels
NELM:	Same as NCH
OFFS:	Array of energy offsets after calibration                       
POL:	Input signal polarity   
PUEN:	Pileup detector enable
PUTR:	Pileup detector threshold trim        
RAT1:	Update frequency of display, one-shot            
RATE:	Update frequency of display, auto-count
RUNNO:	count of number of frames acquired.          
SHPT:	Shaping time of amplifier
SLP:	Array of energy calibration slope values
SPCT:	Single spectrum data
SPCTX:	X-values for single spectrum data  
TDC:	Time-to-digital converter data array
TDM:	TDC mode, time-of-arrival or time-over-threshold.
TDS:	TDC ramp time.
THRSH:	Array of pulse-height threshold values, one for each chip.
THTR:	Array of threshold trim values, one for each channel 
TP:	Time per count frame (in seconds) for one-shot mode
TP1:	Time per count frame (in seconds) for auto-count mode
TPAMP:	Amplitude of test pulses
TPCNT:	Number of test pulses to deliver
TPENB:	Internal test pulse generator enable.
TPFRQ:	Test pulse frequency.
TSEN:	Array of enable flags for test pulses (one per chanel)         
TXSIZE:	X-axis size for ToT / ToA spectra plot
TYSIZE:	Y-axis size for ToT / ToA spectra plot
VERS:	IOC code version             
