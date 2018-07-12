Microchip LoRaWAN Stack Release Notes
=====================================

This document provides information on release details of Microchip LoRaWAN stack.


Release Version :-
------------------
	* MS4_E_0
	
Release Date :-
---------------
	* 2018-02-05
	
LoRaWAN Stack Spec version :-
-----------------------------
	* 1.0.2
	
Supported LoRaWAN ED Class :-
-----------------------------
	* Class A
	* Class C
	
Package Contents :-
--------------------

	* enddevice_demo_multiband_samr34_xpro.zip
	* MLS Quick Start Guide.pdf
	* SAMR34 Setup Guide For Microchip LoRaWAN Stack.pdf
		
Software Requirements :-
-------------------------
	* Atmel Studio 7.0.1417  - https://www.atmel.com/Microsite/atmel-studio
	* Atmel® Software Framework (ASF) 3.34.1.745
	* Atmel.SAMR34_DFP-0.1.7.atpack


Changes in MS4_E_0 on top of MS3_E_0 LoRaWAN stack :-
----------------------------------------------------
    1.  Multi-band feature support added - Dynamic selection of different region band during run-time.
	2.  System STANDBY sleep enhancements - Low Sleep current achieved with MLS. 
	3.	Bug fixes from previous releases.


Known Issues:-
--------------
	1. Issue identified while receiving MAXMIMUM Payload on DR5 in Down-link data. Current stack supports only 230 bytes of Down-link data.

General Notes:
--------------
	* With the support of Multi-band feature, User can select any band in run-time and also switch to different band in run-time. For more details, please refer section 10.2 of MLS_Quick_Start_Guide.pdf
	* LBT is not end-to-end tested(with a gateway).
	* FSK mode is not tested in MAC level.
