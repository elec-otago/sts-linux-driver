''' This is an example code for a simple acquisition of a spectrum using the
    driver written for operation of the STS spectrometer made by Ocean Optics.
    This will only work for 1 spectrometer that is plugged in at a time, there
    are previsions in the driver for multiple. (More examples may follow)

    This STS-Driver is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    It is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the software.  If not, see <http://www.gnu.org/licenses/>.

    Last updated: 27/08/2014

    Author: Matthew West
        Department of Physics,
        University of Otago
        Dunedin, New Zealand

    Email: wesma651@student.otago.ac.nz
'''

import numpy as np
import matplotlib.pyplot as plt
import OceanOptics
import datetime
import argparse
import sys, os, errno
from OceanOptics import sts_utils as utilities
from scipy import interpolate

if __name__ == '__main__':
    
    #Initialise the spectrometer driver
    spec = OceanOptics.STSVIS()

    # serial = spec.get_serial()

    #Set the desired integration time in seconds
    integration_seconds = 0.5

    #Set some basic values for workings within the device
    spec.set_boxcar(0)
    spec.set_scans_to_avg(3)
    spec.set_integration_time(integration_seconds*1e6, 1)

    #Find the wavelength bins
    bins = utilities.calculate_wavlengths(spec)

    print "Taking a Dark Spectrum, please block all light into the device"
    raw_input("Press Enter to Continue...")
    dark_spec = spec.get_corrected_spectrum(1)

    print 'Now taking source data'
    raw_input("Press Enter to continue...")
    raw_data = spec.get_corrected_spectrum(1)


    plt.plot(raw_data - dark_spec)
    plt.show()

    ''' One can also load a calibration file and then simply multiply what is
        being plotted here by this (and divide by the integration time (in 
        seconds), the collection area (cm^2) and the wavelength spread (nm).
        There are functions in the utilities file that do these things.
    '''



