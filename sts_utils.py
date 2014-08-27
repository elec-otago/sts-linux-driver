''' STS functions module file which contains the commonly used functions for
    using the STS driver we have written.

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

    Author: Matthew West
    Department of Physics,
    University of Otago
    Dunedin, New Zealand

    Email: wesma651@student.otago.ac.nz
'''

from STS import STSVIS
import struct
import usb.core as core
import time
import numpy as np
from scipy import interpolate
from scipy import optimize
import datetime
import sys, os, errno

import matplotlib.pyplot as plt

def mkdir_p(path):
    ''' This function checks to see if path exists in the OS directory
        structure and if not, it creates the folders neccessary.
    '''
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise
        
def find_bin_factor(bins):
    ''' This function looks at the centre of the wavelength bins returned from
        the spectrometer and calulates the width of those bins for unit
        conversion to get the intensity of light.
    '''

    bin_factor = np.zeros(1024)
    bin_factor[0] = bins[1] - bins[0]
    bin_factor[1:1023] = (bins[2:1024] - bins[0:1022]) / 2
    bin_factor[1023] = bins[1023] - bins[1022]
    return bin_factor

def get_multiplication(serial, bin_factor, calibration, integration_sec):
    ''' This function returns the coefficients to multiply the 
        counts - baseline value by to get the intensity.
    '''
    core_cm = 0.04
    area = np.pi*((core_cm/2)**2)
    return calibration/integration_sec/area/bin_factor

def calculate_wavlengths(spec):
    ''' This function asks the spectrometer for the wavelength that is the
        centre of the bins and returns an array of those wavelengths.
    '''
    coef1 = spec.get_wav_coeff(0)
    coef2 = spec.get_wav_coeff(1)
    coef3 = spec.get_wav_coeff(2)
    coef4 = spec.get_wav_coeff(3)
    wave = np.zeros(1024)
    for index in range(1024):
        wave[index] = coef1 + coef2*index + coef3*(index**2) + coef4*(index**3)
    return wave

def get_non_linear_correction(spec):
    ''' This function gets the array of coefficients to be multiplied by the
        result of the scan to take into account the non linearity of the
        device.
    '''
    coeff = np.zeros(8)
    for aa in range(8):
        coeff[aa] = spec.get_nonlin_coeff(aa)

    return coeff

def do_non_lin(data, coeff, dark_spec, integration_sec):
    ''' This function does the non linearity correction by multiplying by
        the coefficients in the manner laid out in the STS spec sheet
        provided by Ocean Optics.
    '''
    step_1 = data - dark_spec*integration_sec
    poly = coeff[0] + coeff[1]*step_1 + coeff[2]*(step_1**2) + \
        coeff[3]*(step_1**3) + coeff[4]*(step_1**4) + \
        coeff[5]*(step_1**5) + coeff[6]*(step_1**6) + coeff[7]*(step_1**7)
    lin_data = step_1/poly
    return lin_data

def get_lamp_data(bins):
    ''' This is a function used in calibration which loads the lamp file for
        the calibrated source.
    '''
    actual = np.loadtxt('lmp.LMP')
    wave_limited = actual[:,0]
    rad_limited = actual[:,1]
    f = interpolate.interp1d(wave_limited, rad_limited, kind='linear')
    rad = f(bins)
    return rad

def do_collection(spec, coefficients ,integration_sec, length="", averaging=1):
    ''' This function does the data collection from the spectrometer using a
        sum over a loop method to improve the signal to noise via increased
        integration time, without the device saturating.
    '''
    serial = spec.get_serial()
    dark_spec = np.loadtxt('../{0}/{0}_{1}_dark.txt'.format(serial,length))
    scan_data = np.zeros(1024)
    spec.set_integration_time(integration_sec*1e6, 1)

    for scan in range(int(averaging)):

        raw_data = spec.get_corrected_spectrum(1)
        data = do_non_lin(raw_data, coefficients, dark_spec, integration_sec)
        scan_data += data
    
    return scan_data

def get_time_stamp(base_path):
    ''' This function create a time stamp for naming files which contain data
        collected by the spectrometer.
    '''
    # Time stamp information for naming files
    ts = datetime.datetime.utcnow()

    # Create a meaningful directory structure to organize recorded data
    p = base_path + '/' + str(ts.year) +'/' + str(ts.month) + '/' +\
        str(ts.day) + '/'
    mkdir_p(p)
    # Call time stamp again (the directory name will not have changed,
    # but the timestamp will be more accurate)
    ts = datetime.datetime.utcnow()
    return ts, p

    class OceanOpticsError(Exception):
    pass