''' STS functions module file which contains the commonly used functions for
    using the STS driver we have written.

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
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise
        
def find_bin_factor(bins):

    bin_factor = np.zeros(1024)
    bin_factor[0] = bins[1] - bins[0]
    bin_factor[1:1023] = (bins[2:1024] - bins[0:1022]) / 2
    bin_factor[1023] = bins[1023] - bins[1022]
    return bin_factor

def get_multiplication(serial, bin_factor, calibration, integration_sec):
    core_cm = 0.04
    area = np.pi*((core_cm/2)**2)
    return calibration/integration_sec/area/bin_factor

def calculate_wavlengths(spec):
    coef1 = spec.get_wav_coeff(0)
    coef2 = spec.get_wav_coeff(1)
    coef3 = spec.get_wav_coeff(2)
    coef4 = spec.get_wav_coeff(3)
    wave = np.zeros(1024)
    for index in range(1024):
        wave[index] = coef1 + coef2*index + coef3*(index**2) + coef4*(index**3)
    return wave

def get_non_linear_correction(spec):
    coeff = np.zeros(8)
    for aa in range(8):
        coeff[aa] = spec.get_nonlin_coeff(aa)

    return coeff

def do_non_lin(data, coeff, dark_spec, integration_sec):

    step_1 = data - dark_spec*integration_sec
    poly = coeff[0] + coeff[1]*step_1 + coeff[2]*(step_1**2) + \
        coeff[3]*(step_1**3) + coeff[4]*(step_1**4) + \
        coeff[5]*(step_1**5) + coeff[6]*(step_1**6) + coeff[7]*(step_1**7)
    lin_data = step_1/poly
    return lin_data

def cal_dc_off(data, lampd, multi_factor, cutoff=0, averaging=1):

   ret = optimize.minimize(get_lsq, 1500, args=(data, lampd, multi_factor, cutoff, \
       averaging))

   return ret.x[0]

def get_lsq(dc_off, data, rad, multi_factor, cutoff, averaging):
    calib = (data[cutoff:] - (dc_off*averaging)) * multi_factor[cutoff:]/averaging
    return sum((calib-rad[cutoff:])**2)

def get_lamp_data(bins):
    actual = np.loadtxt('lmp.LMP')
    wave_limited = actual[:,0]
    rad_limited = actual[:,1]
    f = interpolate.interp1d(wave_limited, rad_limited, kind='linear')
    rad = f(bins)
    return rad

def do_collection(spec, coefficients ,integration_sec, length="", averaging=1):
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