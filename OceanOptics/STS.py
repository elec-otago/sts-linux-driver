''' This is a classfile which acts as a driver for the OceanOptics STS-VIS
    spectrometer based purely on Python code, it is formatted for use with the
    usb package for USB communication, most importantly the usb.core file.
    There are a few methods for spectrometer functions not currently
    Implemented in this version.

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

    Last updated: 11/09/2014

    Author: Matthew West
        Department of Physics,
        University of Otago
        Dunedin, New Zealand

    Email: wesma651@student.otago.ac.nz
'''

import usb.core
import usb
import struct
import numpy as np
import time

class STSVIS(object):
    """ class STSVIS:
        This classfile for STS-VIS spectrometer communication was written using
        information from the Spec sheet:
        http://www.oceanoptics.com/technical/engineering/STS%20Data%20Sheet.pdf
        Some of the documentation for each method was taken directly from this
        spec sheet.
    """

    def __init__(self, index=0):
        ''' Initialization of the device, this finds the device and prints the
            address. Also sets up the default values for the packet for sending
            data to the device. May edit to make more robust later.
        '''
        device_list = usb.core.find(find_all=True, idVendor=0x2457, \
            idProduct=0x4000)
        self.list = len(device_list)
        if device_list is None:
            raise STS_Error('No OceanOptics STS-VIS spectrometer found!')
        else:
            if len(device_list) > index:
                self._dev = device_list[index]
            else: raise STS_Error('Not enough Spectrometers connected check ' \
                'your connections')
        
        # This part makes the initialization a little bit more robust
        if self._dev.is_kernel_driver_active(0) is True:
            self._dev.detach_kernel_driver(0)
            usb.util.claim_interface(self._dev, 0)
            usb.util.release_interface(self._dev, 0)
        #Set the addresses for the Endpoints for USB communication
        self._EP1_out = 0x01
        self._EP1_in = 0x81
        self._EP2_in = 0x82
        self._EP2_out = 0x02
        #Maximum data packet size.
        self._EP1_in_size = 64
        self._EP2_in_size = 64

        #Initialize the different fields for packet size. This is important
        #    as the packet is stitched together from these data fields. Each
        #    command that requires an adjustment to these default values will
        #    set that itself, and then re set to the default itself also.

        self.headerTop = np.array([193, 192])
        self.protocolVersion = np.array([17, 0])
        self.flags = np.array([0, 0])
        self.errorNumber = np.array([0, 0])
        self.messageType = None
        self.regarding = np.array([0, 0, 0, 0])
        self.reserved = np.array([0, 0, 0, 0, 0, 0])
        self.checksumType = 0
        self.immediateDataLength = 0
        self.immediateData = np.zeros(16)
        self.bytesRemaining = np.array([20, 0, 0, 0])
        self.payload = None
        self.checksum = np.zeros(16)
        self.footer = np.array([197, 196, 195, 194])

        #Flags: RESPONSE_FLAG -> 1, ACK_FLAG -> 2, ACK_REQUESTED_FLAG -> 4,
        #    NACK_FLAG -> 8, EXCEPTION_FLAG -> 16.
        serial = self.get_serial()
        # print 'Device Found, The device is : %s'  %serial

    # ################################################# #
    # The following functions make up the User interface
    # ################################################# #

    # ################################################# #
    #      These are the general control functions      #
    # ################################################# #

    def reset_device(self, line=1):
        ''' Sends the reset signal to the device. Needs to except core.USBError
            and device may not work afterwards, it seems to be unavailable, may
            be only useful for when multiple devices are connected to the same
            board/computer.
        '''
        try:
            self._send_command_to_device(0x00000000, line)
        except usb.core.USBError:
            pass
        time.sleep(1.5)

    def reset_defaults(self, line=1):
        ''' Clears certain persisted values including default baud rate and
            pixel binning mode. Does not erase serial number, bench, alias,
            calibration, or user strings.
        '''
        self._send_command_to_device(0x00000001, line)

    def get_hardware_revision(self, line=1):
        ''' This value is sensed from the hardware itself. Request has no
            payload. Reply is a single byte.
            Returns the integer value.
        '''
        return self._query_device(0x00000080, line)[0]

    def get_firmware_revision(self, line=1):
        ''' Firmware version as binary coded decimal. The same value should
            be available through the USB descriptor as the bcdDevice field.
            Request has no payload. Reply is a 2-byte integer (LSB first) of
            the revision.
            Returns the integer value.
        '''
        data = self._query_device(0x00000090, line)
        return data[0] + data[1]*256

    def get_serial(self, line=1):
        ''' Returns the serial number of the device, returned as a string,
            that is this function converts the bytestring to letters.
        '''
        string = self._query_device(0x00000100, line)
        result = ''
        for ab in range(len(string)):
            result += str(unichr(int(string[ab])))
        return result

    def get_serial_length(self, line=1):
        ''' Reply is a single byte saying the maximum length of the serial
            number string that can be set.
            Returns the integer value.
        '''
        return self._query_device(0x00000101, line)[0]

    def get_alias_length(self, line=1):
        ''' Reply is a single byte saying the maximum length of the alias
            string that can be set.
            Returns the integer value.
        '''
        return self._query_device(0x00000201, line)[0]

    def get_alias(self, line=1):
        ''' Returns the user allocated alias of the device, returned as a
            string, that is this function converts the bytestring to letters.
        '''
        string = self._query_device(0x00000200, line)
        result = ''
        for ab in range(len(string)):
            result += str(unichr(int(string[ab])))
        return result

    def set_alias(self, stringname, line=1):
        ''' Sets the user allocated alias of the device, given as a string.
        '''
        if len(stringname) <= 16:
            for ab in range(len(stringname)):
                self.immediateData[ab] = ord(stringname[ab])
            self.immediateDataLength = len(stringname)
            self._send_command_to_device(0x00000210, line)
            self.immediateDataLength = 0
        else:
            print "Alias Length cannot be longer than 16 characters"

    def get_user_string_count(self, line=1):
        ''' This function returns the integer number of user defined strings
        '''
        string = self._query_device(0x00000300, line)[0]
        return int(string)

    def get_user_string_length(self, line=1):
        ''' This function returns the maximum integer length of user defined
            strings
        '''
        string = self._query_device(0x00000301, line)
        length = string[0] + 256*string[1]
        return int(length)

    def get_user_string(self, ind, line=1):
        ''' This function returns the user defined string of index ind
        '''
        self.immediateDataLength = 1
        self.immediateData[0] = ind
        string = self._query_device(0x00000302, line)
        self.immediateDataLength = 0
        result = ''
        if string == None:
            print "There is no User String with this Index"
        else:
            for ab in range(len(string)):
                dat = str(unichr(int(string[ab])))
                if dat != struct.pack('<B', 0):
                    result += str(unichr(int(string[ab])))
        return result

    def set_user_string(self, stringname, string_ind, line=1):
        ''' Sets the user string with index string_ind, given as a string.
        '''
        self.regarding = np.array([0, 1, 0, 2])
        if len(stringname) <= 348:
            if len(stringname) <= 15:
                self.immediateData[0] = string_ind
                for ab in range(len(stringname)):
                    self.immediateData[ab+1] = ord(stringname[ab])
                self.immediateDataLength = len(stringname) + 1
                self._send_command_to_device(0x00000310, line)
                self.immediateDataLength = 0
            else:
                data = np.zeros(len(stringname) + 1)
                data[0] = string_ind
                for ab in range(len(stringname)):
                    data[ab+1] = ord(stringname[ab])
                bb = ((len(data)/64) + 1)*64

                self._update_bytes_remaining(bb)
                self._send_command_to_device(0x00000310, line, True, data)
        else:
            print "Alias Length cannot be longer than %d \
                characters" & self.get_user_string_length()

        self.regarding = np.array([0, 0, 0, 0])
        self._update_bytes_remaining(0)

    #Not Implemented methods
    #    get_RS232_baud()
    #    get_RS232_flow_control()
    #    set_RS232_baud()
    #    set_RS232_flow_control()
    #    save_RS232_setttings()

    def configure_status_led(self, command, line=1):
        ''' This method sets the operation of the LED:
            Sends two byte message: Byte 0 is reserved and must always be 0x00.
            Byte 1 is the pattern to drive the LED:
                1 = LED will blink in an S-O-S pattern at a high priority (this
            will not override a POST or hard fault indication, but will
            override all others)
                2 = LED will fade in and out at a low priority (anything but
            the solid-on idle pattern will override this)
                If this byte is anything other than 1 or 2 then the LED will
                revert to its normal operation.
        '''
        if (command == 1) or (command == 2):
            self.immediateData[0] = 0
            self.immediateData[0] = command
            self.immediateDataLength = 2
            self._send_command_to_device(0x00001010, line)
            self.immediateDataLength = 0
        else:
            print "Please enter either 1 or 2 for status led commands"
            print "Refer to the software documentation for more information"

    def reprogramming_mode(self, line=1):
        ''' Causes the device to accept a .OBP file provided by Ocean Optics.
        '''
        self._send_command_to_device(0x000FFF00, line)

    # ################################################# #
    #        These are the spectrometer commands        #
    # ################################################# #

    def get_corrected_spectrum(self, line=1):
        ''' Request corrected spectra from device and read the response, by
            corrected it returns the intensity of every pixel on the detector.
            If the response is in the correct format, return is as a spectrum
            of intensity values.
        '''
        data = self._query_device(0x00101000, line)
        spectrum = np.zeros(len(data)/2)
        for ab in range(len(spectrum)):
            spectrum[ab] = data[ab*2] + 256*(data[ab*2 + 1])
        return spectrum

    def get_raw_spectrum(self, line=1):
        ''' Request spectra from device and read the response, this returns the
            raw data, that is the actual ADC output of the pixels.
        '''
        data = self._query_device(0x00101100, line)
        spectrum = np.zeros(len(data)/2)
        for ab in range(len(spectrum)):
            spectrum[ab] = data[ab*2] + 256*(data[ab*2 + 1])
        return spectrum

    def get_partial_spectrum_mode(self, line=1):
        ''' Returns a specification for partial spectrum retrieval (see the
            respective set method for details). If no specification has been
            set since the device was started, this will return a NACK
            indicating "no value available".
        '''
        raise NotImplementedError

    #Not Implemented methods
    #     set_partial_spectrum_mode()
    #     get_partial_corrected()

    def set_integration_time(self, time_us, line=1):
        ''' Sets the integration time on the device to be time_us in micro
            seconds.
        '''
        a_bit = (time_us/(256**3))%256
        b_bit = (time_us/(256**2))%256
        c_bit = (time_us/256)%256
        d_bit = time_us%256
        self.immediateData[0:4] = np.array([d_bit, c_bit, b_bit, a_bit])
        self.immediateDataLength = 4
        self._send_command_to_device(0x00110010, line)
        self.immediateDataLength = 0
        time.sleep(.5)

    def set_trigger_mode(self, trig, line=1):
        ''' Sets the STS trigger mode, possible modes are:
                Mode 0 (default): Integration begins as soon as possible after
            request.
                Mode 1: Integration or trigger delay begins with a rising
            trigger edge.
                Mode 2: Integration is internally triggered to synchronize with
            continuous strobe
        '''
        if abs(trig) < 3:
            self.immediateData[0] = trig
            self.immediateDataLength = 1
            self._send_command_to_device(0x00110110, line)
            self.immediateDataLength = 0
        else:
            print 'Please enter and integer value 0, 1 or 2 for trigger mode'

    def simulate_trigger_pulse(self, line=1):
        ''' Causes the STS to react exactly as though an electrical rising edge
            signal was applied to the external trigger pin of the device. This
            is intended for cases where the device is put into an external
            trigger mode and an acquisition has been started, but no signal is
            forthcoming. By sending this command over a different
            communications interface (e.g. RS232 or the second USB endpoint) it
            is possible to make the device finish its acquisition normally.
        '''
        self._send_command_to_device(0x00110120, line)

    def get_pixel_binning_factor(self, line=1):
        ''' Returns a single byte indicating the binning mode.
        '''
        return self._query_device(0x00110280, line)[0]

    def get_max_binning_factor(self, line=1):
        ''' Returns a single byte representing the largest binning factor that
            may be used (3). The minimum is assumed to be zero.
        '''
        return self._query_device(0x00110281, line)[0]

    def get_default_binning_factor(self, line=1):
        ''' Returns the startup binning factor as a single byte
        '''
        return self._query_device(0x00110285, line)[0]

    def set_pixel_binning_factor(self, factor, line=1):
        ''' Takes a single byte indicating the binning mode. This is used for
            this bus until the device is reset.
        '''
        self.immediateData[0] = factor
        self.immediateDataLength = 1
        self._send_command_to_device(0x00110290, line)
        self.immediateDataLength = 0

    def set_default_binning_factor(self, factor=None, line=1):
        ''' Takes a single byte indicating the default binning mode. If no
            factor is given, this will reset to factory default. This is
            used for this bus until the device is reset.
        '''
        if factor != None:
            self.immediateData[0] = factor
            self.immediateDataLength = 1
        self._send_command_to_device(0x00110290, line)
        self.immediateDataLength = 0

    def set_lamp_enable(self, enable, line=1):
        ''' Refers to the external enable pin. Changes take effect at the
            beginning of the next spectral acquisition. If unsynchronized
            control is required, connect to a GPIO instead. Input is 1 byte:
            0 = off, 1= on.
        '''
        if (enable == 0) or (enable == 1):
            self.immediateDataLength = 1
            self.immediateData[0] = enable
            self._send_command_to_device(0x00110410, line)
            self.immediateDataLength = 0
        else:
            print 'Please use either 0 or 1 for lamb enable configuration'

    def set_trigger_delay(self, time_us, line=1):
        ''' Sets the trigger delay on the device to be time_us in micro
            seconds.
        '''
        a_bit = (time_us/(256**3))%256
        b_bit = (time_us/(256**2))%256
        c_bit = (time_us/256)%256
        d_bit = time_us%256
        self.immediateData[0:4] = np.array([d_bit, c_bit, b_bit, a_bit])
        self.immediateDataLength = 4
        self._send_command_to_device(0x00110510, line)
        self.immediateDataLength = 0

    def get_scans_to_avg(self, line=1):
        ''' Returns the current setting for number of the scans to average as
            a single byte. Note this data is actually line dependant. Be aware
            this may be true for other methods.
        '''
        data = self._query_device(0x00120000, line)
        return int(data[0]+data[1]*256)

    def set_scans_to_avg(self, scans, line=1):
        ''' Takes a 16-bit int indicating the number of scans to average over
            when taking a spectrum. If the value is between 1-5000, otherwise
            an error message is returned.  Note this data is actually line
            dependant. Be aware this may be true for other methods.
        '''
        if scans < 5001 and scans > 0:
            self.immediateData[0] = scans%256
            self.immediateData[1] = (scans/256)%256
            self.immediateDataLength = 2
            self._send_command_to_device(0x00120010, line)
        else:
            print "Please enter a number between 1 and 5000 for the number"  \
                " of Scans to average over."
        self.immediateDataLength = 0

    def get_boxcar(self, line=1):
        ''' Returns the boxcar width being applied to all spectra. Valid range
            is 0-15. This is also line dependant.
        '''
        return self._query_device(0x00121000, line)[0]

    def set_boxcar(self, width, line=1):
        ''' Takes a single byte giving the boxcar width to apply to all
            spectra. Valid range is 0-15. Boxcar smoothing will apply to every
            pixel, even if there are not a balanced number of neighbours on
            both sides. This is also line dependant.
        '''
        if width < 16 and width >= 0:
            self.immediateData[0] = width
            self.immediateDataLength = 1
            self._send_command_to_device(0x00121010, line)
        else:
            print "Please enter a number between 0 and 15 for the boxcar" \
                " width."
        self.immediateDataLength = 0

    # ########################################### #
    #     These are the calibration functions     #
    # ########################################### #

    def get_wav_coeff_count(self, line=1):
        ''' Returns the number of wavelength coefficients
        '''
        return self._query_device(0x00180100, line)[0]

    def get_wav_coeff(self, index, line=1):
        ''' Returns the wavelength coefficient specified by 'index'
        '''
        self.immediateDataLength = 1
        self.immediateData[0] = index
        data = self._query_device(0x00180101, line)
        self.immediateDataLength = 0
        return struct.unpack('<f', struct.pack('<4B', data[0], data[1], \
            data[2], data[3]))[0]

    def set_wav_coeff(self, index, coeff, line=1):
        ''' Sets the wavelength coefficient with the index given to be coeff.
        '''
        self.immediateData[0] = index
        self.immediateData[1:5] = struct.unpack('<4B', struct.pack('<f', \
            coeff))
        self.immediateDataLength = 5
        self._send_command_to_device(0x00180111, line)
        self.immediateDataLength = 0

    def get_nonlin_coeff_count(self, line=1):
        ''' Returns the number of non linearity coefficients
        '''
        return self._query_device(0x00181100, line)[0]

    def get_nonlin_coeff(self, index, line=1):
        ''' Returns the non linearity coefficient specified by 'index'
        '''
        self.immediateDataLength = 1
        self.immediateData[0] = index
        data = self._query_device(0x00181101, line)
        self.immediateDataLength = 0
        return struct.unpack('<f', struct.pack('<4B', data[0], data[1], \
            data[2], data[3]))[0]

    def set_nonlin_coeff(self, index, coeff, line=1):
        ''' Sets the non linearity coefficient with the index given to be coeff.
        '''
        self.immediateData[0] = index
        self.immediateData[1:5] = struct.unpack('<4B', struct.pack('<f', coeff))
        self.immediateDataLength = 5
        self._send_command_to_device(0x00181111, line)
        self.immediateDataLength = 0

    def get_irrad_calib(self, line=1):
        ''' Reply has up to 4096 bytes (whatever has been stored previously),
            intended for 1024 x 4-byte floats. If nothing has been stored, the
            reply will have NACK bit set in flags.
        '''
        try:
            data = self._query_device(0x00182001, line)
        except STS_Error:
            print 'There is no data for irradiance calibration.'
        else:
            calib = np.zeros(len(data)/4)
            for ab in range(len(calib)):
                calib[ab] = struct.unpack('<f', struct.pack('<4B', \
                    data[0+ab*4], data[1+ab*4], data[2+ab*4], \
                    data[3+ab*4]))[0]
            return calib

    def get_irrad_calib_count(self, line=1):
        ''' Reply is a 4-byte integer indicating the total number of 4-byte
            floats to be returned in a reply of get_irrad_calibration
            including the zero values
        '''
        data = self._query_device(0x00182002, line)
        return int(data[0]+256*(data[1]+256*(data[2]+256*data[3])))

    def get_irrad_calib_area(self, line=1):
        ''' If a collection area has been set, it is returned as a 4-byte float
            if not defined a NACK is returned and an error message is printed.
        '''
        try:
            data = self._query_device(0x00182003, line)
        except STS_Error:
            print 'There is no area for collection set.'
        else:
            area = struct.unpack('<f', struct.pack('<4B', data[0], data[1], \
                data[2], data[3]))[0]
            return area

    def set_irrad_calib(self, calibration, line=1):
        ''' Send a list of floats as the calibration. Request has up to 4096
            bytes in payload. This corresponds to up to 1024 floats. Sending a
            zero-length buffer will delete any irradiance calibration from STS.
            No reply. This is data storage. 
        '''
        ind = 0
        count = 0
        data = np.zeros(len(calibration)*4)
        for index in calibration:
            data[ind] = index%256
            ind += 1
            data[ind] = (index/256)%256
            ind += 1
            data[ind] = (index/(256**2))%256
            ind += 1
            data[ind] = (index/(256**3))%256
            ind += 1
            
            self._update_bytes_remaining(len(data))
            self._send_command_to_device(0x00186010, line, \
                payload=True, data=data)
            self._update_bytes_remaining(0)

    def set_irrad_calib_area(self, area, line=1):
        ''' Sets the colection area for irradiance calibration, Sending a
            zero-length buffer will delete any collection area previously
            stored.
        '''
        self.immediateData[0:4] = struct.unpack('<4B', struct.pack('<f', \
            area))
        self.immediateDataLength = 4
        self._send_command_to_device(0x00182011, line)
        self.immediateDataLength = 0

    def get_stray_light_coeff_count(self, line=1):
        ''' Returns the number of stray light coefficients
        '''
        return self._query_device(0x00183100, line)[0]

    def get_stray_light_coeff(self, order, line=1):
        ''' Returns the non linearity coefficient with the order of the
            coefficient, specified by 'order'
        '''
        self.immediateDataLength = 1
        self.immediateData[0] = order
        data = self._query_device(0x00183101, line)
        self.immediateDataLength = 0
        return struct.unpack('<f', struct.pack('<4B', data[0], data[1], \
            data[2], data[3]))[0]

    def set_stray_light_coeff(self, order, coeff, line=1):
        ''' Sets the stray light coefficient of order given by
            'order' to be 'coeff'.
        '''
        self.immediateData[0] = order
        self.immediateData[1:5] = struct.unpack('<4B', struct.pack('<f', coeff))
        self.immediateDataLength = 5
        self._send_command_to_device(0x00183111, line)
        self.immediateDataLength = 0

    def get_hot_pixel_index(self, line=1):
        ''' Reply is up to 52 x 2-byte integers. This returns an array of
            integer indexes of the stored hot pixels of the device.
            If nothing has been stored this will return a NACK in Flags.
        '''
        data = self._query_device(0x00186000, line)
        ind = len(data)/2
        indices = np.zeros(ind)
        for ab in range(ind):
            indices[ab] = 256*data[ab*2 + 1] +data[ab*2]
        return indices

    def set_hot_pixel_index(self, indices, line=1):
        ''' Sets the hot pixel indices of the device. It is suggested that the
            user runs the corresponding get method first and adds any new
            indices onto the end of this np array and then sends it to this
            method as the indices array
        '''
        ind = 0
        count = 0
        data = np.zeros(len(indices)*2)
        for index in indices:
            data[ind] = index%256
            ind += 1
            data[ind] = (index/256)%256
            ind += 1
            if len(data) <= 16:
                self.immediateData[count] = data[count]
                count += 1
                self.immediateData[count] = data[count]
                count += 1
                self.immediateDataLength = ind
                self._send_command_to_device(0x00186010, line)
                self.immediateDataLength = 0
            else:
                self._update_bytes_remaining(len(data))
                self._send_command_to_device(0x00186010, line, \
                    payload=True, data=data)
                self._update_bytes_remaining(0)

    def get_bench_ID(self, line=1):
        ''' Reply is up to 32 byte ASCII string in output.
        '''
        string = self._query_device(0x001B0000, line)
        result = ''
        for ab in range(len(string)):
            result += str(unichr(int(string[ab])))
        return result

    def get_bench_serial(self, line=1):
        ''' Reply is an ASCII string in output.
        '''
        string = self._query_device(0x001B0100, line)
        result = ''
        for ab in range(len(string)):
            result += str(unichr(int(string[ab])))
        return result

    def get_slit_width(self, line=1):
        ''' Reply is a two byte integer of the slit width.
        '''
        data = self._query_device(0x001B0200, line)
        width = data[0] + 256*data[10]
        return width

    def get_fiber_diameter(self, line=1):
        ''' Reply is a two byte integer of the fiber diameter.
        '''
        data = self._query_device(0x001B0300, line)
        width = data[0] + 256*data[1]
        return width

    def get_grating(self, line=1):
        ''' Reply is an ASCII string in output.
        '''
        string = self._query_device(0x001B0400, line)
        result = ''
        for ab in range(len(string)):
            result += str(unichr(int(string[ab])))
        return result    

    def get_filter(self, line=1):
        ''' Reply is an ASCII string in output.
        '''
        string = self._query_device(0x001B0500, line)
        result = ''
        for ab in range(len(string)):
            result += str(unichr(int(string[ab])))
        return result

    def get_coating(self, line=1):
        ''' Reply is an ASCII string in output.
        '''
        string = self._query_device(0x001B0600, line)
        result = ''
        for ab in range(len(string)):
            result += str(unichr(int(string[ab])))
        return result

    # ########################################### #
    #     These are the GPIO command functions    #
    # ########################################### #

    def get_number_GPIO_pins(self, line=1):
        ''' Reply is an unsigned byte of the number of pins.
        '''
        return self._query_device(0x00200000, line)[0]
    
    def get_output_enable_vector(self, line=1):
        raise NotImplementedError

    def set_output_enable_vector(self, line=1):
        raise NotImplementedError

    def get_value_vector(self, line=1):
        raise NotImplementedError

    def set_value_vector(self, line=1):
        raise NotImplementedError

    # ########################################### #
    #    These are the strobe command functions   #
    # ########################################### #

    def set_single_strobe_pulse_delay(self, line=1):
        raise NotImplementedError

    def set_single_strobe_pulse_width(self, line=1):
        raise NotImplementedError

    def set_single_strobe_enable(self, line=1):
        raise NotImplementedError

    def set_cont_strobe_period(self, line=1):
        raise NotImplementedError

    def set_cont_strobe_enable(self, line=1):
        raise NotImplementedError

    # ########################################### #
    #     These are the temperature functions     #
    # ########################################### #

    def get_temperature_sensor_count(self, line=1):
        ''' Reply is a byte with the number of sensors.
        '''
        return self._query_device(0x00400000, line)[0]

    def read_temperature_sensor(self, sensor, line=1):
        ''' Reply is a 4-byte corresponding to a single precision float.
            The input is which sensor to get the temperature from:
                0 = Detector Board Thermistor
                1 = Reserved/Internal Use
                2 = Microcontroller Sensor Temperature
        '''
        self.immediateDataLength = 1
        self.immediateData[0] = sensor
        data = self._query_device(0x00400001, line)
        self.immediateDataLength = 0
        return struct.unpack('<f', struct.pack('<4B', data[0], data[1], \
            data[2], data[3]))[0]

    def read_all_temperature(self, line=1):
        ''' Reply is 3 4-bytes corresponding to 3 single precision floats.
            Each refers to the the result returned by the function
            read_temperature_sensor() for all 3 possible indices.
        '''
        data = self._query_device(0x00400002, line)
        self.immediateDataLength = 0
        return struct.unpack('<3f', struct.pack('<12B', data[0], data[1], \
            data[2], data[3], data[4], data[5], data[6], data[7], data[8], \
            data[9], data[10], data[11]))

    # ########################################### #
    # The user doesn't need to see these function #
    # ########################################### #

    def _send_command_to_device(self, command, line, payload=False, data=None):
        ''' This function writes the packet to the device, and does a single
            read to look for the ACK.
        '''
        if payload == True:
            if data == None:
                print "No Payload Present"
            else:
                # print "Payload Received"
                packet = self._build_packet(command, 4, data)
                sends = len(packet)/64

                # Send to the correct line then wait a small amount of time
                for ab in range(sends):
                    n = 64*ab
                    if line == 1:
                        self._dev.write(self._EP1_out, packet[n:n+64])
                    elif line == 2:
                        self._dev.write(self._EP2_out, packet[n:n+64])
                    else:
                        print 'Please enter correct line choice. 1 or 2'
                        raise _OOError('Wrong endpoint line choice')
                    time.sleep(.1)

        else:
            packet = self._build_packet(command, 4)

            #Send to the correct line then wait a small amount of time
            if line == 1:
                self._dev.write(self._EP1_out, packet)
            elif line == 2:
                self._dev.write(self._EP2_out, packet)
            else:
                print 'Please enter correct line choice. 1 or 2'
                raise _OOError('Wrong endpoint line choice')
            time.sleep(.1)

        if command != 0: #If we didn't send the reset command
            read = self._read_device(line)
            # print read
            if read[4] != 3: #If "dead" data, read what is on the line
                read = self._read_device(line)
                # print read
                if read[4] != 3: #If still wrong, manage the error
                    self._error_management(read[6])

    def _query_device(self, command, line):
        ''' This function also writes the packet to the device, but this time
            it is for a data request, so it does the initial read looking for
            the response and when found it will return that data. If the
            response has a payload then an external read is called looking for
            the amount of data specified in the bytesRemaining field of the
            first packet, else the internal read is called looking for only
            the immediateData field with length given by immediateDataLength
        '''
        packet = self._build_packet(command, 0)

        #Send to the correct line then wait a small amount of time
        if line == 1:
            self._dev.write(self._EP1_out, packet)
        elif line == 2:
            self._dev.write(self._EP2_out, packet)
        else:
            print 'Please enter correct line choice. 1 or 2'
            raise _OOError('Wrong endpoint line choice')
        time.sleep(.1)

        read = self._read_device(line)
        # print read
        if read[4] != 1: #If "dead" data, read the data on the line already
            read = self._read_device(line)
            # print read
            if read[4] != 1: #If still wrong, manage the error
                self._error_management(read[6])
            else:
                bytes_left = read[40] + 256*(read[41] + 256*(read[42] + \
                    256*(read[43])))
                if bytes_left == 20:
                    to_read = read[23]
                    return self._internal_read(read, to_read)
                else: return self._external_read(line, read, bytes_left)

    def _read_device(self, line):
        ''' This function reads the device on the correct line. It is called
            by a function which the user can see and that function defines the
            repeating reads or not
        '''
        if line == 1:
            ret = self._dev.read(self._EP1_in, 64, timeout=1000000)
        elif line == 2:
            ret = self._dev.read(self._EP2_in, 64, timeout=1000000)
        else:
            print 'Please enter correct line choice. 1 or 2'
            raise _OOError('Wrong endpoint line choice')
        return ret

    def _build_packet(self, commandtype, flags_up, data=None):
        ''' This is the function that constructs the packet from the internal
            data structures and returns it.
        '''
        package = struct.pack('<8B', self.headerTop[0], self.headerTop[1], \
            self.protocolVersion[0], self.protocolVersion[1], \
            (self.flags[0] + flags_up), self.flags[1], self.errorNumber[0], \
            self.errorNumber[1])

        package += struct.pack('<I', commandtype)

        package += struct.pack('<12B', self.regarding[0], self.regarding[1], \
            self.regarding[2], self.regarding[3], 0, 0, 0, 0, 0, 0, \
            self.checksumType, self.immediateDataLength)

        for ii in range(16):
            package += struct.pack('<B', self.immediateData[ii])

        package += struct.pack('<4B', self.bytesRemaining[0], \
            self.bytesRemaining[1], self.bytesRemaining[2], \
            self.bytesRemaining[3])

        if data != None:
            #Need to be able to add the payload in somehow
            for jj in range(len(data)):
                package += struct.pack('<B', data[jj])
            while len(package) % 64 != 44:
                package += struct.pack('<B', 0)


        for ii in range(16):  #Reserved BYTES
            package += struct.pack('<B', self.checksum[ii])

        package += struct.pack('<4B', self.footer[0], self.footer[1], \
            self.footer[2], self.footer[3])

        return package

    def _internal_read(self, read, bytes_for_reading):
        ''' This function will read a single packet off the device and return
            just the data from this packet
        '''
        data = np.zeros(bytes_for_reading)
        for ab in range(bytes_for_reading):
            data[ab] = read[24 + ab]
        return data

    def _external_read(self, line, read, bytes_for_reading):
        ''' This function will read all the remaining packets in the data
            stream and returns the data of interest
        '''
        to_read = bytes_for_reading/64
        for kk in range(to_read):
            read += self._read_device(line)

        #Takes the data off the read packets.
        data = np.zeros(to_read*64) #Needs this not 'bytes' to ignore last 20
        for  ll in range(to_read*64):
            data[ll] = read[44 + ll]
        return data

    def _update_bytes_remaining(self, change):
        ''' This function updtes the data field which is concerned with the
            number of bytes remaining in the package. An input value of 0 in
            the change field will reset it to the default 20, which corresponds
            to no payload.
        '''
        val = change + 20
        a_bit = (val/(256**3))%256
        b_bit = (val/(256**2))%256
        c_bit = (val/256)%256
        d_bit = val%256
        self.bytesRemaining = np.array([d_bit, c_bit, b_bit, a_bit])

    def _update_immediate_data(self, length, data):
        ''' This function will replace the code in many of the sending command
            functions that make up the interface, this will just tidy up the
            code somewhat.
        '''
        raise NotImplementedError

    def _error_management(self, error):
        ''' This function is the error handler, it will just print the error
            Type and the message that comes with that error out for the user.
        '''
        if error == 0:
            print "No detectable errors"
        elif error == 1:
            print "Invalid/unsupported protocol"
        elif error == 2:
            print "Unknown message type"
        elif error == 3:
            print "Bad checksum"
        elif error == 4:
            print "Message too large"
        elif error == 5:
            print "Payload length does not match message type"
        elif error == 6:
            print "Payload data invalid"
        elif error == 7:
            print "Device not ready for given message type"
        elif error == 8:
            print "Unknown checksum type"
        elif error == 9:
            print "Device reset unexpectedly"
        elif error == 10:
            print "Too many buses (Commands have come from too many bus" \
                "interfaces)"
        elif error == 11:
            print "Out of memory. Failed to allocate enough space to" \
                "complete request."
        elif error == 12:
            print "Command is valid, but desired information does not exist."
        elif error == 13:
            print "Int Device Error. May be unrecoverable."
        elif error == 100:
            print "Could not decrypt properly"
        elif error == 101:
            print "Firmware layout invalid"
        elif error == 102:
            print "Data packet was wrong size (not 64 bytes)"
        elif error == 103:
            print "Hardware revision not compatible with firmware "
        elif error == 104:
            print "Existing flash map not compatible with firmware"
        elif error == 255:
            print "Operation/Response Deferred. Operation will take some" \
                "time to complete. Do not ACK or NACK yet."
        else:
            print "Error Undetermined"

        raise STS_Error('Device %s sent back error' % (self._dev, ))

class STS_Error(Exception):
    ''' This is the error class which is raised by the Driver in its error
        management function.
    '''
    def __init__(self, value):
        print value