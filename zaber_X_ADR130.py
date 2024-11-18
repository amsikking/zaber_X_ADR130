import serial

class Controller:
    '''
    Basic device adaptor for Zaber X-ADR130 linear motor XY microscope
    stage. Many more commands are available and have not been implemented.
    '''
    def __init__(self,
                 which_port,
                 name='X-ADR130',
                 verbose=True,
                 very_verbose=False,
                 x_limits_mm=(0, 130),
                 y_limits_mm=(0, 100),
                 maxspeed_mmps=(100,100)):
        self.name = name
        self.verbose = verbose
        self.very_verbose = very_verbose
        # legalize axis limits:
        for x in x_limits_mm:
            assert 0 <= x <= 130, 'x limit out of range'
        assert x_limits_mm[0] < x_limits_mm[1], 'limit error: x min > x max'
        for y in y_limits_mm:
            assert 0 <= y <= 100, 'y limit out of range'
        assert y_limits_mm[0] < y_limits_mm[1], 'limit error: y min > y max'
        self.x_min_mm, self.x_max_mm = x_limits_mm
        self.y_min_mm, self.y_max_mm = y_limits_mm
        # open serial port:
        if self.verbose: print("%s: opening..."%name, end='')
        try:
            self.port = serial.Serial(
                port=which_port, baudrate=115200, timeout=1)
        except serial.serialutil.SerialException:
            raise IOError('No connection to %s on port %s'%(name, which_port))
        if self.verbose: print(" done.")
        # check device id and address:
        cmd = b'/get device.id\n'
        if self.verbose:
            print("%s: getting device id"%self.name)
        if self.very_verbose:
            print("%s: sending cmd = "%self.name, cmd)
        self.port.write(cmd)
        response = self.port.readline()
        if self.very_verbose:
            print("%s: response    = "%self.name, response)
        if len(response) == 0:
            raise Exception("%s: no response"%self.name)
        self.device_id = response.decode('ascii').split()[-1]
        if self.verbose:
            print("%s:  = %s"%(self.name, self.device_id))
        if self.device_id == '50998':
            self.device_name = 'X-ADR130B100B-SAE53D12'
            if self.verbose:
                print("%s: device name = %s"%(self.name, self.device_name))
        else:
            raise Exception('device_id (%s) not recognised'%self.device_id)
        self.device_address = response.decode('ascii')[1:3]
        if self.verbose:
            print("%s: device address = %s"%(self.name, self.device_address))
        # get status, check warnings and home if needed:
        self._get_status()
        if self._get_warnings() == ('0 OK IDLE WR 01 WR'):
            self._home()
        warnings = self._get_warnings()
        if warnings not in ('0 OK IDLE -- 00', '0 OK IDLE NI 01 NI'):
            raise Exception('warnings (%s) not recognised'%warnings)
        # get x and y status, position and set maxspeed:
        self._get_status_x()
        self._get_status_y()        
        self.get_position_mm()
        self.set_maxspeed(maxspeed_mmps[0], maxspeed_mmps[1])

    def _send(self, cmd, response_type='reply'):
        response_type_to_char = {'reply':'@', 'info':'#', 'alert':'!'}
        cmd = bytes(
            '/' + self.device_address + ' ' + cmd + '\n', encoding='ascii')
        if self.very_verbose:
            print("%s: sending cmd = "%self.name, cmd)
        self.port.write(cmd)
        response = self.port.readline()
        if self.very_verbose:
            print("%s: response    = "%self.name, response)
        assert response.endswith(b'\r\n'), (
            "%s: unexpected response terminator"%self.name)
        response = response.decode('ascii').strip('\r\n')
        assert response[0] == response_type_to_char[response_type], (
            "%s: unexpected response character (%s)"%(self.name, response[0]))
        assert response[1:3] == self.device_address, (
            "%s: unexpected device address (%s)"%(self.name, response[1:3]))
        if self.verbose and response.split()[2] != 'OK':
            print("%s: command rejected"%self.name)
        assert self.port.in_waiting == 0
        response = response[4:] # remove response char, device address and ' '
        return response

    def _get_status(self):
        if self.very_verbose:
            print("%s: getting status"%self.name)
        status = self._send('')
        if self.very_verbose:
            print("%s:  = %s"%(self.name, status))
        self._moving = False
        if status.split()[2] == 'BUSY':
            self._moving = True
        return status

    def _get_warnings(self):
        if self.very_verbose:
            print("%s: getting warnings"%self.name)
        warnings = self._send('warnings')
        if self.very_verbose:
            print("%s:  = %s"%(self.name, warnings))
        return warnings

    def _finish_moving(self):
        while self._moving:
            self._get_status()
        if self.verbose:
            print('%s:  -> finished moving'%self.name)
        return None

    def _home(self):
        if self.verbose:
            print("%s: homing"%self.name)
        assert self._send('home') == '0 OK BUSY WR 0'
        self._moving = True
        self._finish_moving()
        if self.verbose:
            print("%s: done homing"%self.name)
        return None

    def _get_status_x(self):
        if self.very_verbose:
            print("%s: getting status on x"%self.name)
        status_x = self._send('1')
        if self.very_verbose:
            print("%s:  = %s"%(self.name, status_x))
        self._moving_x = False
        if status_x.split()[2] == 'BUSY':
            self._moving_x = True        
        return status_x

    def _finish_moving_x(self):
        while self._moving_x:
            self._get_status_x()
        if self.very_verbose:
            print('%s:  -> finished moving x'%self.name)
        return None

    def _move_x(self, x_mm, relative=True, block=True):
        if self._moving_x:
            self._finish_moving_x()
        if self.very_verbose:
            print("%s: moving to x_mm = %10.06f (relative=%s)"%(
                self.name, x_mm, relative))
        if relative: x_mm = self.x_mm + x_mm
        if not self.x_min_mm <= x_mm <= self.x_max_mm:
            if self.verbose:
                print('%s: ***WARNING*** -> x_mm out of limits'%self.name)
            return None
        self._send('1 move abs ' + str(1e6 * x_mm)) # to nm
        self._moving_x = True
        self.x_mm = x_mm
        if block:
            self._finish_moving_x()
        return None

    def _stop_x(self):
        if self.very_verbose:
            print("%s: stopping x axis"%self.name)
        assert self._send('1 stop') in ('1 OK BUSY -- 0', '1 OK BUSY NI 0')
        if self.very_verbose:
            print("%s: done stopping"%self.name)
        return None

    def _get_position_x(self):
        if self.very_verbose:
            print("%s: getting position x"%self.name)
        self.x_mm = 1e-6 * float(self._send('1 get pos').split()[4]) # from nm
        if self.very_verbose:
            print("%s:  = %6.03f (mm)"%(self.name, self.x_mm))
        return self.x_mm

    def _get_status_y(self):
        if self.very_verbose:
            print("%s: getting status on y"%self.name)
        status_y = self._send('2')
        if self.very_verbose:
            print("%s:  = %s"%(self.name, status_y))
        self._moving_y = False
        if status_y.split()[2] == 'BUSY':
            self._moving_y = True        
        return status_y

    def _finish_moving_y(self):
        while self._moving_y:
            self._get_status_y()
        if self.very_verbose:
            print('%s:  -> finished moving y'%self.name)
        return None

    def _move_y(self, y_mm, relative=True, block=True):
        if self._moving_y:
            self._finish_moving_y()
        if self.very_verbose:
            print("%s: moving to y_mm = %10.06f (relative=%s)"%(
                self.name, y_mm, relative))
        if relative: y_mm = self.y_mm + y_mm
        if not self.y_min_mm <= y_mm <= self.y_max_mm:
            if self.verbose:
                print('%s: ***WARNING*** -> y_mm out of limits'%self.name)
            return None
        self._send('2 move abs ' + str(1e6 * y_mm)) # to nm
        self._moving_y = True
        self.y_mm = y_mm
        if block:
            self._finish_moving_y()
        return None

    def _stop_y(self):
        if self.very_verbose:
            print("%s: stopping y axis"%self.name)
        assert self._send('2 stop') in ('2 OK BUSY -- 0', '2 OK BUSY NI 0')
        if self.very_verbose:
            print("%s: done stopping"%self.name)
        return None

    def _get_position_y(self):
        if self.very_verbose:
            print("%s: getting position y"%self.name)
        self.y_mm = 1e-6 * float(self._send('2 get pos').split()[4]) # from nm
        if self.very_verbose:
            print("%s:  = %6.03f (mm)"%(self.name, self.y_mm))
        return self.y_mm

    def get_position_mm(self):
        if self.verbose:
            print("%s: getting position"%self.name)
        self.x_mm, self.y_mm = [
            1e-6 * float(p) for p in self._send('get pos').split()[4:6]]
        if self.verbose:
            print("%s:  = (%6.03f, %6.03f) (mm)"%(
                self.name, self.x_mm, self.y_mm))
        return self.x_mm, self.y_mm

    def move_mm(self, x_mm, y_mm, relative=True, block=True):
        if self.verbose:
            print("%s: moving to x_mm, y_mm"%self.name)
            print("%s:  = %10.06f, %10.06f (relative=%s)"%(
                self.name, x_mm, y_mm, relative))
        self._move_x(x_mm, relative=relative, block=False)
        self._move_y(y_mm, relative=relative, block=False)
        self._moving = True
        if block:
            self._finish_moving()
            self._moving_x = False
            self._moving_y = False
        return None

    def get_maxspeed(self):
        if self.verbose:
            print("%s: getting maxspeed"%self.name)
        self.x_mmps, self.y_mmps = [
            round(1e-6 * float(s) / 1.6384, 3) for s in self._send(
                'get maxspeed').split()[4:6]]
        if self.verbose:
            print("%s:  = %6.03f, %6.03f (mm/s)"%(
                self.name, self.x_mmps, self.y_mmps))
        return self.x_mmps, self.y_mmps

    def set_maxspeed(self, x_mmps=None, y_mmps=None):
        if x_mmps is None: x_mmps = self.x_mmps
        if y_mmps is None: y_mmps = self.y_mmps
        if self.verbose:
            print("%s: setting maxspeed (x, y)"%self.name)
            print("%s:  = %10.06f, %10.06f (mm/s)"%(self.name, x_mmps, y_mmps))
        assert 0.01 <= x_mmps <= 750 and 0.01 <= y_mmps <= 750
        self._send('1 set maxspeed ' + str(1e6 * 1.6384 * x_mmps))
        self._send('2 set maxspeed ' + str(1e6 * 1.6384 * y_mmps))
        assert self.get_maxspeed() == (x_mmps, y_mmps)
        if self.verbose:
            print("%s: -> done setting maxspeed"%self.name)
        return None

    def close(self):
        if self.verbose: print("%s: closing..."%self.name, end=' ')
        self.port.close()
        if self.verbose: print("done.")

if __name__ == '__main__':
    stage = Controller(which_port='COM15', verbose=True, very_verbose=False)

    print('\nAbsolute and relative moves:')
    stage.move_mm(65, 50, relative=False)
    stage.move_mm(1, 1)

    print('\nNon-blocking call:')
    stage.move_mm(1, 1, block=False)
    print(' do something else...')
    stage.move_mm(1, 1)

##    print('\nMove and stop:')
##    stage._move_x(stage.x_max_mm, relative=False, block=False)
##    stage._stop_x()
##    stage._get_position_x()
##    stage._move_y(stage.y_max_mm, relative=False, block=False)
##    stage._stop_y()
##    stage._get_position_y()

    print('\nRe-center:')
    stage.move_mm(65, 50, relative=False)

    stage.close()
