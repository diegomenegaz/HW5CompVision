I am literally just keeping track of errors in this file.
Traceback (most recent call last):
  File "/usr/lib/python3.11/threading.py", line 1038, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.11/threading.py", line 975, in run
    self._target(*self._args, **self._kwargs)
  File "/home/g37/librealsense/Maestro/detectMarker.py", line 141, in movement_thread
    motor_controller.setTarget(MPORT2, 5500)
  File "/home/g37/librealsense/Maestro/maestro.py", line 93, in setTarget
    self.sendCmd(cmd)
  File "/home/g37/librealsense/Maestro/maestro.py", line 54, in sendCmd
    self.usb.write(bytes(cmdStr,'latin-1'))
  File "/usr/local/lib/python3.11/dist-packages/pyserial-3.5-py3.11.egg/serial/serialposix.py", line 662, in write
serial.serialutil.SerialException: write failed: [Errno 5] Input/output error
QObject::killTimer: Timers cannot be stopped from another thread
QObject::~QObject: Timers cannot be stopped from another thread
