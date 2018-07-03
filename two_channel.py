import threespace_api as ts_api
import stimulator
import serial
import struct
import math
import thread
import time

IMU_controller_address_1 = 1
IMU_controller_address_2 = 2
IMU_subject_address_1 = 7
IMU_subject_address_2 = 3
# IMU_port = 'COM6'
IMU_port = '/dev/tty.usbmodem1421'
# stimulator_port = '/dev/tty.usbserial-HMQYVD6B'
stimulator_port = '/dev/tty.usbserial-HMCX9Q6D'
running = True
stimulation = True
filter_size = 5
angle_controller_1 = []
angle_controller_2 = []
angle_subject_1 = []
angle_subject_2 = []
counter = 0
pw_max = 500
current = [6, 6]


def read_sensors():
    global counter
    serial_port = serial.Serial(port=IMU_port, baudrate=115200, timeout=0.001)
    while running:
        bytes_to_read = serial_port.inWaiting()
        if bytes_to_read > 0:
            data = bytearray(serial_port.read(bytes_to_read))
            IMU_id = int(''.join('{:02x}'.format(data[6])))


            # angle
            b = ''.join(chr(i) for i in data[8:12])  # angle y
            ang = struct.unpack('>f', b)
            ang = ang[0]
            # print(x)

            # angle
            # b = ''.join(chr(i) for i in data[12:16])  # angle y
            # ang = struct.unpack('>f', b)
            # ang = ang[0]
            if ang >= 0:
                ang = (ang / math.pi) * 180
                ang = ang + 180
                # if abs(x) > (math.pi*0.75):
                #     ang = ang + 2*(90-ang)
            else:
                ang = 360 + ((ang / math.pi) * 180)
                ang = ang - 180
                # if abs(x) > (math.pi*0.75):
                #     ang = ang - 2*(ang-270)

            # print(ang)
            # print(ang)
            if IMU_id == IMU_controller_address_1:
                # print 'controller'
                angle_controller_1.append(ang)
                # if counter >= filter_size and ((ang > 60 and ang < 120) or (ang > 240 and ang < 300)):
                # angle_controller[-1] = numpy.mean(angle_controller[-filter_size:])
            elif IMU_id == IMU_controller_address_2:
                # print 'subject'
                angle_controller_2.append(ang)
                # if counter >= filter_size and ((ang > 60 and ang < 120) or (ang > 240 and ang < 300)):
                # angle_subject[-1] = numpy.mean(angle_subject[-filter_size:])
            elif IMU_id == IMU_subject_address_1:
                # print 'subject'
                angle_subject_1.append(ang)
                # if counter >= filter_size and ((ang > 60 and ang < 120) or (ang > 240 and ang < 300)):
                # angle_subject[-1] = numpy.mean(angle_subject[-filter_size:])
            elif IMU_id == IMU_subject_address_2:
                # print 'subject'
                angle_subject_2.append(ang)
                # if counter >= filter_size and ((ang > 60 and ang < 120) or (ang > 240 and ang < 300)):
                # angle_subject[-1] = numpy.mean(angle_subject[-filter_size:])
            # else:
            #     print 'neigher'

            # angle.append(ang)

            # print(angle[-1])

            counter += 1
    serial_port.close()


def control(angle_controller_1_in, angle_subject_1_in, angle_controller_2_in, angle_subject_2_in):
    kp = 5
    return [abs(-(angle_controller_1_in-angle_subject_1_in)*kp), abs(-(angle_controller_2_in-angle_subject_2_in)*kp)]


def main():
    while running:
        # print(angle_controller[-1],angle_subject[-1])
        control_signal = [0, 0]
        control_signal = control(angle_controller_1[-1], angle_subject_1[-1], angle_controller_2[-1], angle_subject_2[-1])
        # control_signal = round(control_signal)
        # print(control_signal)
        if control_signal[0] < 0:
            control_signal[0] = 0
        elif control_signal[0] > pw_max:
            control_signal[0] = pw_max
        if control_signal[1] < 0:
            control_signal[1] = 0
        elif control_signal[1] > pw_max:
            control_signal[1] = pw_max
        pw = control_signal
        # print(pw, current)
        if stimulation:
            stim.update(3, pw, current)
        # time.sleep(0.2)
    if stimulation:
        stim.stop()
        serialPortStimulator.close()


# IMU setup
dng_device = ts_api.TSDongle(com_port=IMU_port)
IMU_controller_1 = dng_device[IMU_controller_address_1]
IMU_controller_2 = dng_device[IMU_controller_address_2]
IMU_subject_1 = dng_device[IMU_subject_address_1]
IMU_subject_2 = dng_device[IMU_subject_address_2]
IMU_controller_1.setEulerAngleDecompositionOrder(0)
IMU_controller_1.setCompassEnabled(0)
IMU_controller_1.setFilterMode(1)
IMU_controller_1.setStreamingTiming(interval=0, delay=0, duration=0, timestamp=False)
IMU_controller_1.setStreamingSlots(slot0='getTaredOrientationAsEulerAngles', slot1='getNormalizedGyroRate')
IMU_controller_1.tareWithCurrentOrientation()
IMU_controller_1.startStreaming()
IMU_controller_2.setEulerAngleDecompositionOrder(0)
IMU_controller_2.setCompassEnabled(0)
IMU_controller_2.setFilterMode(1)
IMU_controller_2.setStreamingTiming(interval=0, delay=0, duration=0, timestamp=False)
IMU_controller_2.setStreamingSlots(slot0='getTaredOrientationAsEulerAngles', slot1='getNormalizedGyroRate')
IMU_controller_2.tareWithCurrentOrientation()
IMU_controller_2.startStreaming()
IMU_subject_1.setEulerAngleDecompositionOrder(0)
IMU_subject_1.setCompassEnabled(0)
IMU_subject_1.setFilterMode(1)
IMU_subject_1.setStreamingTiming(interval=0, delay=0, duration=0, timestamp=False)
IMU_subject_1.setStreamingSlots(slot0='getTaredOrientationAsEulerAngles', slot1='getNormalizedGyroRate')
IMU_subject_1.tareWithCurrentOrientation()
IMU_subject_1.startStreaming()
IMU_subject_2.setEulerAngleDecompositionOrder(0)
IMU_subject_2.setCompassEnabled(0)
IMU_subject_2.setFilterMode(1)
IMU_subject_2.setStreamingTiming(interval=0, delay=0, duration=0, timestamp=False)
IMU_subject_2.setStreamingSlots(slot0='getTaredOrientationAsEulerAngles', slot1='getNormalizedGyroRate')
IMU_subject_2.tareWithCurrentOrientation()
IMU_subject_2.startStreaming()
dng_device.close()

# Stimulator setup
if stimulation:
    serialPortStimulator = serial.Serial(stimulator_port, timeout=1, baudrate=115200)
    stim = stimulator.Stimulator(serialPortStimulator)
    stim.initialization(50, 3)


thread.start_new_thread(read_sensors, ())
time.sleep(0.1)
thread.start_new_thread(main, ())

while running:
    command = raw_input('Press ENTER to stop, a/s to increase current, z/x to decrease:')
    if command == 'a':
        current[0] += 2
    elif command == 's':
        current[1] += 2
    elif command == 'z':
        current[0] -= 2
        if current[0] < 0:
            current[0] = 0
    elif command == 'x':
        current[1] -= 2
        if current[1] < 0:
            current[1] = 0
    else:
        running = False
    print(current)
time.sleep(0.2)

dng_device = ts_api.TSDongle(com_port=IMU_port)
IMU_controller_1 = dng_device[IMU_controller_address_1]
IMU_controller_2 = dng_device[IMU_controller_address_2]
IMU_subject_1 = dng_device[IMU_subject_address_1]
IMU_subject_2 = dng_device[IMU_subject_address_2]
IMU_controller_1.stopStreaming()
IMU_controller_2.stopStreaming()
IMU_subject_1.stopStreaming()
IMU_subject_2.stopStreaming()
dng_device.close()


