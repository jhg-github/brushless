import serial
import time
import struct
import matplotlib.pyplot as plt 



def ParseBuffer(buffer, sub_fmt, n_samples):
    sub_size = struct.calcsize(sub_fmt)
    offset = 0
    result = []
    while offset < (n_samples*sub_size):
        result.append( struct.unpack_from(sub_fmt, buffer, offset)[0] )
        offset += sub_size
    return result


N_REVS = 1
N_STEP = 2
N_SAMPLES_PER_STEP = 1000
N_RECORD_SAMPLES = N_REVS*N_STEP*N_SAMPLES_PER_STEP
N_RECORD_SAMPLE_SIZE_BYTES = 2

FS = 20000


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
time.sleep(0.2) # added because it returns from serial.Serial before the port is really opened so the 
                # following functions wouldn't have any effect
ser.flushInput()
ser.flushOutput()

print("READING PHASE A")
serBuffer_phase_a = ser.read(N_RECORD_SAMPLES * N_RECORD_SAMPLE_SIZE_BYTES)
print('READING PHASE A BYTES:', len(serBuffer_phase_a))
ser.write(1)    # acknowledge
print("READING PHASE B")
serBuffer_phase_b = ser.read(N_RECORD_SAMPLES * N_RECORD_SAMPLE_SIZE_BYTES)
print('READING PHASE B BYTES:', len(serBuffer_phase_b))
ser.write(1)    # acknowledge
print("READING PHASE C")
serBuffer_phase_c = ser.read(N_RECORD_SAMPLES * N_RECORD_SAMPLE_SIZE_BYTES)
print('READING PHASE C BYTES:', len(serBuffer_phase_c))
ser.write(1)    # acknowledge
print("READING STEPS")
serBuffer_steps = ser.read(N_RECORD_SAMPLES)
print('READING STEPS BYTES:', len(serBuffer_steps))


phase_a = ParseBuffer(serBuffer_phase_a, '<H', N_RECORD_SAMPLES)
phase_b = ParseBuffer(serBuffer_phase_b, '<H', N_RECORD_SAMPLES)
phase_c = ParseBuffer(serBuffer_phase_c, '<H', N_RECORD_SAMPLES)
steps = ParseBuffer(serBuffer_steps, '<B', N_RECORD_SAMPLES)



t = [ i/FS for i in range(len(phase_a))]

plt.plot(t,phase_a, label='Phase A')
plt.plot(t,phase_b, label='Phase B')
plt.plot(t,phase_c, label='Phase C')
plt.plot(t,steps, label='Steps')
plt.tight_layout()
plt.legend()
plt.grid()
plt.show()