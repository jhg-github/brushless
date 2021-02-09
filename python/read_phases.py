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


N_REVS = 2
N_STEP = 6
N_SAMPLES_PER_STEP = 100
N_RECORD_SAMPLES = N_REVS*N_STEP*N_SAMPLES_PER_STEP
N_RECORD_SAMPLE_SIZE_BYTES = 2

FS = 20000


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
time.sleep(0.2) # added because it returns from serial.Serial before the port is really opened so the 
                # following functions wouldn't have any effect
ser.flushInput()
ser.flushOutput()

print("READING BEMF A")
serBuffer_bemf_a = ser.read(N_RECORD_SAMPLES * N_RECORD_SAMPLE_SIZE_BYTES)
print('READING BEMF A BYTES:', len(serBuffer_bemf_a))
ser.write(1)    # acknowledge
print("READING BEMF B")
serBuffer_bemf_b = ser.read(N_RECORD_SAMPLES * N_RECORD_SAMPLE_SIZE_BYTES)
print('READING BEMF B BYTES:', len(serBuffer_bemf_b))
ser.write(1)    # acknowledge
print("READING BEMF C")
serBuffer_bemf_c = ser.read(N_RECORD_SAMPLES * N_RECORD_SAMPLE_SIZE_BYTES)
print('READING BEMF C BYTES:', len(serBuffer_bemf_c))
ser.write(1)    # acknowledge
print("READING STEPS")
serBuffer_steps = ser.read(N_RECORD_SAMPLES)
print('READING STEPS BYTES:', len(serBuffer_steps))


bemf_a = ParseBuffer(serBuffer_bemf_a, '<H', N_RECORD_SAMPLES)
bemf_b = ParseBuffer(serBuffer_bemf_b, '<H', N_RECORD_SAMPLES)
bemf_c = ParseBuffer(serBuffer_bemf_c, '<H', N_RECORD_SAMPLES)
steps = ParseBuffer(serBuffer_steps, '<B', N_RECORD_SAMPLES)



t = [ i/FS for i in range(len(bemf_a))]

plt.plot(t,bemf_a, label='BEMF A')
plt.plot(t,bemf_b, label='BEMF B')
plt.plot(t,bemf_c, label='BEMF C')
# plt.plot(t,steps, label='Steps')
plt.tight_layout()
plt.legend()
plt.grid()
plt.show()