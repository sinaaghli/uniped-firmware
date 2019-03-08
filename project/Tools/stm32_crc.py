# This is bade on the solution provided by Lynn Linse at:
# https://community.st.com/s/question/0D50X00009XkhnhSAB/crc32-on-f4f7


import struct


def stm32_crc(data):
    def step(crc_, data):
        crc_ = crc_ ^ data
        for i in range(0, 32):
            if crc_ & 0x80000000:
                crc_ = ((crc_ << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc_ = (crc_ << 1) & 0xFFFFFFFF
        return crc_
    padding = ((len(data) + 3)//4)*4 - len(data)
    values = struct.unpack(
        f'<{(len(data) + padding)//4}L', data + b'\x00'*padding)
    crc = 0xFFFFFFFF
    for value in values:
        crc = step(crc, value)
    return crc
