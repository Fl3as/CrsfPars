#!/usr/bin/env python3
import serial
import time
import argparse

CRSF_SYNC = 0xC8
RC_CHANNELS_PACKED = 0x16

def crc8_dvb_s2(crc, a) -> int:
  crc = crc ^ a
  for ii in range(8):
    if crc & 0x80:
      crc = (crc << 1) ^ 0xD5
    else:
      crc = crc << 1
  return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

def unpack_channel_values(data: bytearray) -> list:
    """
    Skips first 3 bytes, then unpacks 16 11-bit channels from the next 22 bytes.
    
    Args:
        data (bytearray): Bytearray containing at least 25 bytes (3 skip + 22 data)
        
    Returns:
        list: List of 16 unsigned integers, each representing an 11-bit channel value
    """
    working_data = data[3:25]
    
    result = []
    bit_position = 0
    
    for i in range(16):
        # Calculate which bytes we need to read from
        byte_index = bit_position // 8
        bit_offset = bit_position % 8
        
        # Read enough bytes to cover our 11 bits
        value = 0
        for j in range(3):  # We might need up to 3 bytes to get our 11 bits
            if byte_index + j < len(working_data):
                value |= working_data[byte_index + j] << (8 * j)
        
        # Shift right to align to start of our 11 bits and mask to get only 11 bits
        value = (value >> bit_offset) & 0x7FF
        result.append(value)
        bit_position += 11
    
    return result

def pack_channels(channels: list) -> bytearray:
    """
    Packs 16 11-bit channel values into 22 bytes.
    
    Args:
        channels (list): List of 16 integers, each representing an 11-bit channel value (0-2047)
        
    Returns:
        bytearray: 22 bytes containing the packed 11-bit values
    """
    if len(channels) != 16:
        raise ValueError("Must provide exactly 16 channel values")
        
    # Validate channel values
    for i, value in enumerate(channels):
        if not 0 <= value <= 2047:  # 2047 is max value for 11 bits (0x7FF)
            raise ValueError(f"Channel {i+1} value {value} exceeds 11-bit range (0-2047)")
    
    result = bytearray(22)  # Initialize empty 22-byte array
    bit_position = 0
    
    for value in channels:
        # Calculate which bytes we need to write to
        byte_index = bit_position // 8
        bit_offset = bit_position % 8
        
        # Shift value to its bit position
        shifted_value = value << bit_offset
        
        # Write the value across the necessary bytes
        for j in range(3):  # Might need up to 3 bytes per value
            if byte_index + j < len(result):
                # Mask out the bits we're about to write
                mask = (0xFF << bit_offset if j == 0 else 0xFF) if j < 2 else 0xFF
                # Clear the bits we're about to write
                result[byte_index + j] &= ~mask
                # Write the new bits
                result[byte_index + j] |= (shifted_value >> (8 * j)) & 0xFF
        
        bit_position += 11
    return result

def generate_frame(roll : int, pitch : int, yaw : int, throttle : int, armed : int, custom : int) -> bytes:
    channeldata = [roll, pitch, yaw, throttle, armed, custom]
    for i in range(6,16):
      channeldata.append(992)
    result = [CRSF_SYNC,24,RC_CHANNELS_PACKED]
    result += pack_channels(channeldata)
    result.append(crc8_data(result[2:]))
    return result

#Switch between microseconds and 11-bit Value and vise versa
def get_us(crsf) -> int:
    return int(1500 + (5/8 * (crsf - 992)))
def get_crsf(us) -> int:
    return int(992 + (8/5 * (us - 1500)))



parser = argparse.ArgumentParser()
parser.add_argument('-P', '--port', default='/dev/ttyAMA0', required=False)
parser.add_argument('-b', '--baud', default=420000, required=False)
args = parser.parse_args()

with serial.Serial(args.port, args.baud, timeout=2) as ser:
    while True:
        # Receive a Frame and decode it
        input = bytearray()
        if ser.in_waiting > 0:
            input.extend(ser.read(ser.in_waiting))
        else:
            time.sleep(0.020)

        if len(input) > 2:
            expected_len = input[1] + 2
            if crsf_validate_frame(input[:expected_len]):
                    if input[2] == RC_CHANNELS_PACKED:
                        received = unpack_channel_values(input)
                        print(received)
                        # received_us = []
                        # for i in range(len(received)):
                        #     us = get_us(received[i])
                        #     received_us.append(us)
                        # print(f"Microseconds:{received_us}")
        # Generate a frame and send it
        frame = generate_frame(111,222,333,444,555,666)
        ser.write(frame)
        time.sleep(0.020)
