import bleak
import asyncio
import struct
import json
from bleak import BleakClient, BleakScanner, BleakError
import sys
import time
from copy import deepcopy

import argparse


t = 0
timers = None
stream = None
targetIndex = -1

indexs = 0

if sys.platform.startswith("win"):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

target_name = "MagicWand_BLE"

base_uuid = "7843b000-91f7-46a1-8795-173154692a6a"
sensor_uuid = base_uuid.replace("b000", "b001")

byte_buffer = bytearray() # SIZE

def jsonWrite(byte_frame, index, target_label, jsonStream):
    
    internal_index = 0
    frameA = byte_frame[0::2]
    frameB = byte_frame[1::2]

    #frame별 분리 처리

    TARGET_FRAME_SIZE = 30
    SAMPLE_FRAME_SIZE = 50
    SAMPLE_HOP = 2

    for i in range(0, len(frameA) - TARGET_FRAME_SIZE + 1, SAMPLE_HOP):
        json_obj = {
            "label": target_label,
            "index": index + internal_index,
            "frame": frameA[i:i+TARGET_FRAME_SIZE]
        }
        internal_index += 1
        jsonStream.write(json.dumps(json_obj) + "\n")
        json_obj = {
            "label": target_label,
            "index": index + internal_index,
            "frame": frameB[i:i+TARGET_FRAME_SIZE]
        }
        internal_index += 1
        jsonStream.write(json.dumps(json_obj) + "\n")

    return internal_index
        



    


async def run():

    global byte_buffer
    global t
    global timers

    devices = await BleakScanner.discover()
    target_device = None
    for d in devices:
        if d.name and target_name in d.name:  # Replace with actual device name
            target_device = d
            break

    if not target_device:
        print("Target device not found.")
        return
    print("Target device found:", target_device.name, target_device.address)
    print(target_device.details)


    async with BleakClient(target_device.address, timeout=None, winrt={"use_cached_services": False}) as client:
        print(f"Connected to {target_device.name} - {target_device.address}")
        try:
            await client.start_notify(sensor_uuid, notification_handler)
            print("Notification started. Listening for data...")
            while True:
                await asyncio.sleep(1)
                if timers is not None and (time.time() - timers) > 2:
                    print("⚠️ Timeout: Incomplete data received within 2 seconds.")
                    byte_buffer = bytearray()
                    t = 0
                    timers = None
            
        except Exception as e:
            await client.stop_notify(sensor_uuid)
            print("Notification stopped.")
            print(f"An error occurred: {e}")
def notification_handler(sender, data):
    global byte_buffer
    global t
    global timers
    global indexs
    
    if timers is None:
        timers = time.time() # Start time
    
    # 1. 들어오는 족족 버퍼에 붙이기 (20바이트씩 들어옴)
    byte_buffer.extend(data)
    t += 1
    print(f"Buffer Length: {len(byte_buffer)} bytes - Packet {t}")
    
    

    # 2. 600바이트가 다 찼는지 확인
    if len(byte_buffer) >= 600:
        
        frames = []
        for i in range(100): # 100개 프레임
            start = i * 6
            end = start + 6
            # 6바이트씩 잘라서 int8 6개로 변환
            frame = struct.unpack('<bbbbbb', byte_buffer[start:end])
            frames.append(frame)
        
        print("✅ Received 100 Frames")
        idxmovement = jsonWrite(frames, indexs, targetIndex, stream)
        print(f"Data written to {stream.name} -> {stream}  with label {targetIndex} and index {indexs}")    

        indexs += idxmovement
        
        # 5. 버퍼 초기화
        byte_buffer = bytearray()
        t = 0
        timers = None

    

def main():
    global stream
    parser = argparse.ArgumentParser(description="Bluetooth Data Collector")
    parser.add_argument('--output', type=str, default='data.json', help='Output JSON file name')
    parser.add_argument('--label', type=int, default=0, help='Target label for the data')

    args = parser.parse_args()

    stream = open(args.output, 'a', newline='')
    global targetIndex
    targetIndex = args.label
    
    try:
        asyncio.run(run())
    except Exception as e:
        print(f"An error occurred in main: {e}")
    finally:
        stream.close()


main()