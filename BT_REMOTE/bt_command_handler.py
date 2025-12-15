import bleak
import asyncio
import struct
import json
from bleak import BleakClient, BleakScanner, BleakError
import sys
import time
from copy import deepcopy
from dataclasses import dataclass
from serial_handler import SerialHandler


COMMAND_LIST_FILE = "bt_commands.json"
DEVICE_LIST_FILE = "device_list.csv"

BASE_UUID = "7843b000-91f7-46a1-8795-173154692a6a"
NOTIFY_UUID = BASE_UUID.replace("b000", "b001")
SERVICE_UUID = BASE_UUID.replace("b000", "b002")

TARGET_DEVICE_NAME = "MagicWand_BLE"

HASH_VAL = 0x811C9DC5

MAX_COMMAND_COUNT = 15
MAX_COMMAND_LENGTH = 10

reset_flag = True

client = None

command_map = {
    "UP": 0,
    "DOWN": 1,
    "LEFT": 2,
    "RIGHT": 3,
}

command_list = []


@dataclass
class Command:
    name: str
    params: list
    index: int
    device: str
    command: str


if sys.platform.startswith("win"):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

SerialHandlerInstance = SerialHandler(port="COM20", baudrate=115200, timeout=1)

def create_hash_from_commands(command_list, hash_length):
    exist_block_length = (MAX_COMMAND_COUNT + 7) // 8
    command_hash_length = hash_length
    command_exists_block = [0]*exist_block_length # BLOCK SIZE = 16
    command_hash = [0]*command_hash_length # HASH SIZE = 80
    

    offset = HASH_VAL
    prime = 0x93

    for i in range(command_hash_length):
        command_hash[i] = offset.to_bytes(4, 'big')[i % 4]

    param_buffer = bytearray()


    for command in command_list:
        if command.index < MAX_COMMAND_COUNT:
            block_index = command.index // 8
            bit_index = command.index % 8
            command_exists_block[block_index] |= (1 << bit_index)

            for i, param in enumerate(command.params):
                if i < MAX_COMMAND_LENGTH:
                    global_param_index = command.index * MAX_COMMAND_LENGTH + i

                    hash_idx = global_param_index % command_hash_length #해시가 커맨드 인덱스에 따라 몰리는 문제가 있음. 나중에 고치기

                    mix_val = param ^ (global_param_index &0xFF)

                    new_val = (command_hash[hash_idx] ^ mix_val) * prime

                    command_hash[hash_idx] = new_val & 0xFF


    param_buffer.append(exist_block_length)
    param_buffer.append(command_hash_length)

    for byte in command_exists_block:
        param_buffer.append(byte)

    for byte in command_hash:
        param_buffer.append(byte)

    return param_buffer

def test_hash(command_list):

    hash_buffer = create_hash_from_commands(command_list, 10)

    print("Generated Hash Buffer:", hash_buffer)


def load_commands_from_file(file_path):
    command_list = []
    with open(file_path, 'r') as f:
        data = json.load(f)
        for entry in data:
            pp = entry.get("params", [])
            ncmd = []
            for ps in pp:
                ncmd.append(command_map[ps])
            command = Command(
                name=entry.get("name", ""),
                params=ncmd,
                index=entry.get("index", 0),
                device=entry.get("device", ""),
                command=entry.get("command", "")
            )
            command_list.append(command)
    print(f"Loaded {len(command_list)} commands from {file_path}")
    return command_list

def send_something_to_device_by_serial(device, data):
    print(f"Sending data to device {device}: {data}")
    SerialHandlerInstance.send_command(data)

def create_command_packet(command):
    packet = bytearray()
    header = 0x00 | (len(command.params) + 2)
    packet.append(header)
    packet.append(command.index)
    for param in command.params:
        packet.append(param)
    return packet

def notification_handler(sender, data):
    print(f"Notification from {sender}: {data}")

    target_command = None
    for cmd in command_list:
        if cmd.index == data[0]:
            target_command = cmd
            break
    if target_command:
        print(f"Executing command: {target_command.name} with params {target_command.params}")
        send_something_to_device_by_serial(target_command.device, target_command.command)
    else:
        print(f"[Error] No command found for index {data[0]}")
    
async def send_commands_and_verify(client, command_list):
    print("Sending commands to device...")


    while True:

        for command in command_list:
            packet = create_command_packet(command)
            await client.write_gatt_char(SERVICE_UUID, packet)
            await asyncio.sleep(0.05)  # Small delay between commands
            print(f"Sent command: {command.name} Index: {command.index} Params: {command.params}")
        
        #일단 전달하고 확인한다
        await client.write_gatt_char(SERVICE_UUID, bytearray([0b00000001, 0b00000010])) # REQUEST VERIFY
        await asyncio.sleep(0.1)
        print("Requested command verification from device.")
        response = await client.read_gatt_char(SERVICE_UUID) # HASH
        print("Received verification response from device:", response)
        if response[0]|0b01111111 == 0b01111111 and len(response) >= 3 and response[1] == 0b00000001:
            device_hash = response[2:2+len(response)-2]
            local_hash = create_hash_from_commands(command_list , 10) # 해시는 디바이스 코드와 맞아야 하므로 일단 하드코딩

            append_len = len(device_hash) - len(local_hash)
            if append_len > 0:
                local_hash.extend([0]*append_len)
            elif append_len < 0:
                device_hash = device_hash[:len(local_hash)]
                

            if(device_hash == local_hash):
                print("Device hash matches local command hash. Commands sent successfully.")
                await client.write_gatt_char(SERVICE_UUID, bytearray([0b11000001])) # ACK 1
                break
            else:
                print("Hash mismatch! Device hash does not match local command hash.")
                print("DEVICE HASH:", device_hash)
                print("LOCAL HASH :", local_hash)
                await client.write_gatt_char(SERVICE_UUID, bytearray([0b00000001, 0b00000011])) # REQUEST RESET
                await asyncio.sleep(0.1)
                response = await client.read_gatt_char(SERVICE_UUID)
                print("Received response after requesting reset:", response)
                await asyncio.sleep(0.3) #재전송 시도
        else:
            print("Unexpected response from device after sending commands:", response)
            await client.write_gatt_char(SERVICE_UUID, bytearray([0b10000000])) # ERROR 0
            break # 이건 오류가 맞음
    


async def initial_handshake(client):
    print("Performing initial handshake with the device...")

    #SEND ACK 0 -> READY
    #IF ARDUINO HAS HASH -> GET HASH
    #IF ARDUINO REQ COMMANDS -> SEND COMMANDS -> CHECK HASH

    send_packet = bytearray([0b11000000]) #ACK 0

    await client.write_gatt_char(SERVICE_UUID, send_packet)
    await asyncio.sleep(0.1)
    print("Sent initial ACK packet to device.")

    response = await client.read_gatt_char(SERVICE_UUID)

    if response and response[0] == 0b11000000:
       await send_commands_and_verify(client, command_list)
    elif response[0]&0b01000000 != 0 and len(response) >= 3 and response[1] == 0b00000001:
        self_hash = create_hash_from_commands(command_list , 10)

        if(response[2:2+len(self_hash)] == self_hash):
            print("Device hash matches local command hash. No need to send commands.")
            await client.write_gatt_char(SERVICE_UUID, bytearray([0b11000001])) # ACK 1
            return 
        else:
            #send commands
            await send_commands_and_verify(client, command_list)
    else:
        print("Unexpected response from device during handshake:", response)
        await client.write_gatt_char(SERVICE_UUID, bytearray([0b10000000])) #ERROR 0
        return
        
    await client.write_gatt_char(SERVICE_UUID, bytearray([0b11000001])) # ACK 1
    print("Handshake completed successfully.")

async def run():
    global client
    global reset_flag

    print("Scanning for devices...")

    device = await bleak.BleakScanner.find_device_by_name(TARGET_DEVICE_NAME)

    if not device:
        print("Device not found.")
        reset_flag = True
        return
    
    print(f"Found device: {device.name}, {device.address} Connecting...")

    async with BleakClient(device, disconnected_callback=disconnected_handler) as client:
        print(f"Connected to {device.address}")

        #await client.start_notify(NOTIFY_UUID, notification_handler)
        #첫 연결 시 command 전송 및 핸드셰이크

        await initial_handshake(client)
        try:
            await client.start_notify(NOTIFY_UUID, notification_handler)
            print(f"Listening for gestures on {NOTIFY_UUID}...")
        except Exception as e:
            print(f"Failed to start notification: {e}")
            return
        #이제 연결이 끊길 때까지 명령어 대기
        #콜백 설정


        while client.is_connected:
            await asyncio.sleep(1)

        print("Connection closed. Try Reconnect")


def disconnected_handler(client: BleakClient):
    print("Device disconnected. Attempting to reconnect...")
    time.sleep(1)
    global reset_flag
    reset_flag = True



def main():
    global command_list
    global reset_flag
    try:
        command_list = load_commands_from_file(COMMAND_LIST_FILE)
    except Exception as e:
        print(f"Failed to load commands: {e}")
        print("USING TEST COMMANDS LIST")
        command_list = [
            Command(name="Move Up", params=[0], index=0, device="MagicWand_BLE", command="1 ON"),
            #Command(name="Move Down", params=[2], index=1, device="MagicWand_BLE", command="2 ON"),
            Command(name="Move Down Real", params=[1], index=2, device="TARGET_DEVICE_NAME", command="3 ON"),
            Command(name ="Move Right", params= [1,0], index=3, device="TARGET_DEVICE_NAME", command="RSET"),
            Command(name = "down donwn", params=[1,1], index=4, device="TARGET_DEVICE_NAME", command="DUMMY0"),
            #Command(name = "Move right fast", params=[3], index=5, device="TARGET_DEVICE_NAME", command="DUMMY1"),
        ]
    try:
        while True:
            if reset_flag:      
                reset_flag = False
                asyncio.run(run())
            else:
                break
    except Exception as e:
        print(f"An Exception occurred: {e}")
    

main()