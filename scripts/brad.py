import asyncio
import time
import sys, tty, termios

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakDBusError

UUID = "68a79627-2609-4569-8d7d-3b29fde28877"
MANUAL_CONTROL_UUID = "68a79628-2609-4569-8d7d-3b29fde28877"

async def write(client: BleakClient, msg_len: int, response: bool):
    try:
        msg = b'a' * msg_len
        for i in range(50):
            #start = time.time()
            await client.write_gatt_char(uuid, msg, response=response)
            #end = time.time()
            #print(end - start)
        await client.read_gatt_char(report_uuid)
    except BleakDBusError:
        print("failed, maybe a write command with too high MTU?")
    

async def main():
    print("Searching...")
    devices = await BleakScanner.discover()
    address = None
    for d in devices:
        if d.name == "Bradipograph":
            address = d.address
            break

    if address is None:
        print("Couldn't find it :(")
        return
    print(f"Found bradiograph at address {address}")
    
    async with BleakClient(address) as client:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)
        
        try:
            while True:
                key = sys.stdin.read(1)
                if len(key) != 1:
                    print(f"invalid input: `{key}`")
                    continue

                cmd = None
                if key == 'd':
                    cmd = 0
                elif key == 'f':
                    cmd = 1
                elif key == 'k':
                    cmd = 3
                elif key == 'j':
                    cmd = 4
                if cmd is not None:
                    await client.write_gatt_char(MANUAL_CONTROL_UUID, bytes([cmd]), response=False)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


asyncio.run(main())
