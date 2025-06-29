import asyncio
from bleak import BleakScanner, BleakClient
import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation as R
import struct

NUS_DEVICE_NAME = 'theadmouse'

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX_CHAR_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'  # device → client (notify)

# Plane mesh
# Body (long thin center fuselage)
body = pv.Box(bounds=(-0.5, 0.5, -0.05, 0.05, -0.05, 0.05))

# Wings (wide flat box across Y)
wings = pv.Box(bounds=(-0.2, 0.2, -0.5, 0.5, -0.01, 0.01))

# Tail fin (small vertical stabilizer at back)
tail = pv.Box(bounds=(-0.05, 0.05, -0.01, 0.01, 0.0, 0.2))
tail.translate((-0.45, 0.0, 0.05))  # place at rear-top of body
tail.points += np.array([-0.45, 0.0, 0.05])

# Combine parts into one mesh
mesh = body + wings + tail

# Axis display
x_axis = pv.Arrow(start=(0, 0, 0), direction=(1, 0, 0), tip_length=0.025, tip_radius=0.015, shaft_radius=0.0015)
y_axis = pv.Arrow(start=(0, 0, 0), direction=(0, 1, 0), tip_length=0.025, tip_radius=0.015, shaft_radius=0.0015)
z_axis = pv.Arrow(start=(0, 0, 0), direction=(0, 0, 1), tip_length=0.025, tip_radius=0.015, shaft_radius=0.0015)


# Set up the plotter
plotter = pv.Plotter()
actor = plotter.add_mesh(mesh, color='skyblue')
actor_x = plotter.add_mesh(x_axis, color='red')
actor_y = plotter.add_mesh(y_axis, color='green')
actor_z = plotter.add_mesh(z_axis, color='blue')
plotter.add_axes()
plotter.show(auto_close=False, interactive_update=True)


async def main():
    print('🔍 Scanning for BLE devices advertising NUS...')
    devices = await BleakScanner.discover()
    nus_device = None

    for d in devices:
        if d.name and NUS_DEVICE_NAME in d.name:
            nus_device = d
            break

    if not nus_device:
        print('❌ Could not find NUS device.')
        return

    print(f'✅ Found device: {nus_device.name} [{nus_device.address}]')

    async with BleakClient(nus_device) as client:
        print('🔗 Connected. Waiting for serial data...\n')

        def handle_rx(_, data):
            q_fixed = struct.unpack('<hhhh', data)
            q = [x / 10000.0 for x in q_fixed]
            rot = R.from_quat([q[1], q[2], q[3], q[0]])
            rot_matrix = rot.as_matrix()

            print(f'Quaternion: {q}')

            # Apply rotation to the mesh
            transformed = mesh.copy()
            transformed.points = mesh.points @ rot_matrix.T
            actor.mapper.SetInputData(transformed)
            plotter.update()

        await client.start_notify(NUS_RX_CHAR_UUID, handle_rx)
        while True:
            await asyncio.sleep(1)

if __name__ == '__main__':
    asyncio.run(main())
