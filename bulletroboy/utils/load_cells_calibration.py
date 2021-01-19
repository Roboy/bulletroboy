import os
from datetime import datetime
from load_cell import LoadCell

GRAVITY = 9.81

print("Load Cells calibration...\n")

phidget_serial = int(input("Phidget Serial: "))

load_cells = [LoadCell({'tendon_id': i, 'cal_offset': None, 'cal_factor': None, 'serial': phidget_serial, 'channel': i }) for i in range(4)]

weight_1 = float(input("\nFirst weight in Kg: "))
weight_2 = float(input("Second weight in Kg: "))
weights = [0.0, weight_1, weight_2]

for load_cell in load_cells:
    print(f"\nCalibrating load cell {load_cell.id}...")
    load_cell.openChannel()

    measurements = []
    for w in weights:
        input(f"Apply {w} Kg to the load cell and press enter...")
        measurements.append(load_cell.getVoltageRatio())

    load_cell.cal_offset = measurements[0]
    load_cell.cal_factor = -(weight_2 - weight_1) * GRAVITY / (measurements[2] - measurements[1])

calibration_report =    "# Force (N) = cal_factor * (VoltageRatio + cal_offset)\n" \
                        f"phidget_serial: {phidget_serial}\n" \
                        f"channels: {[load_cell.getChannel() for load_cell in load_cells]}\n" \
                        f"cal_offset: {[load_cell.cal_offset for load_cell in load_cells]}\n" \
                        f"cal_factor: {[load_cell.cal_factor for load_cell in load_cells]}"

print("\nCalibration values:\n")
print(calibration_report)

with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'cal_values.yaml'), 'a+') as f:
    print(f"\nCalibration datetime: {datetime.now()}", file=f)
    print(calibration_report, file=f)

for load_cell in load_cells:
    load_cell.closeChannel()