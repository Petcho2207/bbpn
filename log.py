import serial
import csv
from datetime import datetime
import re

PORT = 'COM3'
BAUD = 115200
FILENAME = f"masureTest_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

pending_voltage_line = None
pending_balancing_text = None

try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser, open(FILENAME, "w", newline='') as csvfile:

        writer = csv.writer(csvfile)
        header_written = False

        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            # ถ้าเป็นข้อมูลแรงดัน (เช่น เริ่มด้วย V[0][)
            if "V[0][" in line:
                pending_voltage_line = line

            # ถ้าเจอข้อความ balancing
            elif line.startswith("Selected Cells for Balancing:"):
                balancing_cells = re.findall(r'(\d+)\s*\(', line)
                pending_balancing_text = ",".join(balancing_cells) if balancing_cells else ""

            # ✅ เมื่อมีข้อมูลแรงดัน ให้เขียนลงไฟล์ (ไม่ต้องรอ balancing)
            if pending_voltage_line is not None:
                data_items = pending_voltage_line.split(",")

                if not header_written:
                    header = ["timestamp"] + [item.split(":")[0] for item in data_items if ":" in item]
                    header.append("Selected Cells for Balancing")
                    writer.writerow(header)
                    header_written = True

                row = [datetime.now().isoformat()]
                row += [item.split(":")[1] for item in data_items if ":" in item]
                row.append(pending_balancing_text if pending_balancing_text else "None")
                writer.writerow(row)
                print(f"Saved row: {row}")

                # reset ตัวแปร
                pending_voltage_line = None
                pending_balancing_text = None

except KeyboardInterrupt:
    print(f"\nStopped by user. Log saved to {FILENAME}")
