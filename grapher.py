import re
import matplotlib.pyplot as plt

# ---------- Load File ----------
with open("grapher_test.txt", "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

# ---------- Storage ----------
imu16_time = []
imu16_gx = []
imu16_gy = []
imu16_gz = []
imu16_ax = []
imu16_ay = []
imu16_az = []

imu32_time = []
imu32_gx = []
imu32_gy = []
imu32_gz = []
imu32_ax = []
imu32_ay = []
imu32_az = []

baro07_time = []
baro07_pressure = []

baro11_time = []
baro11_pressure = []

# ---------- Regex ----------
# Minimal dump format: type[data] timestamp
# type ids match LoggingData in Log.hpp: 0=IMU32G 1=IMU16G 2=MAG 3=BARO07 4=BARO11 5=GPS
IMU32G, IMU16G, MAG, BARO07, BARO11, GPS = range(6)

record_pattern = re.compile(r"\b(\d+)\[([^\]]*)\]\s*(\d+)")

# ---------- Parse ----------
for line in lines:
    match = record_pattern.search(line)
    if not match:
        continue

    sensor = int(match.group(1))
    ts = int(match.group(3))

    # GPS data is a sentence, not numbers
    if sensor == GPS:
        continue

    try:
        values = [int(v) for v in match.group(2).split(",")]
    except ValueError:
        continue

    # IMU data: ax,ay,az,gx,gy,gz,temp
    if sensor in (IMU16G, IMU32G) and len(values) >= 6:
        ax, ay, az, gx, gy, gz = values[:6]

        if sensor == IMU16G:
            imu16_time.append(ts)
            imu16_ax.append(ax)
            imu16_ay.append(ay)
            imu16_az.append(az)
            imu16_gx.append(gx)
            imu16_gy.append(gy)
            imu16_gz.append(gz)

        else:
            imu32_time.append(ts)
            imu32_ax.append(ax)
            imu32_ay.append(ay)
            imu32_az.append(az)
            imu32_gx.append(gx)
            imu32_gy.append(gy)
            imu32_gz.append(gz)

    # Baro data: pressure,temp
    elif sensor in (BARO07, BARO11) and len(values) >= 1:
        pressure = values[0]

        if sensor == BARO07:
            baro07_time.append(ts)
            baro07_pressure.append(pressure)

        else:
            baro11_time.append(ts)
            baro11_pressure.append(pressure)

# ---------- IMU16G Gyro ----------
plt.figure()
plt.plot(imu16_time, imu16_gx)
plt.plot(imu16_time, imu16_gy)
plt.plot(imu16_time, imu16_gz)
plt.title("IMU16G Gyro vs Time")
plt.xlabel("Timestamp")
plt.ylabel("Gyro")
plt.show()

# ---------- IMU16G Accel ----------
plt.figure()
plt.plot(imu16_time, imu16_ax)
plt.plot(imu16_time, imu16_ay)
plt.plot(imu16_time, imu16_az)
plt.title("IMU16G Accel vs Time")
plt.xlabel("Timestamp")
plt.ylabel("Accel")
plt.show()

# ---------- IMU32G Gyro ----------
plt.figure()
plt.plot(imu32_time, imu32_gx)
plt.plot(imu32_time, imu32_gy)
plt.plot(imu32_time, imu32_gz)
plt.title("IMU32G Gyro vs Time")
plt.xlabel("Timestamp")
plt.ylabel("Gyro")
plt.show()

# ---------- IMU32G Accel ----------
plt.figure()
plt.plot(imu32_time, imu32_ax)
plt.plot(imu32_time, imu32_ay)
plt.plot(imu32_time, imu32_az)
plt.title("IMU32G Accel vs Time")
plt.xlabel("Timestamp")
plt.ylabel("Accel")
plt.show()

# ---------- BARO07 ----------
plt.figure()
plt.plot(baro07_time, baro07_pressure)
plt.title("BARO07 Pressure vs Time")
plt.xlabel("Timestamp")
plt.ylabel("Pressure")
plt.show()

# ---------- BARO11 ----------
plt.figure()
plt.plot(baro11_time, baro11_pressure)
plt.title("BARO11 Pressure vs Time")
plt.xlabel("Timestamp")
plt.ylabel("Pressure")
plt.show()