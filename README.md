# 3D LiDAR Room Scanner 📡

Welcome to the 3D LiDAR Room Scanner project! This system captures accurate 3D scans of indoor environments using a rotating Time-of-Flight sensor controlled by a microcontroller. Developed for **COMPENG 2DX3** at McMaster University, the project combines embedded systems and Python-based 3D visualization to produce interactive point clouds.

---

## Technologies Used

- **C (Keil IDE)**: For microcontroller programming and hardware control.
- **Python 3.9**: For data processing and visualization.
- **Open3D**: Renders real-time point cloud scans in 3D.
- **PySerial**: Enables UART communication between the MCU and PC.
- **MSP432E401Y Microcontroller**: Controls scanning logic and interfaces with sensors/motors.
- **VL53L1X ToF Sensor**: Provides distance measurements using infrared light.
- **28BYJ-48 Stepper Motor + ULN2003 Driver**: Rotates the sensor to capture a full 360° view.

---

## Features

- **Full 3D Scanning**:
  - Captures spatial data in multiple layers to create a detailed room scan.
  - Converts polar measurements into Cartesian coordinates for visualization.
- **Live Data Transmission**:
  - Uses UART to transmit distance data to the PC in real time.
- **Interactive 3D Visualization**:
  - Open3D displays point clouds that users can pan, rotate, and zoom.
- **Modular Scan Settings**:
  - Adjustable resolution (steps per rotation), number of layers, and vertical displacement.
- **Visual Feedback**:
  - Onboard LEDs indicate scan progress, UART activity, and status.

---

## How to Use

### Part 1: Hardware Setup
- Assemble the sensor mount with the VL53L1X and stepper motor.
- Connect the components to the MSP432E401Y microcontroller as per the schematic.
- Ensure all components are powered appropriately (3.3V for the sensor, 5V for the stepper).

---

### Part 2: Programming the Microcontroller

1. Launch **Keil µVision** and open the project file: `Final_Project.uvprojx`
2. Click `Translate` to compile the code.
3. Click `Build` to generate the hex output.
4. Click `Download` to flash the code onto the microcontroller.

---

### Part 3: Running the Scanning & Visualization Program

1. On your PC, open **Device Manager** and identify the **COM port** the microcontroller is connected to.
2. Open the Python script `3dvisual.py` in your preferred Python IDE.
3. Modify the configuration variables at the top of the script (commented as `CONFIG`):
   ```python
   num_slices = 3
   num_segments = 32
   z_step_height = 100
   ser = serial.Serial('COM4', 115200, timeout=5)
   ```
4. Run the program and wait for the terminal to show `waiting for PJ0 button press`.
5. Press the **PJ0** button on the board to begin the scanning sequence.
6. After each scan layer, **manually move the system along the Z-axis** to prepare for the next layer.
7. The program will continue scanning until all layers are completed.

---

### Part 4: Accessing the Visuals

- After the last scan, all the 3D points are saved to a file named `3Dvisual.xyz`.
- This file can be opened using **Open3D** or other third-party 3D tools for further analysis or export.
- A 3D viewer will launch automatically, displaying the full scan with connected points and layers.

---

## Project Files

- `Final_Project.c`: Microcontroller code for stepper and sensor control.
- `3dvisual.py`: Python script for serial data reading and visualization.
- `3Dvisual.xyz`: Output file with all scanned XYZ points.

---

## Future Improvements

- Add vertical motor control for automated Z-axis movement.
- Upgrade data transfer to USB-C or wireless for higher throughput.
- Add filtering and mesh generation for cleaner 3D models.
- Export point clouds directly to STL or PLY for CAD applications.

---

Enjoy scanning your world in 3D! 📡
