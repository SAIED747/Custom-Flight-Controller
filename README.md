




# 🚁 Engineering a Drone Flight Controller

> **A from-scratch STM32H7 flight controller developed in C, combining custom sensor drivers, real-time sensor fusion, attitude estimation, cascaded feedback control, and real-world flight validation.**

---

## Introduction

I worked on this project from **October 2025 to March 2026**. During this period, I developed a strong fascination with embedded systems and real-time control, which led me to take on the challenge of building a complete flight controller from the ground up.

<img width="1280" height="960" alt="WhatsApp Image 2026-04-14 at 16 53 59 (4)" src="https://github.com/user-attachments/assets/991084b5-c786-4c8a-a8e2-5e92437d83a3" />

My goal was to implement a **perception-to-action flight system** on a real quadcopter using firmware entirely developed by me. The system starts from raw measurements coming from the onboard sensors, processes and filters those measurements, estimates the vehicle's state, computes the required control actions, and finally translates those actions into motor commands.

This project became a deep hands-on learning experience in:

* Embedded C programming
* STM32 microcontrollers and real-time systems
* Sensor interfacing and low-level drivers
* Digital signal processing and filtering
* Sensor fusion and state estimation
* Attitude representation and estimation
* Feedback control systems
* Cascaded PID control
* Motor mixing and actuator control
* Real-time scheduling and timing analysis
* System identification and experimental tuning

I learned an enormous amount throughout the process and genuinely enjoyed every stage of the engineering journey—from debugging individual SPI transactions to tuning feedback loops and finally seeing the complete system achieve stable flight on a real drone.

> **Note:** This project was developed with very limited financial and laboratory resources. Many design decisions were therefore driven by the hardware and tools that were realistically available to me. Rather than relying on an existing flight-control stack, I used these constraints as an opportunity to understand and implement the system at a much deeper level.

---

# 📌 About the Project

This repository contains the embedded firmware for a **custom quadcopter flight controller**, written entirely in **C** and developed for the **STM32H743VIT6** microcontroller using both **STM32 HAL and LL drivers**.

The project was designed as a self-directed systems-engineering exercise in:

> **Sensing → Filtering → State Estimation → Control → Actuation**

The objective was not simply to make a drone fly, but to understand and implement the complete real-time control pipeline required to make that possible.

The firmware includes custom low-level interfaces for the flight sensors, real-time sensor acquisition, signal processing, attitude estimation, cascaded feedback controllers, motor mixing, and PWM generation.

The system was ultimately **validated through real-world flight testing**.

---

# 🧠 System Architecture

The overall architecture can be summarized as:

```text
                    ┌─────────────────────┐
                    │       Sensors       │
                    │                     │
                    │  IMU / Barometer    │
                    │  Magnetometer / GPS │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │  Sensor Drivers     │
                    │                     │
                    │  SPI / I²C / UART   │
                    │  Register Access    │
                    │  Calibration        │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ Signal Processing   │
                    │                     │
                    │ LPF 1st/2nd-order   |
                    | Notch Filtering     │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │  State Estimation   │
                    │                     │
                    │ Attitude / Position │
                    │ Sensor Fusion       │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │  Attitude Control   │
                    │     Outer Loop      │
                    │                     │
                    │ Desired Attitude    │
                    │       ↓             │
                    │ Attitude Error      │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │    Rate Control     │
                    │     Inner Loop      │
                    │                     │
                    │ Desired Rate        │
                    │       ↓             │
                    │ Rate Error          │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │    Motor Mixer      │
                    │                     │
                    │ Roll / Pitch / Yaw  │
                    │ + Throttle          │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │    PWM Outputs      │
                    │                     │
                    │      ESCs           │
                    │       ↓             │
                    │      Motors         │
                    └─────────────────────┘
```

The important aspect of the architecture is that the flight controller is treated as a **closed-loop dynamical system**.

The controller continuously performs:

```text
Measure
   ↓
Estimate
   ↓
Compare with reference
   ↓
Compute control action
   ↓
Actuate motors
   ↓
Vehicle dynamics
   ↓
Measure again
```

This loop runs continuously in real time.

---

# 🔩 Hardware

## Flight Controller

| Component            | Description                |
| -------------------- | -------------------------- |
| MCU                  | **STM32H743VIT6**          |
| CPU                  | ARM Cortex-M7              |
| Firmware             | C                          |
| Development          | STM32CubeIDE / STM32CubeMX |
| Low-level interfaces | STM32 HAL + LL             |
| Debugging            | STM Studio                 |
| IMU                  | ICM-45686                  |
| Barometer            | DPS310                     |
| GPS                  | u-blox M8N                 |
| Magnetometer         | QMC5883P                   |
| RC Receiver          | FlySky FS-iA6B / iBUS      |
| Motors               | ReadyToSky RS-2212 920KV   |
| Propellers           | 10-inch, 2-blade           |
| ESC                  | 400 Hz capable ESCs SIMONK |
| Battery              | 3S LiPo (8000mA)           |

---

# 📡 Sensor Layer

One of the main objectives of the project was to avoid treating sensors as black boxes.

I implemented the sensor interfaces and communication logic myself, working directly with device registers and communication protocols.

## IMU — ICM-45686

The IMU provides the fundamental measurements required for attitude and angular-rate control:

```text
Accelerometer
      +
Gyroscope
      ↓
Raw measurements
      ↓
Calibration
      ↓
Filtering
      ↓
State estimation
      ↓
Flight control
```

The IMU communicates with the STM32 through **SPI**.

The driver handles:

* Device initialization
* Register configuration
* WHO_AM_I verification
* Accelerometer configuration
* Gyroscope configuration
* Output data rate configuration
* Full-scale configuration
* Raw register acquisition
* Sensor conversion
* Calibration
* Real-time measurement acquisition

The IMU was operated at high sampling rates to provide sufficient bandwidth for the inner attitude-rate control loop.

---

# 🌡️ Barometer — DPS310

The DPS310 provides pressure measurements that can be used for altitude estimation.

The processing chain is conceptually:

```text
Pressure
   ↓
Temperature compensation
   ↓
Pressure filtering
   ↓
Altitude estimation
   ↓
State estimation
```

---

# 🧭 Magnetometer — QMC5883P

The magnetometer provides measurements of the Earth's magnetic field and can be used to estimate heading.

The magnetometer is particularly useful for reducing long-term yaw drift when combined with gyroscope measurements in a sensor-fusion framework.

---

# 🛰️ GPS — u-blox M8N

The GPS provides global position and velocity information.

It can be used by the state-estimation layer for:

* Position estimation
* Velocity estimation
* Long-term drift correction
* Navigation
* Future autonomous-flight functionality

The GPS operates at a much lower update rate than the IMU, so the system treats GPS as a **low-rate measurement source** within the broader real-time estimation architecture.

---

# 🎮 RC Input

The FlySky FS-iA6B receiver provides pilot commands through **iBUS**.

The received commands are converted into control references such as:

```text
Throttle
Roll command
Pitch command
Yaw command
```

These references are then passed to the flight-control system.

---





# 📈 Digital Signal Processing

Raw IMU measurements are not directly suitable for a high-performance controller.

The measurements contain:

* Sensor noise
* Mechanical vibration
* Motor harmonics
* Structural resonance
* High-frequency disturbances

Therefore, signal processing is performed before measurements are used by the control system.


The project includes investigation and implementation of digital filters such as:

* First-order low-pass filters
* Second-order IIR low-pass filters
* Notch filtering
* Frequency-domain analysis

  Before low pass filter:
  <img width="1202" height="941" alt="1-LPF100FcBefore" src="https://github.com/user-attachments/assets/fea24edc-2b80-4baa-b0ad-ce29813e607e" />

  after low pass filter:
  <img width="1190" height="932" alt="2-LPF100FcAfter" src="https://github.com/user-attachments/assets/aeda4f9f-882f-4f8e-9281-c2b74cf79764" />

  Notchfilter 1st-2nd-3rd harmonics:
  <img width="1188" height="927" alt="6-After_Notch_1st_2nd__3rdHarmonic_WITH_LPF_2nd_Order" src="https://github.com/user-attachments/assets/fb777abb-e3e4-4702-807c-9330b0d4e6b1" />

  

---

# 🔊 Vibration and Frequency Analysis

One of the most interesting parts of the project was analyzing the vibration characteristics of the drone.

The IMU data was sampled at high frequency and analyzed using **FFT/RFFT techniques** using the CMSIS DPS library.

The signal-processing pipeline can be represented as:

```text
IMU
 ↓
Sampled vibration signal
 ↓
Window / preprocessing
 ↓
FFT
 ↓
Frequency spectrum
 ↓
Identify dominant frequencies
 ↓
Design appropriate filter
```



to be investigated experimentally.

The frequency-domain analysis was particularly useful for identifying unwanted vibration components and designing notch/low-pass filtering strategies.

---

# 🧮 State Estimation and Sensor Fusion

A flight controller cannot directly measure every state required for control.

Instead, multiple noisy sensors must be combined to estimate the vehicle state.

The project therefore investigates **multi-sensor state estimation and sensor fusion**.

A simplified model is:

```text
             Gyroscope
                 │
                 ▼
          Angular dynamics
                 │
                 │
Accelerometer ───┼───► State Estimator
                 │
Magnetometer ────┤
                 │
GPS ─────────────┤
                 │
Barometer ───────┘
                 │
                 ▼
          Estimated state
```

The estimator combines information from sensors with different characteristics:

| Sensor        | Strength                                 | Limitation                       |
| ------------- | ---------------------------------------- | -------------------------------- |
| Gyroscope     | Excellent short-term angular information | Drift                            |
| Accelerometer | Gravity reference                        | Sensitive to linear acceleration |
| Magnetometer  | Heading reference                        | Magnetic disturbances            |
| GPS           | Global position/velocity                 | Low rate and noisy               |
| Barometer     | Altitude information                     | Pressure disturbances            |

The central idea is that no single sensor provides a complete and reliable estimate by itself.

---

# 🧭 Attitude Estimation

Attitude estimation is one of the most important parts of the flight controller.

The system needs to estimate the orientation of the vehicle from inertial measurements.

The project explores quaternion-based attitude representation using the MADGWICK ALGORTHEM and sensor-fusion techniques.

A quaternion can be represented as:

```text
q = [q₀  q₁  q₂  q₃]ᵀ
```

and is used to represent the orientation without the singularities associated with Euler-angle representations.

The estimated attitude can subsequently be converted into quantities such as:

```text
Roll
Pitch
Yaw
```

for visualization, reference generation, and control.

---

# 🎯 Cascaded Flight Control

The control architecture is based on a **cascaded feedback structure**.

Instead of directly commanding motor outputs from an attitude error, the system separates the problem into two control loops.

```text
                Desired Attitude
                       │
                       ▼
              ┌─────────────────┐
              │  Outer Loop     │
              │ Attitude PID    │
              └────────┬────────┘
                       │
                 Desired Rate
                       │
                       ▼
              ┌─────────────────┐
              │  Inner Loop     │
              │   Rate PID      │
              └────────┬────────┘
                       │
                 Control Torque
                       │
                       ▼
                 Motor Mixer
                       │
                       ▼
                    Motors
```

---

# 🔄 Outer Attitude Loop

The outer loop compares the desired attitude with the estimated attitude:

```text
Attitude reference
        -
Estimated attitude
        ↓
Attitude error
        ↓
Attitude controller
        ↓
Desired angular rate
```

For example:

```text
θ_error = θ_reference - θ_estimated
```

The controller converts this attitude error into a desired angular velocity.

This creates a natural separation between:

* **Where the vehicle should point**
* **How quickly the vehicle should rotate**

---

# ⚡ Inner Rate Loop

The inner loop operates directly on gyroscope measurements.

```text
Desired angular rate
        -
Measured angular rate
        ↓
Rate error
        ↓
Rate controller
        ↓
Control torque
```

Because the gyroscope provides high-rate measurements, the rate loop can operate at a substantially higher frequency than slower navigation processes.

The inner loop is therefore responsible for rejecting disturbances and rapidly controlling the rotational dynamics of the vehicle.





# 🔀 Motor Mixing

The controller does not directly command individual motors from the PID outputs.

Instead, the desired control actions are converted into motor commands through a mixer.

The controller generates:

```text
Throttle
Roll
Pitch
Yaw
```

which are transformed into individual motor commands.

For a conventional quadcopter:

```text
              Front
               ↑

          M1       M2
            \     /
             \   /
              \ /
              / \
             /   \
            /     \
          M4       M3
```

The mixer combines the collective throttle command with the required roll, pitch, and yaw corrections.

Conceptually:

$$
M_i = T + R_i + P_i + Y_i
$$

where:

* \(T\) = throttle contribution
* \(R_i\) = roll contribution
* \(P_i\) = pitch contribution
* \(Y_i\) = yaw contribution

The resulting values are constrained to the valid actuator range before being sent to the ESCs.

---

# ⚡ ESC and PWM Generation

The final motor commands are converted into PWM signals generated by STM32 timers.

The signal chain is:

```text
Controller
    ↓
Motor Mixer
    ↓
Output Limiting
    ↓
PWM Compare Registers
    ↓
ESC
    ↓
Brushless Motor
    ↓
Propeller
    ↓
Vehicle Dynamics
```

The ESCs receive the PWM commands and regulate motor speed accordingly.

This closes the complete control loop:

```text
Sensor
  ↓
State estimation
  ↓
Controller
  ↓
Mixer
  ↓
PWM
  ↓
ESC
  ↓
Motor
  ↓
Propeller
  ↓
Drone dynamics
  ↓
Sensor
  ↺
```

---

# 🧰 Development Methodology

The project was developed incrementally rather than attempting to implement the complete flight controller at once.

A simplified development progression was:

```text
STM32 initialization
        ↓
SPI / I²C / UART communication
        ↓
Individual sensor drivers
        ↓
Raw sensor validation
        ↓
Calibration
        ↓
Sensor filtering
        ↓
Attitude estimation
        ↓
Rate controller
        ↓
Motor mixer
        ↓
PWM / ESC control
        ↓
Inner-loop flight testing
        ↓
Outer-loop attitude control
        ↓
Controller tuning
        ↓
Stable flight
```

Each layer was tested before being integrated into the next.

This approach made it possible to isolate problems and distinguish between:

* Hardware problems
* Driver problems
* Timing problems
* Sensor problems
* Estimation problems
* Control problems
* Mechanical problems

---





# 🔭 Future Work

The project provides a foundation for further experimentation in autonomous aerial robotics.

Potential future directions include:

### Advanced State Estimation

Development of a more complete EKF-based estimator incorporating:

```text
IMU
+
GPS
+
Barometer
+
Magnetometer
        ↓
   Full Vehicle State
```

### Advanced Control

Investigation of nonlinear control techniques such as:

* Sliding Mode Control
* Nonlinear attitude control
* Model-based control
* Adaptive control
* Robust control

### Autonomous Flight

Extending the controller from manual stabilization toward:

```text
Attitude Hold
      ↓
Altitude Hold
      ↓
Position Hold
      ↓
Waypoint Navigation
      ↓
Trajectory Tracking
```

### Computer Vision

A future perception layer could provide visual information for tasks such as object tracking and visual servoing.

This would extend the architecture toward:

```text
Camera
  ↓
Computer Vision
  ↓
Target Estimation
  ↓
Desired Motion
  ↓
Flight Controller
  ↓
Drone
```

---


# 🛠️ Software & Tools

The project was developed using:

* **C**
* **STM32CubeIDE**
* **STM32CubeMX**
* **STM32 HAL**
* **STM32 LL**
* **CMSIS**
* **CMSIS-DSP**
* **Git / GitHub**
* **ST-LINK**
* Oscilloscope / measurement tools where available
* MATLAB / Octave for signal-processing and control analysis

---



# 👨‍💻 Author

Developed as a self-directed embedded systems and control engineering project.

**Mohammad Saied**

Electrical & Electronics Engineering
Interests: **Control Systems · Embedded Systems · Robotics · Signal Processing · Autonomous Systems**

