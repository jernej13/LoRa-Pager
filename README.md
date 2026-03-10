# LoRa Pager (ESP32)

This repository contains my **high school graduation project (Slovenian *matura*)**, where I designed and built a **portable LoRa-based communication device** capable of long-range peer-to-peer messaging without relying on existing network infrastructure.

The device is built around an **ESP32 microcontroller** and uses **LoRa radio communication** to send predefined messages between devices over long distances.

The project includes **hardware design, embedded firmware, and a 3D printed enclosure**.

<p align="center">
  <img src="pictures/epicpicture.jpg" width="500">
</p>

---

## Features

* Long-range **LoRa peer-to-peer communication**
* **GNSS location support**
* **OLED graphical interface**
* Menu-based navigation with buttons
* **Emergency beacon mode**
* Battery monitoring and low-power operation
* Custom **3D printed enclosure**

---

## Hardware

Main components:

* ESP32 microcontroller
* Semtech **SX1276 LoRa transceiver**
* **U-blox NEO-M8N GNSS module**
* 128×64 **OLED display**
* Buttons and piezo buzzer
* Li-ion battery with power management

Development board used:

* **LilyGO TTGO T-Beam V1.1**

---

## Software

The firmware is written in **C++ using the Arduino framework** and developed with **PlatformIO**.

The program manages:

* LoRa communication
* GNSS data processing
* graphical user interface
* power management

---

## Results

Field tests achieved communication distances of **over 9 km** using LoRa with a transmission power of **0.1 W**.

---

## Repository Contents

```
firmware/     ESP32 source code
cad/          3D enclosure models
docs/         project report
hardware/     schematics
```

---

## Author

**Jernej Leskovec**
High School Graduation Project (*Matura*)
Ljubljana, Slovenia

