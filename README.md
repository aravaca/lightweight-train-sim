# Super Express trainsim: Lightweight Train Simulator (PTS)

 
[Try PTS!⛅️ → Play full screen on PC](https://jrtasc.kro.kr/) 
---
<img width="1913" height="1077" alt="jrets" src="https://github.com/user-attachments/assets/7ee236d8-8310-46d8-9eaf-679de23e2ee2" />


A lightweight **web-based 3D railway simulation** replicating key control functions of **JR East/West and Korean EMUs**.  
Unlike heavy simulators, PTS is designed for **essential driving skill training** on PCs, as it enables repetitive **braking** and **stop-point accuracy** practice.

Supports:
- **Manual stop control** practice
- **TASC-assisted automatic stopping**
- **Emergency** scenarios including obstacles on track and rail suicides.
- **Brake navigation** to explore efficient brake profiles
- EMU Rolling Stock with Accurate Real-World and Performance Data
- Real-world Time-Space diagrams in Drive Mode
- VVVF sounds based on velocity-based pitch modulation 
- ATC, an automatic railway safety system crucial to modern EMUs

Powered by **Three.js 3D visualization**, the simulator delivers an immersive driver’s POV and realistic track environment.  
Real-time **velocity–distance** and **braking curve** rendering provides intuitive, instant feedback.

By replicating authentic **EMU physics**, it enables:
- **Repetitive stop training** using the random scenario toggle
- **Scenario-based evaluation**  
- **Quantitative performance scoring** under diverse track conditions
---

## ⚙️ Tech Stack
- **Backend**: Python (FastAPI, WebSocket)
- **Frontend**: HTML, CSS, JavaScript (Three.js for BVE-style graphics)
- **Deployment**: AWS EC2 (Amazon Linux), Nginx, Route53, 내도메인.한국

---

## 🚆 Application Scenarios

- **Operator Training:** Repetitive personal-level stopping practice using simplified simulator hardware  
- **Educational Demonstrations:** Visualization of efficient brake curves 
- **Research & Development:** Testing human–automation interaction (TASC) algorithms    

---

## 🧠 Why TASC?
- **Reduces driver workload & fatigue**: automatically executes initial, stair-step braking and relaxing → reduces cognitive load and control fatigue.
- **Higher stopping accuracy & consistent ride comfort**: useful when stopping at stations with PSDs (platform doors); always produces a **similar braking profile**.
- **Low-cost and low-risk**: cheaper than ATO systems; TASC allows manual override if equipment malfunctions.


---

## ⚙️ Config
```bash
tasc/
├── __pycache__
│   └── server.cpython-312.pyc
├── scenario.json // Scenario configuration file
├── server.py // FastAPI backend server
└── static 
    ├── E233 // E233 train model real data
    ├── audio // Audio files
    ├── emu_db // EMU database files
    ├── favicon.ico
    ├── helper // Notch calculator scripts
    ├── index.html // Main HTML/JS frontend script
    ├── logos 
    ├── photos // Panel photos
    └── textures // Texture files
```
---

## 📄 License

Copyright © 2025 Hyungsuk Choi, University of Maryland

Permission is hereby granted to use and modify this software for personal or internal purposes only.  
Redistribution, reproduction, resale, public posting, or sharing of this software or any modified versions  
is strictly prohibited without the express written permission of the author.

## E233 Data Provider Credit (Required Attribution)

- **Data Provider**: `E233-3639`  
- **Original Repository**: `https://github.com/E233-3639/BVE_E233-3000_Data.git`

The vehicle data (OriginalData) included in the `tasc/static/E233` folder is provided by the above contributor. When using or distributing this data, you **must display the provider credit** in the project UI and documentation (e.g., README). This requirement follows the licensing and copyright attribution terms of the E233 data, so please ensure compliance.

















