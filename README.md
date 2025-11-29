# JR Train Simulator: Lightweight Precision Stopping Trainer (PTS)

 
[Try PTS!⛅️ → Play full screen on PC](https://jrtasc.kro.kr/) 
---
<img width="1456" height="763" alt="a" src="https://github.com/user-attachments/assets/5e59b9bd-435f-40ae-966d-7077c6e64e5a" />
<img width="1472" height="769" alt="b" src="https://github.com/user-attachments/assets/2afecde8-b4c3-490b-8e6d-5736c79b5865" />
<img width="1466" height="756" alt="e1" src="https://github.com/user-attachments/assets/ba73debc-bf24-4ae4-ab7d-b23be708bb84" />
<img width="1505" height="762" alt="e2" src="https://github.com/user-attachments/assets/7945a09f-268a-4e84-ba7d-4f621925b3e5" />
<img width="980" height="265" alt="feedback" src="https://github.com/user-attachments/assets/b523694b-a208-4161-b031-6abb18f66f38" />

A lightweight **web-based railway simulation framework** replicating key control functions of **JR East/West and Korean EMUs**.  
Unlike heavy simulators, PTS is designed for **essential driving skill training** on PCs, as it enables repetitive **braking** and **stop-point accuracy** practice.

Supports:
- **Manual stop control** practice
- **TASC-assisted automatic stopping**
- **Emergency** scenarios including obstacles on track and rail suicides.
- **Brake navigation** to explore efficient brake profiles (Toggle with B)
- **Mini HUD** to assist trainees
- Realistic VVVF sounds

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

## 🔭 Future Development

Planned upgrades include integration with advanced railway control frameworks and signals such as:

- **ATC (Automatic Train Control)**  
- **ATP (Automatic Train Protection)**  
- **ATS (Automatic Train Stop)**  


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














