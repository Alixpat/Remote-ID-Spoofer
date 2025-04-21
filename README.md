
# ESP32 Drone Remote ID Spoofer & Simulator
 
A powerful Flask-based web application and ESP32 firmware for spoofing and simulating drone Remote ID broadcasts over Wi-Fi. Features real‑time map-based flight path design, USB serial integration, dynamic Remote ID and MAC spoofing, and full play/pause/stop control.
 
## 🚀 Features
 
- **USB Serial Port Selection**  
  Centered, monospace-styled screen for selecting a single USB device with ASCII art header.
 
- **Interactive Map Simulator**  
  - Click to drop waypoints; first waypoint highlighted lime-green, last highlighted purple.  
  - Set pilot location separately.  
  - Play, Pause, and Stop buttons (green, orange, red outlines) control simulated flight and serial broadcasts.  
  - Real-time drone icon (“🛸”) moves smoothly along a path at configurable speed (default 25 mph).  
  - Looping paths draw connecting light-blue line between final and first waypoint.  
  - Pause retains current position; Stop halts all serial transmissions immediately.
 
- **Dynamic Remote ID & MAC Spoofing**  
  - Override Remote ID fields and pilot info via on-screen form in hot-pink labels.  
  - Default MAC starts with `60:60:1f`, random suffix each boot; can also be set manually from the GUI (triggering ESP32 reboot).
 
- **Web UI UX**  
  - OLED-style black background with neon lime, purple, and hot-pink accents.  
  - Drone/pilot icons (“🛸” and “👤”) with colored outlines indicating status:  
    - **Green** circle = active (Play),  
    - **Orange** circle = paused,  
    - **Red** circle = stopped.  
  - Connected/disconnected USB status in bottom-right, updating instantly (red/green text).
 
- **JSON-over-Serial Protocol**  
  - ESP32 firmware receives full Remote ID JSON payload via USB serial from Python script.  
  - Serial messages only sent when simulation is playing; messages stop instantly on “Stop”.  
  - Example JSON structure:
    ```json
    {
      "mac": "60:60:1f:AA:BB:CC",
      "basic_id": "DRONE1234",
      "drone_lat": 35.5934,
      "drone_long": -82.5546,
      "pilot_lat": 35.5920,
      "pilot_long": -82.5530,
      "drone_altitude": 120.5,
      "timestamp": "2025-04-21T13:00:00Z"
    }
    ```
 
## 📦 Installation
 
1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/esp32-drone-spoofer.git
   cd esp32-drone-spoofer
   ```
 
2. **Setup Python environment**:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```
 
3. **Install PlatformIO and build firmware**:
   ```bash
   pip install platformio
   cd firmware
   platformio run --environment seeed_xiao_esp32s3
   platformio run --target upload
   ```
 
4. **Run the Flask App**:
   ```bash
   cd ../spoof
   python spoof.py
   ```
   Open `http://localhost:5000` in your browser.
 
## ⚙️ Configuration
 
- **Play Speed**: Adjust speed (mph) in the “Speed” field before playing.
- **MAC Override**: Enter full MAC under “Override MAC” and click **Set MAC** (hot-pink). ESP32 will reboot with new spoofed MAC.
- **Pilot Location**: Click “Set Pilot Location” (hot-pink) then click on map; displayed persistently.
- **Waypoints**: Click on map to add waypoints (lime); first and last waypoints highlighted.
 
## 🛠️ Usage
 
1. Select your USB serial device.
2. Design your flight path and set pilot location.
3. Optionally override Remote ID and MAC.
4. Click **Play** (green) to start simulating and broadcasting.
5. **Pause** (orange) to freeze position (serial continues sending last state).
6. **Stop** (red) to halt all serial traffic immediately; icon remains on map.
 
## 🔄 Looping Paths
 
- Loops automatically connect final waypoint back to the first with a solid light-blue line.
- On **Play**, path loops seamlessly.
 
## 💡 Serial Protocol
 
- Messages sent every **200 ms** during simulation.
- JSON payload fields correspond to the Remote ID spec.
- Transmission halts instantly on **Stop**.
 
## 📝 License
 
MIT © Your Name
