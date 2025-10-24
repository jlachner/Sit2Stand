# Sit2Stand

**Stephan Stansfield, Johannes Lachner**  

MIT, Department of Mechanical Engineering, USA  


### Build · Debug · Run (tested for Linux + VS Code)

Using **CMake Tools**:

1. **Dependencies**
   - System: `cmake`, `build-essential`, `gdb` (for debugging).
   - VS Code extensions: **C/C++**, **CMake**, **CMake Tools**.

2. **Configure**
   - Open the folder with your `CMakeLists.txt` in VS Code.  
   - Run:
     ```bash
     cmake -S ~/Documents/GitHub/Sit2Stand/robot_posCtrl/src       -B ~/Documents/GitHub/Sit2Stand/robot_posCtrl/build       -DCMAKE_BUILD_TYPE=Debug
     ```

3. **Build**
   - Run:
     ```bash
     cmake --build ~/Documents/GitHub/Sit2Stand/robot_posCtrl/build --config Debug
     ```

4. **Run**
   - If your generator places config subfolders:
     ```bash
     ../build/Debug/Sit2Stand
     ```

### Run Java Application (SmartPAD)
