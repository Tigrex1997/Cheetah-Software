## Cheetah-Software
This repository contains the Robot and Simulation software project.  For a getting started guide, see the documentation folder.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified. This should just be libsoem for Cheetah 3, which Pat modified at one point.

## Build
To build all code:
```
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j4
```

If you are building code on your computer that you would like to copy over to the mini cheetah, you must replace the cmake command with
```
cmake -DMINI_CHEETAH_BUILD=TRUE
```
otherwise it will not work.  If you are building mini cheetah code one the mini cheetah computer, you do not need to do this.

This build process builds the common library, robot code, and simulator. If you just change robot code, you can simply run `make -j4` again. If you change LCM types, you'll need to run `cmake ..; make -j4`. This automatically runs `make_types.sh`.

To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.

Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```
## Run simulator
To run the simulator:
1. Open the control board
```
./sim/sim
```
2. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/JPos_Controller/jpos_ctrl 3 s
```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot

## Run Mini cheetah
1. Create build folder `mkdir mc-build`
2. Build as mini cheetah executable `cd mc-build; cmake -DMINI_CHEETAH_BUILD=TRUE ..; make -j`
3. Connect to mini cheetah over ethernet, verify you can ssh in
4. Copy program to mini cheetah with `../scripts/send_to_mini_cheetah.sh`
5. ssh into the mini cheetah `ssh user@10.0.0.34`
6. Enter the robot program folder `cd robot-software-....`
7. Run robot code `./run_mc.sh` 



## Dependencies:
- Qt 5.10 - https://www.qt.io/download-qt-installer
- LCM - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- Eigen - http://eigen.tuxfamily.org
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

To use Ipopt, use CMake Ipopt option. Ex) cmake -DIPOPT_OPTION=ON ..

## Instructions with Ubuntu2004 (Use 'hotfix-Ubuntu2004' branch codes)
### Build (Compile)
1. Ubuntu2004 (If VMWare, allocate 4 CPUs, 8G memory (when 4G memory, compilation error), hard disk space 40G);
2. Download (Link: https://download.qt.io/new_archive/qt/) and install qt5, the installation path MUST be set to `/home/{username}/Qt` because when the source code is compiled, the default path of qt is coded as this:\
(1)\
Open a terminal and give authority to the file:\
`sudo chmod -R 777 qt-opensource-linux-x64-5.xx.x.run`\
(2)\
Use terminal to start the installing process:\
`./qt-opensource-linux-x64-5.xx.x.run`
3. Run the following command:\
`sudo apt-get install build-essential autoconf automake autopoint libglib2.0-dev libtool openjdk-8-jdk python-dev`\
Just make sure set JAVA usage as openjdk-8-jdk to avoid building failures (openjdk 1.8.0_292 & javac 1.8.0_292);
4. Download (Link: https://github.com/lcm-proj/lcm/releases/tag/v1.3.1) and install LCM 1.3.1 with the following steps:\
(1)\
`unzip lcm-1.3.1.zip`\
`cd lcm-1.3.1`\
`./configure`\
`make`\
`sudo make install`\
(2)\
`export LCM_INSTALL_DIR=/usr/local/lib`\
`sudo sh -c "echo $LCM_INSTALL_DIR > /etc/ld.so.conf.d/lcm.conf"`\
`sudo ldconfig`
5. Test if LCM 1.3.1 is successfully installed using:\
`lcm-logplayer-gui`
6. Install Eigen and set it as path used in the default code (otherwise need to change code):\
`sudo apt-get install libeigen3-dev`\
`sudo cp -r /usr/include/eigen3 /usr/local/include/`
7. Install necessary dependencies:\
`sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev
`
8. Make sure set gcc&g++ usage as 'gcc-7' and 'g++-7' (gcc-9 and g++-9 may need code change);
9. Download source code and follow the instrcutions in a terminal to compile:\
`cd Cheetah-Software/`\
`mkdir build`\
`cd build`\
`cmake ..`\
`./../scripts/make_types.sh`\
`make -j4`

### Simulation (Basic)
1. Under `Cheetah-Software/build` folder:\
`./sim/sim`
2. Select 'Mini Cheetah' and 'Simulation' buttons and press the 'start' button;
3. When an editable parameter list appears, set 'cheater_mode' to 1, 'control_mode' to 1, and 'use_rc' to 0;
4. Start a controller (Here MIT):\
Under `Cheetah-Software/build` folder:\
`./user/MIT_Controller/mit_ctrl m s`
After starting the controller, the robot will stretch out 4 legs and stand up.