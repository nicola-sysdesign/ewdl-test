## Requirements:
### Operating System:
- Ubuntu 16.04
### Kernel
- Linux 4.14 + PREEMPT_RT
## Build
```
git clone --recursive https://github.com/nicola-sysdesign/ewdl-test.git
mkdir -p ewdl-test/build/
cd ewdl-test/build/
cmake -D CMAKE_BUILD_TYPE=Release ..
make
```
## Warning
The executable will configure the EWDL drive to operate in **Cyclic Synchronous Position Mode (CSP)**, then it will execute a _sin_ function between +/- 10000 encoder counts.

***Before run it be sure that the motor is free to move between the limits.***
## Run
```
sudo chrt --rr 90 ./main
```
