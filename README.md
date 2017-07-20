# Unscented Kalman Filter Project Starter Code

The original pre-fork README can be found [here](README_ori.md).

### Debug Builds

Created via:
```
mkdir debug
cd debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```
### GDB Commands

  - break kalman_filter.cpp:65 if px < -14
  - kill
  - run
  - info stack
  - info local
