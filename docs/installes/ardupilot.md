# Ardupilot copter

- Download binary from https://firmware.ardupilot.org
- Download last stable SITL for x64 [...](https://firmware.ardupilot.org/Copter/stable-4.1.3/SITL_x86_64_linux_gnu/)

## Build SILT from code
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

## Run SITL
```
./arducopter413 -S --speedup 1 -I0 --model gazebo-iris \
--defaults /home/user/projects/wasp/apm/copter.parm,/home/user/projects/wasp/apm/gazebo-iris.parm
```

---

# Gazebo plugins
- [khancyr ardupilt_gazebo](https://github.com/khancyr/ardupilot_gazebo)

```


```