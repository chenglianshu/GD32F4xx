# GD32F4xx grblHAL Driver

grblHAL driver for GD32F405xx, GD32F407xx, GD32F425xx, and GD32F450xx ARM processors.

This is the world's first project porting grblHAL to GigaDevice's GD32F4xx series chips; the primary reason for undertaking this port is that I happen to have a few CNC control boards based on the GigaDevice GD32F425RET6 on hand.

This project has been tested on actual hardware; limit switches, homing, spindle movement for engraving, the laser, and SD card functionality all operate correctly.

## Timeline

### 2026-11-20

**Current**

- Removed all PlatformIO-related resources from the project architecture. The project is now built entirely with **CMake + Ninja + GCC (Arm GNU toolchain)**, with **OpenOCD** for flashing and debugging.
- Ported **FreeRTOS** into the project.

**Future**

- Adapt the project to support more GD32 chips.
