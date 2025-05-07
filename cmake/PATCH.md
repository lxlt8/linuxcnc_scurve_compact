# PATCH MANUAL

- This manual is only needed if the normal 
```bash
    /cmake/./patch 
    /cmake/./installer 
```
  failed to compile. 

- To install the scurve planner on different lcnc versions, <br>
  you need to patch the original lcnc clone or verion. <br>

- If you use a tool like :https://www.diffchecker.com/text-compare/ <br>
  you can compare the files to path, with your current file, <br>
  and see the difference between them. <br>
  This manual shows how to patch the current lcnc git clone, as it is today. <br>

- The path includes replacing 4 files, <br>
  and needs at least 3 files to be replaced or modified : <br>
 
---

# Files to patch
```bash
    ~/linuxcnc/src/emc/motion/motion.c
    ~/linuxcnc/src/emc/tp/tp.c
    ~/linuxcnc/src/emc/tp/tp.h
```
 ---

## 1. motion.c
~Line 242 :
```bash
    // exsisting: 
    tpMotData(emcmotStatus
             ,emcmotConfig
             );
    // new:         
    tpMotExtraData(emcmotCommand,
                   emcmot_hal_data
                   );
```

---

## 2. tp.c
~Line 64
```bash
    // exsisting: 
    remove : static emcmot_status_t *emcmotStatus;
    remove : static emcmot_config_t *emcmotConfig;
    
    // new:
    emcmot_status_t *emcmotStatus;
    emcmot_config_t *emcmotConfig;
    emcmot_command_t *emcmotCommand;
    emcmot_hal_data_t *emcmot_hal_data;
```
Note : in latest lcnc version some dev now used "static emcmot_status_t *emcmotStatus;".
We don't use that.

---

## 3. tp.h
~Line 20
```bash
    // exsisting:
    #include "tcq.h"
    // new:
    #include "mot_priv.h"
```
~Line 85
```bash
    // exsisting:
    ,emcmot_config_t *
    );
    // new:
    // Forward decleration.
    void tpMotExtraData(emcmot_command_t *
                   ,emcmot_hal_data_t *
                   );
```

---

## Test patch
Then i tested a linuxcnc git clone, cq version and a patch by : <br>

```bash
    git clone https://github.com/LinuxCNC/linuxcnc.git lcnc_path_test
```

***Note: Use recursive, this downloads the scurve & clothoid lib from external sources.***
```bash
    git clone --recurse-submodules https://codeberg.org/skynet/linuxcnc_scurve_compact lcnc_codeberg
```

copy the cmake folder & content from lcnc_codeberg/cmake to lcnc_path_test/cmake <br>

```bash
    cd lcnc_path_test/cmake
    ./patch
    ./build_lcnc
    cd ..
    cd scripts
    . ./rip-environment
    linuxcnc
```

---

## Scurve
When above steps are working, and linuxcnc axis gui run's ok, <br>
the last thing to do is install the scurve code on top of the linuxcnc installation. <br>

## Dependencies:
```bash
    sudo apt-get install libceres-dev
```

---

## Install scurve:
Then to build & install the scurve, you can navigate to the ~/cmake directoy: <br>
This then builds & installs the scurve component, clothoid lib, scurve lib etc. <br>
```bash
./build_cmake 
```

---

## Author
michelwijnja@gmail.com












