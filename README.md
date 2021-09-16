# IROS2021_ICE

## Usage (py_gym)
---


### 1. Requirements

1. Framework

    ```
    python3 3.8 or later
    gym
    pyyaml
    numba
    numpy
    ```

2. F1TENTH Gym Environment
    
   to install f1tenth_gym, follow the [Quickstart](https://github.com/f1tenth/f1tenth_gym) at f1tenth/f1tenth_gym or try:
   ```commandline
   pip3 install --user -e gym/
   ```

### 2. Run

1. Quickstart
   ```commandline
   # run qualification
   python3 sim_solo.py
   
   # run head-to-head
   python3 sim_duo.py
   ```
   
2. Choose Planner

   1. `sim_solo.py`
   
      ```buildoutcfg
      # Import class from planner/ folder.
      from planner.fgm_stech import FGM
      from planner.fgm_gnu import FGM_GNU
      from planner.odg_pf import ODGPF
      from planner.odg_gnu import ODGGNU
      from planner.fgm_conv import FGM_CONV
      from planner.fgm_stech_conv import FGM as FGM_STECH_CONV
    
      ...
    
      # choose you want 
      # planner = FGM_GNU(conf)
      # planner = FGM_STECH(conf)
      planner = ODGPF(conf)
      ```
      
   2. `sim_duo.py`
      ```buildoutcfg
      # Import class from planner/ folder.
      from planner.fgm_stech import FGM
      from planner.fgm_gnu import FGM_GNU
      from planner.odg_pf import ODGPF
      from planner.odg_gnu import ODGGNU
      from planner.fgm_conv import FGM_CONV
      from planner.fgm_stech_conv import FGM as FGM_STECH_CONV
      
      ...
      
      # choose you want
      # planner = [ODGPF(conf), FGM_GNU(conf)]
      planner = [FGM_CONV(conf), FGM_STECH_CONV(conf)]
      ```


### 3. Parameters
 
   If you want to change the parameters, you need modify `sim_params.yaml`
   
   ```buildoutcfg
   # Environment Constants
   map_path: "map/SOCHI_OBS"
   map_ext: '.png'
   
   p1:               
     sx: 0.8007017
     sy: -0.2753365
     stheta: 4.1421595
   p2:               
     sx: 0.8162458
     sy: 1.1614572
     stheta: 4.1446321
   
   
   # Speed Controller
   # 0: Constant
   # 1: Braking Distance Based
   # 2: Angle Based
   # 3: Braking dist + Angle Based
   speed_controller: 3
   
   
   # Debugging Settings
   debug:
     gui_render: true           # rendering screen (true|false)
     plotting: false            # print speed graph (true|false)
     logging: false             # save log (true|false)
     collision_reset: true      # restart at collision (true|false)
     collision_interval: 2      # repeat time at collision (int)
   
   
   
   # Planner Constants
   wpt_path: 'map/wp_SOCHI.csv' # path of waypoint file (you **MUST** change!)
   wpt_delimeter: ','           # delimiter of csv file
   
   max_speed: 14.0              # Max speed 
   min_speed: 5.0               # Min speed 
   
   robot_length: 0.3302
   robot_scale: 0.2032
   mass: 3.47
   
   mu: 0.523
   pi: 3.141592
   g: 9.81
   
   fgm:
     look: 2.0
     threshold: 3.0
     gap_size: 1
     filter_scale: 1.1
     gap_theta_gain: 20.0
     ref_theta_gain: 1.5
   
   odg:
     threshold: 2.0
     filter_scale: 1.1

   ```

   - `wpt_path` - **real** path of waypoint file (you **MUST** change!)
   - `wpt_delimeter` - delimiter of csv file
   - `max_speed` - Max speed 
   - `min_speed` - Min speed
   - `rate` - provide particular rate for a loop (Hz)
   - `robot_length` - length of wheelbase
    
   - `robot_scale` - width of car
   - `mass` - mass of car
   - `mu` - surface friction coefficient
   - `pi` - π
   - `g`- Gravitational acceleration 
    
