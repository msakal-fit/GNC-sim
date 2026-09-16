#

Changelog
- adding looking ahead

## P05
```
Starting the P05 - LMPC simulation...
t=0.0s | pos=[0.01,0.01,-5.00] err=[0.01,0.01,0.00](0.02m) | vel=[0.00,-0.01,-0.00] | | 
t=0.0s | pos=[0.01,0.01,-5.00] err=[0.01,0.01,0.00](0.02m) | vel=[0.00,-0.01,-0.00] | | T=14.8N τ=[-0.001,-0.043,0.026]Nm
| T_virt=14.8N τ_virt=[-0.001,-0.043,0.026]Nm | N=15 k_look=6 t_MPC=0.014s
t=8.3s | pos=[0.91,0.00,-4.99] err=[0.07,0.00,0.01](0.07m) | vel=[0.05,-0.00,-0.00] | | 
t=8.3s | pos=[0.91,0.00,-4.99] err=[0.07,0.00,0.01](0.07m) | vel=[0.05,-0.00,-0.00] | | T=14.7N τ=[-0.000,-0.017,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.017,-0.000]Nm | N=15 k_look=6 t_MPC=0.011s
t=16.7s | pos=[1.71,-0.01,-5.00] err=[0.05,-0.01,0.00](0.05m) | vel=[0.10,0.02,0.00] | |
t=16.7s | pos=[1.71,-0.01,-5.00] err=[0.05,-0.01,0.00](0.05m) | vel=[0.10,0.02,0.00] | | T=14.7N τ=[-0.002,-0.013,0.000]Nm
| T_virt=14.7N τ_virt=[-0.002,-0.013,0.000]Nm | N=15 k_look=6 t_MPC=0.011s
t=25.0s | pos=[2.52,-0.00,-5.00] err=[0.02,-0.00,-0.00](0.02m) | vel=[0.12,0.00,0.00] | 
t=25.0s | pos=[2.52,-0.00,-5.00] err=[0.02,-0.00,-0.00](0.02m) | vel=[0.12,0.00,0.00] | | T=14.7N τ=[-0.001,-0.013,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,-0.013,-0.000]Nm | N=15 k_look=6 t_MPC=0.014s
t=33.3s | pos=[3.36,0.02,-5.00] err=[0.03,0.02,0.00](0.04m) | vel=[0.11,-0.01,-0.01] | |
t=33.3s | pos=[3.36,0.02,-5.00] err=[0.03,0.02,0.00](0.04m) | vel=[0.11,-0.01,-0.01] | | T=14.7N τ=[-0.000,-0.013,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.013,0.000]Nm | N=15 k_look=6 t_MPC=0.017s
t=41.7s | pos=[4.19,0.01,-4.99] err=[0.02,0.01,0.01](0.03m) | vel=[0.09,0.00,-0.01] | | 
t=41.7s | pos=[4.19,0.01,-4.99] err=[0.02,0.01,0.01](0.03m) | vel=[0.09,0.00,-0.01] | | T=14.7N τ=[-0.001,-0.013,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,-0.013,-0.000]Nm | N=15 k_look=6 t_MPC=0.021s
t=50.0s | pos=[5.05,-0.00,-4.99] err=[0.05,-0.00,0.01](0.05m) | vel=[0.11,-0.01,-0.01]
t=50.0s | pos=[5.05,-0.00,-4.99] err=[0.05,-0.00,0.01](0.05m) | vel=[0.11,-0.01,-0.01]
t=50.0s | pos=[5.05,-0.00,-4.99] err=[0.05,-0.00,0.01](0.05m) | vel=[0.11,-0.01,-0.01] | | T=14.7N τ=[-0.002,-0.013,0.001]Nm
| T_virt=14.7N τ_virt=[-0.002,-0.013,0.001]Nm | N=15 k_look=6 t_MPC=0.016s
t=58.3s | pos=[5.86,0.01,-5.01] err=[0.03,0.01,-0.01](0.04m) | vel=[0.09,-0.01,0.01] |
t=58.3s | pos=[5.86,0.01,-5.01] err=[0.03,0.01,-0.01](0.04m) | vel=[0.09,-0.01,0.01] |
t=58.3s | pos=[5.86,0.01,-5.01] err=[0.03,0.01,-0.01](0.04m) | vel=[0.09,-0.01,0.01] | | T=14.7N τ=[-0.002,-0.013,0.000]Nm
| T_virt=14.7N τ_virt=[-0.002,-0.013,0.000]Nm | N=15 k_look=6 t_MPC=0.015s
Control log saved to: log_p05_lmpc.mat

=== Metrics (P05 - LMPC Tracking (Multi Shooting)) ===
RMSE [ex, ey, ez] = [0.040, 0.012, 0.006] m
RMSE of norm(e)       = 0.042 m
Max err [ex, ey, ez] = [0.095, 0.027, 0.016] m
Max of norm(e)        = 0.095 m

Average control effort  mean of (u)   = 14.716
Max control effort      max(u)    = 14.774
Average thrust          mean(T)       = 14.716 N
Max thrust              max(T)        = 14.774 N
Average torque norm     mean(tau) = 0.014 Nm
Max torque norm         max(tau)  = 0.051 Nm
Average CPU time per step  = 0.1041 s
Max CPU time per step      = 0.1138 s
======================================

Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode

```

## P04
```
run_P04_LMPC_point_multi
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the LMPC simulation...
t=0.0s | pos=[-0.00,0.00,-5.00] err=[-0.00,0.00,-4.00](4.00m) | vel=[0.01,0.02,-0.00] | | T=7.4N τ=[-0.001,0.000,-0.000]Nm
| T_virt=7.4N τ_virt=[-0.001,0.000,-0.000]Nm | N=15 k_look=6 t_MPC=0.028s
t=8.3s | pos=[-0.02,0.01,-1.01] err=[-0.02,0.01,-0.01](0.03m) | vel=[0.00,-0.01,0.00] | | T=14.7N τ=[0.000,0.001,-0.000]Nm
| T_virt=14.7N τ_virt=[0.000,0.001,-0.000]Nm | N=15 k_look=6 t_MPC=0.016s
t=16.7s | pos=[0.01,-0.01,-1.00] err=[0.01,-0.01,-0.00](0.01m) | vel=[-0.00,0.02,-0.00] | | T=14.7N τ=[-0.002,-0.000,0.000]Nm
| T_virt=14.7N τ_virt=[-0.002,-0.000,0.000]Nm | N=15 k_look=6 t_MPC=0.014s
t=25.0s | pos=[0.00,0.02,-0.99] err=[0.00,0.02,0.01](0.03m) | vel=[0.00,-0.01,-0.01] | | T=14.7N τ=[0.001,0.000,0.000]NmRefactor hover thrust calculation and update dynamics constraint for LMPC
| T_virt=14.7N τ_virt=[0.001,0.000,0.000]Nm | N=15 k_look=6 t_MPC=0.012s
t=33.3s | pos=[-0.02,0.00,-1.01] err=[-0.02,0.00,-0.01](0.02m) | vel=[-0.00,0.00,0.00] | | T=14.7N τ=[-0.001,0.000,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.000,0.000]Nm | N=15 k_look=6 t_MPC=0.015s
t=41.7s | pos=[0.02,0.01,-1.01] err=[0.02,0.01,-0.01](0.02m) | vel=[0.00,-0.01,0.00] | | T=14.7N τ=[-0.001,-0.001,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,-0.001,-0.000]Nm | N=15 k_look=6 t_MPC=0.016s
t=50.0s | pos=[-0.02,0.00,-1.00] err=[-0.02,0.00,0.00](0.02m) | vel=[0.02,-0.00,-0.00] | | T=14.7N τ=[-0.001,0.002,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.002,0.000]Nm | N=15 k_look=6 t_MPC=0.012s
t=58.3s | pos=[-0.03,-0.01,-1.01] err=[-0.03,-0.01,-0.01](0.03m) | vel=[0.02,0.01,0.01] | | T=14.7N τ=[-0.002,0.003,0.000]Nm
| T_virt=14.7N τ_virt=[-0.002,0.003,0.000]Nm | N=15 k_look=6 t_MPC=0.014s
Control log saved to: log_p04a_lmpc.mat

=== Metrics ( P04a - LMPC Multi-Shooting) ===
RMSE [ex, ey, ez] = [0.016, 0.013, 0.575] m
RMSE of norm(e)       = 0.575 m
Max err [ex, ey, ez] = [0.037, 0.030, 3.998] m
Max of norm(e)        = 3.998 m

Average control effort  mean of (u)   = 14.523
Max control effort      max(u)    = 15.647
Average thrust          mean(T)       = 14.523 N
Max thrust              max(T)        = 15.647 N
Average torque norm     mean(tau) = 0.002 Nm
Max torque norm         max(tau)  = 0.004 Nm
Average CPU time per step  = 0.1032 s
Max CPU time per step      = 0.1228 s
======================================

Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode

```





## P07
```
>> run_P07_NMPC
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting NMPC tracking control...
Starting the NMPC Single Shooting simulation...
t=0.0s | pos=[0.00,-0.01,-4.99] err=[0.00,-0.01,0.01](0.01m) | vel=[0.02,0.02,-0.01] | | T=14.7N τ=[-0.001,-0.014,-0.011]Nm
| T_virt=14.7N τ_virt=[-0.001,-0.014,-0.011]Nm | N=15 k_look=6 t_MPC=0.059s
t=8.3s | pos=[0.87,-0.00,-4.99] err=[0.04,-0.00,0.01](0.04m) | vel=[0.11,-0.00,-0.01] | | T=14.7N τ=[-0.000,0.008,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.008,0.000]Nm | N=15 k_look=6 t_MPC=0.063s
t=16.7s | pos=[1.70,0.01,-4.99] err=[0.03,0.01,0.01](0.03m) | vel=[0.11,-0.01,-0.00] | | T=14.7N τ=[-0.000,0.007,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.007,-0.000]Nm | N=15 k_look=6 t_MPC=0.048s
t=25.0s | pos=[2.52,0.00,-5.00] err=[0.02,0.00,-0.00](0.02m) | vel=[0.09,0.01,0.00] | | T=14.7N τ=[-0.000,0.005,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.005,0.000]Nm | N=15 k_look=6 t_MPC=0.049s
t=33.3s | pos=[3.37,-0.01,-4.99] err=[0.03,-0.01,0.01](0.04m) | vel=[0.11,-0.00,-0.01] | | T=14.7N τ=[0.001,0.008,0.000]Nm
| T_virt=14.7N τ_virt=[0.001,0.008,0.000]Nm | N=15 k_look=6 t_MPC=0.045s
t=41.7s | pos=[4.19,0.00,-5.00] err=[0.02,0.00,-0.00](0.03m) | vel=[0.10,-0.01,0.01] | | T=14.7N τ=[-0.000,0.006,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.006,0.000]Nm | N=15 k_look=6 t_MPC=0.038s
t=50.0s | pos=[5.04,0.03,-5.00] err=[0.04,0.03,0.00](0.05m) | vel=[0.08,-0.02,-0.00] | | T=14.7N τ=[-0.001,0.006,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.006,-0.000]Nm | N=15 k_look=6 t_MPC=0.065s
t=58.3s | pos=[5.87,-0.02,-4.99] err=[0.04,-0.02,0.01](0.04m) | vel=[0.09,-0.02,-0.01] | | T=14.7N τ=[0.002,0.007,0.000]Nm
| T_virt=14.7N τ_virt=[0.002,0.007,0.000]Nm | N=15 k_look=6 t_MPC=0.053s
Control log saved to: log_p07_nmpc.mat

=== Metrics (P07 - NMPC Single Shooting) ===
RMSE [ex, ey, ez] = [0.034, 0.015, 0.008] m
RMSE of norm(e)       = 0.037 m
Max err [ex, ey, ez] = [0.058, 0.048, 0.023] m
Max of norm(e)        = 0.059 m

Average control effort  mean of (u)   = 14.716
Max control effort      max(u)    = 14.732
Average thrust          mean(T)       = 14.716 N
Max thrust              max(T)        = 14.732 N
Average torque norm     mean(tau) = 0.007 Nm
Max torque norm         max(tau)  = 0.018 Nm
Average CPU time per step  = 0.1466 s
Max CPU time per step      = 0.1944 s
======================================

```



## P08b
```
>> run_P08b_traj_multi
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting NMPC tracking control...
Starting the P08b - NMPC Tracking (Multi Shooting) simulation...
t=0.0s | pos=[0.00,-0.00,-5.00] err=[0.00,-0.00,0.00](0.01m) | vel=[-0.02,-0.01,-0.00] | | T=14.7N τ=[0.002,-0.017,0.030]Nm
| T_virt=14.7N τ_virt=[0.002,-0.017,0.030]Nm | N=15 k_look=6 t_MPC=0.036s
t=8.3s | pos=[0.87,0.00,-4.99] err=[0.03,0.00,0.01](0.03m) | vel=[0.12,0.00,-0.01] | | T=14.7N τ=[-0.000,0.008,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.008,0.000]Nm | N=15 k_look=6 t_MPC=0.018s
t=16.7s | pos=[1.70,0.01,-5.01] err=[0.03,0.01,-0.01](0.03m) | vel=[0.11,0.00,0.01] | | T=14.7N τ=[-0.000,0.007,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.007,-0.000]Nm | N=15 k_look=6 t_MPC=0.019s
t=25.0s | pos=[2.52,0.00,-4.99] err=[0.02,0.00,0.01](0.02m) | vel=[0.11,0.01,-0.01] | | T=14.7N τ=[-0.001,0.007,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.007,-0.000]Nm | N=15 k_look=6 t_MPC=0.016s
t=33.3s | pos=[3.36,0.02,-5.00] err=[0.03,0.02,-0.00](0.04m) | vel=[0.10,-0.01,0.00] | | T=14.7N τ=[-0.000,0.007,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.007,-0.000]Nm | N=15 k_look=6 t_MPC=0.015s
t=41.7s | pos=[4.20,0.01,-5.01] err=[0.04,0.01,-0.01](0.04m) | vel=[0.10,-0.01,0.01] | | T=14.7N τ=[0.000,0.008,0.000]Nm
| T_virt=14.7N τ_virt=[0.000,0.008,0.000]Nm | N=15 k_look=6 t_MPC=0.016s
t=50.0s | pos=[5.03,0.02,-5.00] err=[0.03,0.02,0.00](0.03m) | vel=[0.11,-0.00,-0.00] | | T=14.7N τ=[-0.001,0.007,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.007,0.000]Nm | N=15 k_look=6 t_MPC=0.020s
t=58.3s | pos=[5.84,-0.04,-5.02] err=[0.01,-0.04,-0.02](0.05m) | vel=[0.12,0.03,0.02] | | T=14.7N τ=[0.002,0.006,0.000]Nm
| T_virt=14.7N τ_virt=[0.002,0.006,0.000]Nm | N=15 k_look=6 t_MPC=0.016s
Control log saved to: log_p08b_nmpc.mat

=== Metrics (P08b - NMPC Tracking (Multi Shooting)) ===
RMSE [ex, ey, ez] = [0.038, 0.015, 0.007] m
RMSE of norm(e)       = 0.041 m
Max err [ex, ey, ez] = [0.068, 0.045, 0.021] m
Max of norm(e)        = 0.070 m

Average control effort  mean of (u)   = 14.717
Max control effort      max(u)    = 14.741
Average thrust          mean(T)       = 14.717 N
Max thrust              max(T)        = 14.741 N
Average torque norm     mean(tau) = 0.008 Nm
Max torque norm         max(tau)  = 0.046 Nm
Average CPU time per step  = 0.1066 s
Max CPU time per step      = 0.1367 s
======================================

Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode

```

