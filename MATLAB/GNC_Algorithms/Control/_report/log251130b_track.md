## P02

```
>> run_P02_LQT
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
State weight matrix Q as diag:
    50     0     0
     0     5     0
     0     0    25

    12     0     0
     0     2     0
     0     0     2

    20     0     0     0
     0    20     0     0
     0     0    20     0
     0     0     0    20

     2     0     0
     0     2     0
     0     0     2

Input weight matrix R as diag:
     2     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1

Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the LQT simulation...
t=0.0s | pos=[-0.03,0.01,-5.00] err=[-0.03,0.01,0.00](0.03m) | vel=[0.02,0.01,-0.00] | | T=14.8N τ=[-0.019,-0.583,0.356]Nm
t=8.3s | pos=[0.72,-0.01,-4.95] err=[-0.11,-0.01,0.05](0.12m) | vel=[0.10,0.01,0.01] | | T=14.9N τ=[0.013,-0.652,0.002]Nm
t=16.7s | pos=[1.56,0.00,-4.92] err=[-0.11,0.00,0.08](0.14m) | vel=[0.10,0.01,0.00] | | T=15.0N τ=[-0.009,-0.651,0.002]Nm
t=25.0s | pos=[2.38,-0.02,-4.90] err=[-0.12,-0.02,0.10](0.16m) | vel=[0.12,-0.00,0.01] | | T=15.1N τ=[0.022,-0.633,0.000]Nm
t=33.3s | pos=[3.24,-0.01,-4.90] err=[-0.09,-0.01,0.10](0.13m) | vel=[0.12,0.01,0.00] | | T=15.1N τ=[0.009,-0.515,0.000]Nm
t=41.7s | pos=[4.08,-0.03,-4.91] err=[-0.08,-0.03,0.09](0.13m) | vel=[0.10,0.01,-0.00] | | T=15.0N τ=[0.036,-0.532,-0.000]Nm
t=50.0s | pos=[4.93,-0.02,-4.90] err=[-0.07,-0.02,0.10](0.13m) | vel=[0.09,0.01,-0.00] | | T=15.1N τ=[0.024,-0.503,0.001]Nm
t=58.3s | pos=[5.74,0.01,-4.91] err=[-0.09,0.01,0.09](0.13m) | vel=[0.12,-0.00,0.00] | | T=15.0N τ=[-0.019,-0.518,-0.000]Nm
Control log saved to: log_p02_lqt.mat

=== Metrics (P02 - LQT) ===
RMSE [ex, ey, ez] = [0.090, 0.018, 0.085] m
RMSE of norm(e)       = 0.125 m
Max err [ex, ey, ez] = [0.129, 0.039, 0.128] m
Max of norm(e)        = 0.157 m

Average control effort  mean of (u)   = 15.018
Max control effort      max(u)    = 15.146
Average thrust          mean(T)       = 15.008 N
Max thrust              max(T)        = 15.137 N
Average torque norm     mean(tau) = 0.541 Nm
Max torque norm         max(tau)  = 0.684 Nm
Average CPU time per step  = 0.0904 s
Max CPU time per step      = 0.0998 s
======================================

```



## P05

```
run_P05_LMPC_traj_multi
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
State weight matrix Q as diag:
    50     0     0
     0     5     0
     0     0    25

    12     0     0
     0     2     0
     0     0     2

    20     0     0     0
     0    20     0     0
     0     0    20     0
     0     0     0    20

     2     0     0
     0     2     0
     0     0     2

Input weight matrix R as diag:
     2     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1

Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting control...
Starting the P05 - LMPC simulation...
t=0.0s | pos=[0.01,-0.02,-5.01] err=[0.01,-0.02,-0.01](0.03m) | vel=[-0.03,0.00,0.01] | | T=14.8N τ=[-0.003,-0.027,0.028]Nm
| T_virt=14.8N τ_virt=[-0.003,-0.027,0.028]Nm | N=15 k_look=6 t_MPC=0.021s
t=8.3s | pos=[0.87,0.00,-5.00] err=[0.04,0.00,0.00](0.04m) | vel=[0.09,0.01,-0.00] | | T=14.7N τ=[-0.002,-0.010,0.000]Nm
| T_virt=14.7N τ_virt=[-0.002,-0.010,0.000]Nm | N=15 k_look=6 t_MPC=0.016s
t=16.7s | pos=[1.66,0.01,-5.00] err=[-0.00,0.01,0.00](0.01m) | vel=[0.12,-0.00,0.00] | | T=14.7N τ=[-0.000,-0.007,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.007,0.000]Nm | N=15 k_look=6 t_MPC=0.015s
t=25.0s | pos=[2.50,-0.00,-5.01] err=[0.00,-0.00,-0.01](0.01m) | vel=[0.10,-0.00,0.00] | | T=14.7N τ=[-0.000,-0.008,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.008,-0.000]Nm | N=15 k_look=6 t_MPC=0.015s
t=33.3s | pos=[3.37,-0.01,-5.00] err=[0.03,-0.01,0.00](0.04m) | vel=[0.10,-0.01,-0.00] | | T=14.7N τ=[-0.002,-0.008,0.000]Nm
| T_virt=14.7N τ_virt=[-0.002,-0.008,0.000]Nm | N=15 k_look=6 t_MPC=0.015s
t=41.7s | pos=[4.18,0.01,-5.00] err=[0.01,0.01,0.00](0.02m) | vel=[0.11,0.02,0.00] | | T=14.7N τ=[-0.002,-0.009,0.000]Nm
| T_virt=14.7N τ_virt=[-0.002,-0.009,0.000]Nm | N=15 k_look=6 t_MPC=0.017s
t=50.0s | pos=[4.99,0.01,-5.00] err=[-0.01,0.01,0.00](0.01m) | vel=[0.12,0.00,0.00] | | T=14.7N τ=[-0.001,-0.006,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,-0.006,0.000]Nm | N=15 k_look=6 t_MPC=0.020s
t=58.3s | pos=[5.85,0.00,-5.02] err=[0.01,0.00,-0.02](0.02m) | vel=[0.12,-0.03,0.01] | | T=14.7N τ=[-0.001,-0.009,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,-0.009,-0.000]Nm | N=15 k_look=6 t_MPC=0.011s
Control log saved to: log_p05_lmpc.mat

=== Metrics (P05 - LMPC Tracking (Multi Shooting)) ===
RMSE [ex, ey, ez] = [0.023, 0.020, 0.008] m
RMSE of norm(e)       = 0.031 m
Max err [ex, ey, ez] = [0.083, 0.060, 0.020] m
Max of norm(e)        = 0.084 m

Average control effort  mean of (u)   = 14.716
Max control effort      max(u)    = 14.787
Average thrust          mean(T)       = 14.716 N
Max thrust              max(T)        = 14.787 N
Average torque norm     mean(tau) = 0.009 Nm
Max torque norm         max(tau)  = 0.053 Nm
Average CPU time per step  = 0.1081 s
Max CPU time per step      = 0.1333 s
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
Starting control...
Starting the P07 - NMPC Single Shooting simulation...
t=0.0s | pos=[-0.00,0.03,-5.00] err=[-0.00,0.03,-0.00](0.03m) | vel=[0.00,-0.01,0.01] | | T=14.7N τ=[0.000,-0.008,0.025]Nm
| T_virt=14.7N τ_virt=[0.000,-0.008,0.025]Nm | N=15 k_look=6 t_MPC=0.078s
t=8.3s | pos=[0.89,0.03,-5.01] err=[0.05,0.03,-0.01](0.06m) | vel=[0.10,-0.03,0.01] | | T=14.7N τ=[-0.000,0.009,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.009,-0.000]Nm | N=15 k_look=6 t_MPC=0.069s
t=16.7s | pos=[1.70,0.01,-5.01] err=[0.03,0.01,-0.01](0.03m) | vel=[0.11,0.02,0.01] | | T=14.7N τ=[-0.000,0.009,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.009,-0.000]Nm | N=15 k_look=6 t_MPC=0.044s
t=25.0s | pos=[2.54,0.00,-4.99] err=[0.04,0.00,0.01](0.04m) | vel=[0.09,-0.01,-0.00] | | T=14.7N τ=[-0.001,0.009,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.009,0.000]Nm | N=15 k_look=6 t_MPC=0.042s
t=33.3s | pos=[3.35,0.01,-5.00] err=[0.02,0.01,-0.00](0.02m) | vel=[0.11,-0.01,0.00] | | T=14.7N τ=[0.000,0.009,-0.000]Nm
| T_virt=14.7N τ_virt=[0.000,0.009,-0.000]Nm | N=15 k_look=6 t_MPC=0.035s
t=41.7s | pos=[4.21,0.00,-5.01] err=[0.04,0.00,-0.01](0.04m) | vel=[0.09,-0.01,0.01] | | T=14.7N τ=[-0.001,0.009,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.009,0.000]Nm | N=15 k_look=6 t_MPC=0.039s
t=50.0s | pos=[5.01,-0.00,-4.99] err=[0.01,-0.00,0.01](0.02m) | vel=[0.12,0.01,-0.01] | | T=14.7N τ=[-0.001,0.008,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.008,0.000]Nm | N=15 k_look=6 t_MPC=0.038s
t=58.3s | pos=[5.87,-0.03,-5.00] err=[0.04,-0.03,-0.00](0.05m) | vel=[0.09,0.01,0.00] | | T=14.7N τ=[-0.001,0.008,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.008,0.000]Nm | N=15 k_look=6 t_MPC=0.048s
Control log saved to: log_p07_nmpc.mat

=== Metrics (P07 - NMPC Single Shooting) ===
RMSE [ex, ey, ez] = [0.029, 0.016, 0.008] m
RMSE of norm(e)       = 0.034 m
Max err [ex, ey, ez] = [0.056, 0.038, 0.024] m
Max of norm(e)        = 0.063 m

Average control effort  mean of (u)   = 14.715
Max control effort      max(u)    = 14.728
Average thrust          mean(T)       = 14.715 N
Max thrust              max(T)        = 14.728 N
Average torque norm     mean(tau) = 0.009 Nm
Max torque norm         max(tau)  = 0.038 Nm
Average CPU time per step  = 0.1405 s
Max CPU time per step      = 0.1977 s
======================================

Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode

```


# P08b
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
t=0.0s | pos=[-0.04,-0.01,-4.99] err=[-0.04,-0.01,0.01](0.04m) | vel=[0.00,-0.01,-0.01] | | T=14.7N τ=[0.000,-0.008,0.027]Nm
| T_virt=14.7N τ_virt=[0.000,-0.008,0.027]Nm | N=15 k_look=6 t_MPC=0.021s
t=8.3s | pos=[0.87,0.01,-5.00] err=[0.03,0.01,0.00](0.04m) | vel=[0.11,0.01,-0.01] | | T=14.7N τ=[-0.000,0.010,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.010,-0.000]Nm | N=15 k_look=6 t_MPC=0.019s
t=16.7s | pos=[1.70,-0.01,-5.00] err=[0.03,-0.01,-0.00](0.03m) | vel=[0.09,0.00,-0.00] | | T=14.7N τ=[-0.001,0.009,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.009,-0.000]Nm | N=15 k_look=6 t_MPC=0.021s
t=25.0s | pos=[2.49,0.00,-5.01] err=[-0.01,0.00,-0.01](0.01m) | vel=[0.12,0.01,0.01] | | T=14.7N τ=[-0.000,0.009,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.009,0.000]Nm | N=15 k_look=6 t_MPC=0.018s
t=33.3s | pos=[3.38,0.04,-5.01] err=[0.05,0.04,-0.01](0.07m) | vel=[0.09,-0.02,0.01] | | T=14.7N τ=[-0.001,0.010,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.010,0.000]Nm | N=15 k_look=6 t_MPC=0.017s
t=41.7s | pos=[4.23,-0.03,-5.00] err=[0.06,-0.03,-0.00](0.07m) | vel=[0.07,-0.00,0.01] | | T=14.7N τ=[-0.001,0.009,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.009,0.000]Nm | N=15 k_look=6 t_MPC=0.022s
t=50.0s | pos=[5.00,0.00,-4.99] err=[-0.00,0.00,0.01](0.01m) | vel=[0.10,0.01,-0.01] | | T=14.7N τ=[-0.001,0.008,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.008,-0.000]Nm | N=15 k_look=6 t_MPC=0.018s
t=58.3s | pos=[5.85,0.00,-5.00] err=[0.01,0.00,0.00](0.01m) | vel=[0.11,-0.01,0.00] | | T=14.7N τ=[-0.001,0.009,0.000]Nm
| T_virt=14.7N τ_virt=[-0.001,0.009,0.000]Nm | N=15 k_look=6 t_MPC=0.020s
Control log saved to: log_p08b_nmpc.mat

=== Metrics (P08b - NMPC Tracking (Multi Shooting)) ===
RMSE [ex, ey, ez] = [0.037, 0.015, 0.007] m
RMSE of norm(e)       = 0.041 m
Max err [ex, ey, ez] = [0.094, 0.043, 0.019] m
Max of norm(e)        = 0.094 m

Average control effort  mean of (u)   = 14.717
Max control effort      max(u)    = 14.731
Average thrust          mean(T)       = 14.717 N
Max thrust              max(T)        = 14.731 N
Average torque norm     mean(tau) = 0.010 Nm
Max torque norm         max(tau)  = 0.045 Nm
Average CPU time per step  = 0.1099 s
Max CPU time per step      = 0.1259 s
======================================

```


