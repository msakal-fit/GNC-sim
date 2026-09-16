



## P01

```
run_P01_LQR
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Starting simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the LQR simulation...
t=0.0s | pos=[0.02,0.00,-4.99] err=[0.02,0.00,-3.99](3.99m) | vel=[0.00,-0.01,-0.01] | | T=2.9N τ=[0.014,0.012,-0.029]Nm [T_SAT]
t=8.3s | pos=[0.00,0.00,-2.19] err=[0.00,0.00,-1.19](1.19m) | vel=[0.01,-0.00,0.20] | | T=10.3N τ=[-0.003,0.006,0.000]Nm
t=16.7s | pos=[0.02,0.01,-1.33] err=[0.02,0.01,-0.33](0.33m) | vel=[-0.00,0.00,0.06] | | T=13.5N τ=[-0.010,0.020,-0.000]Nm
t=25.0s | pos=[0.02,-0.02,-1.08] err=[0.02,-0.02,-0.08](0.09m) | vel=[0.00,-0.00,0.01] | | T=14.4N τ=[0.032,0.035,0.000]Nm
t=33.3s | pos=[-0.00,0.03,-0.99] err=[-0.00,0.03,0.01](0.03m) | vel=[-0.00,-0.01,-0.00] | | T=14.8N τ=[-0.043,-0.006,-0.001]Nm
t=41.7s | pos=[-0.02,0.00,-0.99] err=[-0.02,0.00,0.01](0.02m) | vel=[0.00,-0.00,-0.00] | | T=14.8N τ=[0.000,-0.026,-0.002]Nm
t=50.0s | pos=[-0.02,0.00,-1.00] err=[-0.02,0.00,0.00](0.02m) | vel=[0.00,0.00,0.00] | | T=14.7N τ=[-0.009,-0.031,0.004]Nm
t=58.3s | pos=[-0.01,-0.04,-1.02] err=[-0.01,-0.04,-0.02](0.05m) | vel=[-0.00,0.02,0.00] | | T=14.6N τ=[0.061,-0.012,-0.002]Nm
Control log saved to: log_p01_lqr.mat

=== Metrics ( P01 - LQR) ===
RMSE [ex, ey, ez] = [0.016, 0.021, 1.006] m
RMSE of norm(e)       = 1.006 m
Max err [ex, ey, ez] = [0.039, 0.052, 3.989] m
Max of norm(e)        = 3.989 m

Average control effort  mean of (u)   = 13.049
Max control effort      max(u)    = 14.772
Average thrust          mean(T)       = 13.049 N
Max thrust              max(T)        = 14.772 N
Average torque norm     mean(tau) = 0.037 Nm
Max torque norm         max(tau)  = 0.082 Nm
Average CPU time per step  = 0.0909 s
Max CPU time per step      = 0.1069 s
======================================

Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode

```


## P04

```
>> run_P04_LMPC_point_multi
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the LMPC simulation...
Setting up LMPC with Ts=0.333 s and N=15
Time horizon: 5.000 seconds

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

t=0.0s | pos=[-0.00,0.03,-4.99] err=[-0.00,0.03,-3.99](3.99m) | vel=[0.03,-0.03,-0.01] | | T=7.4N τ=[0.002,-0.000,0.023]Nm
| T_virt=7.4N τ_virt=[0.002,-0.000,0.023]Nm | N=15 k_look=3 t_MPC=0.073s
t=8.3s | pos=[0.05,-0.02,-1.57] err=[0.05,-0.02,-0.57](0.57m) | vel=[-0.00,0.00,0.15] | | T=13.7N τ=[0.002,0.002,0.000]Nm
| T_virt=13.7N τ_virt=[0.002,0.002,0.000]Nm | N=15 k_look=3 t_MPC=0.017s
t=16.7s | pos=[0.04,0.02,-1.07] err=[0.04,0.02,-0.07](0.08m) | vel=[-0.00,0.00,0.01] | | T=14.6N τ=[0.001,0.002,-0.000]Nm
| T_virt=14.6N τ_virt=[0.001,0.002,-0.000]Nm | N=15 k_look=3 t_MPC=0.012s
t=25.0s | pos=[0.03,-0.05,-1.02] err=[0.03,-0.05,-0.02](0.06m) | vel=[0.01,-0.00,0.00] | | T=14.7N τ=[0.002,0.000,0.000]Nm
| T_virt=14.7N τ_virt=[0.002,0.000,0.000]Nm | N=15 k_look=3 t_MPC=0.017s
t=33.3s | pos=[0.04,-0.03,-0.99] err=[0.04,-0.03,0.01](0.04m) | vel=[0.00,0.00,-0.00] | | T=14.7N τ=[0.002,0.002,0.000]Nm
| T_virt=14.7N τ_virt=[0.002,0.002,0.000]Nm | N=15 k_look=3 t_MPC=0.016s
t=41.7s | pos=[0.09,-0.04,-0.98] err=[0.09,-0.04,0.02](0.10m) | vel=[-0.00,0.00,-0.01] | | T=14.7N τ=[0.003,0.004,-0.000]Nm
| T_virt=14.7N τ_virt=[0.003,0.004,-0.000]Nm | N=15 k_look=3 t_MPC=0.015s
t=50.0s | pos=[0.03,-0.07,-1.00] err=[0.03,-0.07,0.00](0.08m) | vel=[-0.00,-0.00,-0.00] | | T=14.7N τ=[0.004,0.002,-0.000]Nm
| T_virt=14.7N τ_virt=[0.004,0.002,-0.000]Nm | N=15 k_look=3 t_MPC=0.013s
t=58.3s | pos=[0.09,-0.05,-1.00] err=[0.09,-0.05,-0.00](0.10m) | vel=[-0.00,-0.00,0.00] | | T=14.7N τ=[0.003,0.004,-0.000]Nm
| T_virt=14.7N τ_virt=[0.003,0.004,-0.000]Nm | N=15 k_look=3 t_MPC=0.014s
Control log saved to: log_p04a_lmpc.mat

=== Metrics ( P04a - LMPC Multi-Shooting) ===
RMSE [ex, ey, ez] = [0.055, 0.035, 0.824] m
RMSE of norm(e)       = 0.827 m
Max err [ex, ey, ez] = [0.092, 0.072, 3.990] m
Max of norm(e)        = 3.990 m

Average control effort  mean of (u)   = 14.181
Max control effort      max(u)    = 14.734
Average thrust          mean(T)       = 14.181 N
Max thrust              max(T)        = 14.734 N
Average torque norm     mean(tau) = 0.004 Nm
Max torque norm         max(tau)  = 0.041 Nm
Average CPU time per step  = 0.1069 s
Max CPU time per step      = 0.1708 s
======================================
```


## P06

```
>> run_P06_NMPC
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Setting up NMPC with Ts=0.333 s and N=15
Time horizon: 5.000 seconds
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the NMPC-Point Stabilization simulation...
t=0.0s | pos=[0.00,-0.00,-5.00] err=[0.00,-0.00,-4.00](4.00m) | vel=[0.00,-0.02,-0.00] | | T=7.4N τ=[0.002,0.000,0.004]Nm
| T_virt=7.4N τ_virt=[0.002,0.000,0.004]Nm | N=15 k_look=3 t_MPC=0.350s
t=8.3s | pos=[0.01,-0.02,-1.28] err=[0.01,-0.02,-0.28](0.28m) | vel=[0.00,0.00,0.10] | | T=14.3N τ=[0.000,0.000,0.000]Nm
| T_virt=14.3N τ_virt=[0.000,0.000,0.000]Nm | N=15 k_look=3 t_MPC=0.032s
t=16.7s | pos=[-0.02,-0.01,-1.02] err=[-0.02,-0.01,-0.02](0.03m) | vel=[0.01,0.00,0.00] | | T=14.7N τ=[0.001,-0.001,0.000]Nm
| T_virt=14.7N τ_virt=[0.001,-0.001,0.000]Nm | N=15 k_look=3 t_MPC=0.034s
t=25.0s | pos=[-0.02,-0.00,-0.98] err=[-0.02,-0.00,0.02](0.02m) | vel=[-0.00,-0.00,-0.01] | | T=14.7N τ=[0.001,-0.001,0.000]Nm
| T_virt=14.7N τ_virt=[0.001,-0.001,0.000]Nm | N=15 k_look=3 t_MPC=0.041s
t=33.3s | pos=[-0.01,0.02,-1.01] err=[-0.01,0.02,-0.01](0.02m) | vel=[0.00,-0.01,0.01] | | T=14.7N τ=[-0.000,-0.000,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.000,-0.000]Nm | N=15 k_look=3 t_MPC=0.023s
t=41.7s | pos=[0.01,-0.01,-0.99] err=[0.01,-0.01,0.01](0.02m) | vel=[-0.00,0.00,-0.00] | | T=14.7N τ=[0.000,0.001,0.000]Nm
| T_virt=14.7N τ_virt=[0.000,0.001,0.000]Nm | N=15 k_look=3 t_MPC=0.026s
t=50.0s | pos=[-0.01,-0.00,-0.99] err=[-0.01,-0.00,0.01](0.01m) | vel=[0.00,-0.00,-0.00] | | T=14.7N τ=[-0.000,-0.000,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.000,0.000]Nm | N=15 k_look=3 t_MPC=0.030s
t=58.3s | pos=[-0.01,0.01,-1.00] err=[-0.01,0.01,-0.00](0.02m) | vel=[0.01,0.01,0.00] | | T=14.7N τ=[-0.000,-0.001,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.001,-0.000]Nm | N=15 k_look=3 t_MPC=0.032s
Control log saved to: log_p06_nmpc.mat

=== Metrics ( P06 - NMPC Single Shooting) ===
RMSE [ex, ey, ez] = [0.016, 0.017, 0.748] m
RMSE of norm(e)       = 0.748 m
Max err [ex, ey, ez] = [0.035, 0.039, 4.001] m
Max of norm(e)        = 4.001 m

Average control effort  mean of (u)   = 14.330
Max control effort      max(u)    = 14.749
Average thrust          mean(T)       = 14.330 N
Max thrust              max(T)        = 14.749 N
Average torque norm     mean(tau) = 0.001 Nm
Max torque norm         max(tau)  = 0.006 Nm
Average CPU time per step  = 0.1282 s
Max CPU time per step      = 0.5475 s
======================================

```


## P08

```
>> run_P08a_point_multi
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Setting up NMPC with Ts=0.333 s and N=15
Time horizon: 5.000 seconds
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the NMPC Point Stab. Multi simulation...
t=0.0s | pos=[-0.01,0.00,-4.99] err=[-0.01,0.00,-3.99](3.99m) | vel=[0.02,-0.01,-0.00] | | T=7.4N τ=[0.001,0.002,0.002]Nm
| T_virt=7.4N τ_virt=[0.001,0.002,0.002]Nm | N=15 k_look=3 t_MPC=0.029s
t=8.3s | pos=[0.00,0.03,-1.25] err=[0.00,0.03,-0.25](0.25m) | vel=[-0.01,0.00,0.09] | | T=14.3N τ=[0.000,0.001,0.000]Nm
| T_virt=14.3N τ_virt=[0.000,0.001,0.000]Nm | N=15 k_look=3 t_MPC=0.016s
t=16.7s | pos=[0.00,-0.01,-1.01] err=[0.00,-0.01,-0.01](0.02m) | vel=[0.00,-0.00,0.00] | | T=14.7N τ=[0.000,-0.000,0.000]Nm
| T_virt=14.7N τ_virt=[0.000,-0.000,0.000]Nm | N=15 k_look=3 t_MPC=0.015s
t=25.0s | pos=[-0.02,0.02,-1.02] err=[-0.02,0.02,-0.02](0.04m) | vel=[0.01,-0.00,0.01] | | T=14.7N τ=[-0.000,-0.001,0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,-0.001,0.000]Nm | N=15 k_look=3 t_MPC=0.017s
t=33.3s | pos=[-0.01,-0.01,-0.99] err=[-0.01,-0.01,0.01](0.02m) | vel=[-0.01,-0.00,-0.01] | | T=14.7N τ=[0.001,0.000,0.000]Nm
| T_virt=14.7N τ_virt=[0.001,0.000,0.000]Nm | N=15 k_look=3 t_MPC=0.020s
t=41.7s | pos=[0.04,-0.01,-1.01] err=[0.04,-0.01,-0.01](0.04m) | vel=[-0.00,-0.01,0.00] | | T=14.7N τ=[0.001,0.001,0.001]Nm
| T_virt=14.7N τ_virt=[0.001,0.001,0.001]Nm | N=15 k_look=3 t_MPC=0.019s
t=50.0s | pos=[0.00,-0.00,-0.99] err=[0.00,-0.00,0.01](0.01m) | vel=[0.00,0.01,-0.00] | | T=14.7N τ=[0.001,-0.000,0.000]Nm
| T_virt=14.7N τ_virt=[0.001,-0.000,0.000]Nm | N=15 k_look=3 t_MPC=0.013s
t=58.3s | pos=[0.01,0.02,-1.01] err=[0.01,0.02,-0.01](0.03m) | vel=[0.00,0.00,0.00] | | T=14.7N τ=[-0.000,0.001,-0.000]Nm
| T_virt=14.7N τ_virt=[-0.000,0.001,-0.000]Nm | N=15 k_look=3 t_MPC=0.015s
Control log saved to: log_p08a_nmpc.mat

=== Metrics ( P08a - NMPC Multi-Shooting) ===
RMSE [ex, ey, ez] = [0.021, 0.022, 0.705] m
RMSE of norm(e)       = 0.706 m
Max err [ex, ey, ez] = [0.055, 0.054, 3.993] m
Max of norm(e)        = 3.993 m

Average control effort  mean of (u)   = 14.369
Max control effort      max(u)    = 14.739
Average thrust          mean(T)       = 14.369 N
Max thrust              max(T)        = 14.739 N
Average torque norm     mean(tau) = 0.001 Nm
Max torque norm         max(tau)  = 0.004 Nm
Average CPU time per step  = 0.1084 s
Max CPU time per step      = 0.1219 s
======================================

```