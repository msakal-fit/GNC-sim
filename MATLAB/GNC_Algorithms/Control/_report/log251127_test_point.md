# Test of Point Stabalization with the same Q, R

Change logs
- add more metrics for control effort and CPU time

## P01
```
=== Metrics ( P01 - LQR) ===
RMSE [ex, ey, ez] = [0.017, 0.020, 1.005] m
RMSE of norm(e)       = 1.005 m
Max err [ex, ey, ez] = [0.038, 0.053, 4.003] m
Max of norm(e)        = 4.003 m

Average control effort  mean of (u)   = 13.035
Max control effort      max(u)    = 14.704
Average thrust          mean(T)       = 13.031 N
Average torque norm     mean(tau) = 0.065 Nm
Max torque norm         max(tau)  = 1.239 Nm
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode
>> plot_point_stab_results('log_p01_lqr.mat', ' P01 - LQR');

=== Metrics ( P01 - LQR) ===
RMSE [ex, ey, ez] = [0.017, 0.020, 1.005] m
RMSE of norm(e)       = 1.005 m
Max err [ex, ey, ez] = [0.038, 0.053, 4.003] m
Max of norm(e)        = 4.003 m

Average control effort  mean of (u)   = 13.035
Max control effort      max(u)    = 14.704
Average thrust          mean(T)       = 13.031 N
Average torque norm     mean(tau) = 0.065 Nm
Max torque norm         max(tau)  = 1.239 Nm
Average CPU time per step  = 0.0899 s
Max CPU time per step      = 0.1017 s
======================================
```


## P06

```
>> run_P06_NMPC
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Setting up NMPC with Ts=0.333 s and N=25
Time horizon: 8.333 seconds
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the NMPC-Point Stabilization simulation...

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

t=0.0s | pos=[-0.00,0.02,-4.99] err=[-0.00,0.02,-3.99](3.99m) | vel=[0.02,-0.00,-0.01] | | T=12.4N τ=[0.091,-0.128,-0.098]Nm
t=8.3s | pos=[0.04,-0.01,-2.93] err=[0.04,-0.01,-1.93](1.93m) | vel=[-0.03,-0.03,0.06] | | T=9.6N τ=[0.001,-0.001,-0.000]Nm
t=16.7s | pos=[0.02,-0.00,-1.67] err=[0.02,-0.00,-0.67](0.67m) | vel=[-0.01,-0.00,0.04] | | T=13.0N τ=[-0.000,0.002,0.000]Nm
t=25.0s | pos=[0.04,0.02,-1.28] err=[0.04,0.02,-0.28](0.29m) | vel=[-0.00,-0.00,0.01] | | T=14.0N τ=[-0.001,0.003,0.000]Nm
t=33.3s | pos=[0.04,0.04,-1.12] err=[0.04,0.04,-0.12](0.13m) | vel=[0.00,-0.00,0.01] | | T=14.4N τ=[-0.002,0.002,-0.000]Nm
t=41.7s | pos=[0.05,0.02,-1.07] err=[0.05,0.02,-0.07](0.09m) | vel=[-0.00,-0.00,0.01] | | T=14.6N τ=[-0.001,0.003,0.000]Nm
t=50.0s | pos=[0.10,0.02,-1.03] err=[0.10,0.02,-0.03](0.10m) | vel=[0.00,0.00,-0.00] | | T=14.6N τ=[-0.001,0.004,-0.000]Nm
t=58.3s | pos=[0.09,0.02,-0.99] err=[0.09,0.02,0.01](0.10m) | vel=[-0.00,-0.00,-0.00] | | T=14.7N τ=[-0.001,0.005,0.000]Nm
Control log saved to: log_p06_nmpc.mat

=== Metrics ( P06 - NMPC) ===
RMSE [ex, ey, ez] = [0.051, 0.023, 1.324] m
RMSE of norm(e)       = 1.325 m
Max err [ex, ey, ez] = [0.113, 0.050, 3.989] m
Max of norm(e)        = 3.989 m

Average control effort  mean of (u)   = 13.310
Max control effort      max(u)    = 14.748
Average thrust          mean(T)       = 13.309 N
Average torque norm     mean(tau) = 0.014 Nm
Max torque norm         max(tau)  = 0.222 Nm
Average CPU time per step  = 0.5106 s
Max CPU time per step      = 1.9937 s
======================================

Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode

```


## P08a

```
>> run_P08a_point_multi
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Setting up NMPC with Ts=0.333 s and N=25
Time horizon: 8.333 seconds
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the NMPC Point Stab. Multi simulation...
t=0.0s | pos=[-0.02,0.01,-5.00] err=[-0.02,0.01,-4.00](4.00m) | vel=[0.01,0.02,-0.00] | | T=7.4N τ=[-0.004,0.000,0.005]Nm
t=8.3s | pos=[0.03,0.00,-2.83] err=[0.03,0.00,-1.83](1.83m) | vel=[-0.00,0.01,0.19] | | T=10.3N τ=[0.000,0.002,-0.000]Nm
t=16.7s | pos=[0.01,-0.02,-1.81] err=[0.01,-0.02,-0.81](0.81m) | vel=[-0.00,0.00,0.09] | | T=12.8N τ=[0.001,0.001,-0.000]Nm
t=25.0s | pos=[0.05,-0.03,-1.35] err=[0.05,-0.03,-0.35](0.36m) | vel=[-0.00,-0.00,0.04] | | T=13.9N τ=[0.001,0.003,0.000]Nm
t=33.3s | pos=[0.06,0.01,-1.14] err=[0.06,0.01,-0.14](0.15m) | vel=[0.00,-0.00,0.02] | | T=14.4N τ=[-0.000,0.003,0.000]Nm
t=41.7s | pos=[0.08,0.01,-1.08] err=[0.08,0.01,-0.08](0.12m) | vel=[-0.01,-0.00,0.01] | | T=14.5N τ=[-0.001,0.005,0.000]Nm
t=50.0s | pos=[0.10,0.06,-1.04] err=[0.10,0.06,-0.04](0.13m) | vel=[-0.00,-0.00,0.00] | | T=14.6N τ=[-0.002,0.006,-0.000]Nm
t=58.3s | pos=[0.10,0.13,-1.03] err=[0.10,0.13,-0.03](0.17m) | vel=[-0.00,0.00,0.00] | | T=14.6N τ=[-0.006,0.006,-0.000]Nm
Control log saved to: log_p08a_nmpc.mat

=== Metrics ( P08a - NMPC Multi-Shooting) ===
RMSE [ex, ey, ez] = [0.067, 0.048, 1.232] m
RMSE of norm(e)       = 1.235 m
Max err [ex, ey, ez] = [0.127, 0.143, 3.998] m
Max of norm(e)        = 3.998 m

Average control effort  mean of (u)   = 13.075
Max control effort      max(u)    = 14.672
Average thrust          mean(T)       = 13.075 N
Average torque norm     mean(tau) = 0.004 Nm
Max torque norm         max(tau)  = 0.010 Nm
Average CPU time per step  = 0.1139 s
Max CPU time per step      = 0.1341 s
======================================

Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Switched to position control mode

```