# Test line tracking with the same Q, R

Change logs


## P02
```
>> run_P02_LQT
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting manual control...
Starting the LQT simulation...
t=0.0s | pos=[-0.00,-0.02,-5.00] err=[-0.00,-0.02,-0.00](0.02m) | vel=[-0.00,-0.00,0.00] | | T=14.8N τ=[0.006,-0.522,0.297]Nm
t=8.3s | pos=[0.75,-0.00,-4.93] err=[-0.08,-0.00,0.07](0.11m) | vel=[0.10,-0.00,0.01] | | T=14.8N τ=[0.003,-0.518,-0.001]Nm
t=16.7s | pos=[1.60,-0.01,-4.88] err=[-0.07,-0.01,0.12](0.14m) | vel=[0.10,-0.01,0.00] | | T=14.9N τ=[0.012,-0.483,-0.002]Nm
t=25.0s | pos=[2.42,-0.01,-4.84] err=[-0.08,-0.01,0.16](0.18m) | vel=[0.11,0.00,-0.00] | | T=15.0N τ=[0.019,-0.483,-0.000]Nm
t=33.3s | pos=[3.22,-0.02,-4.79] err=[-0.12,-0.02,0.21](0.24m) | vel=[0.12,0.00,0.01] | | T=15.1N τ=[0.022,-0.632,0.000]Nm
t=41.7s | pos=[4.07,-0.04,-4.82] err=[-0.10,-0.04,0.18](0.21m) | vel=[0.10,0.00,0.01] | | T=15.0N τ=[0.060,-0.609,-0.003]Nm
t=50.0s | pos=[4.90,-0.01,-4.78] err=[-0.10,-0.01,0.22](0.24m) | vel=[0.11,0.01,-0.00] | | T=15.1N τ=[0.010,-0.555,-0.002]Nm
t=58.3s | pos=[5.74,-0.01,-4.78] err=[-0.09,-0.01,0.22](0.24m) | vel=[0.10,0.00,0.00] | | T=15.1N τ=[0.017,-0.572,-0.001]Nm
Control log saved to: log_p02_lqt.mat

=== Metrics (LQT Controller) ===
RMSE [ex, ey, ez] = [0.084, 0.016, 0.165] m
RMSE of norm(e)       = 0.186 m
Max err [ex, ey, ez] = [0.117, 0.039, 0.242] m
Max of norm(e)        = 0.266 m

Average control effort  mean of (u)   = 14.972
Max control effort      max(u)    = 15.104
Average thrust          mean(T)       = 14.963 N
Max thrust              max(T)        = 15.092 N
Average torque norm     mean(tau) = 0.522 Nm
Max torque norm         max(tau)  = 0.644 Nm
Average CPU time per step  = 0.0894 s
Max CPU time per step      = 0.0964 s

```


## P07

```
run_P07_NMPC
Connecting to 127.0.1.1:8766...
Connected to 127.0.1.1:8766
Starting Simulation...
Control mode: position
Takeoff complete at 5.0m
Switched to position control mode
Starting NMPC tracking control...
Starting the NMPC Single Shooting simulation...
t=0.0s | pos=[0.00,-0.03,-5.01] err=[0.00,-0.03,-0.01](0.03m) | vel=[-0.00,0.01,0.01] | | T=14.7N τ=[-0.000,-0.010,0.027]Nm
t=8.3s | pos=[0.06,0.01,-5.00] err=[-0.77,0.01,-0.00](0.77m) | vel=[0.01,-0.00,-0.00] | | T=14.9N τ=[-0.001,-0.088,-0.000]Nm
t=16.7s | pos=[0.21,0.04,-5.08] err=[-1.46,0.04,-0.08](1.46m) | vel=[0.02,0.00,-0.01] | | T=15.1N τ=[-0.002,-0.114,0.000]Nm
t=25.0s | pos=[0.40,0.08,-5.22] err=[-2.10,0.08,-0.22](2.11m) | vel=[0.02,0.00,-0.01] | | T=15.3N τ=[-0.018,-0.137,0.029]Nm
t=33.3s | pos=[0.67,0.02,-5.46] err=[-2.66,0.02,-0.46](2.70m) | vel=[0.02,0.01,-0.02] | | T=15.6N τ=[0.088,-0.161,-0.176]Nm
t=41.7s | pos=[1.10,0.07,-5.76] err=[-3.07,0.07,-0.76](3.16m) | vel=[0.02,-0.04,-0.02] | | T=15.7N τ=[-0.099,-0.178,0.186]Nm
t=50.0s | pos=[1.53,-0.01,-6.11] err=[-3.47,-0.01,-1.11](3.64m) | vel=[0.00,0.02,-0.02] | | T=15.8N τ=[0.107,-0.194,-0.194]Nm
t=58.3s | pos=[1.93,0.07,-6.47] err=[-3.90,0.07,-1.47](4.17m) | vel=[0.02,-0.01,-0.01] | | T=15.7N τ=[-0.121,-0.200,0.200]Nm
Control log saved to: log_p07_nmpc.mat

=== Metrics (P07 - NMPC Single Shooting) ===
RMSE [ex, ey, ez] = [2.523, 0.042, 0.696] m
RMSE of norm(e)       = 2.618 m
Max err [ex, ey, ez] = [3.974, 0.088, 1.534] m
Max of norm(e)        = 4.260 m

Average control effort  mean of (u)   = 15.349
Max control effort      max(u)    = 15.844
Average thrust          mean(T)       = 15.348 N
Max thrust              max(T)        = 15.841 N
Average torque norm     mean(tau) = 0.192 Nm
Max torque norm         max(tau)  = 0.307 Nm
Average CPU time per step  = 0.5486 s
Max CPU time per step      = 0.9012 s
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
Starting the NMPC simulation...
t=0.0s | pos=[-0.00,-0.01,-5.00] err=[-0.00,-0.01,0.00](0.01m) | vel=[-0.00,0.01,-0.00] | | T=14.7N τ=[-0.000,-0.013,0.005]Nm
t=8.3s | pos=[0.70,-0.00,-4.99] err=[-0.13,-0.00,0.01](0.14m) | vel=[0.09,0.01,-0.01] | | T=14.7N τ=[-0.002,-0.021,0.000]Nm
t=16.7s | pos=[1.52,0.01,-4.99] err=[-0.15,0.01,0.01](0.15m) | vel=[0.10,-0.02,-0.01] | | T=14.7N τ=[-0.001,-0.022,-0.000]Nm
t=25.0s | pos=[2.36,0.01,-5.01] err=[-0.14,0.01,-0.01](0.14m) | vel=[0.10,0.01,0.01] | | T=14.7N τ=[-0.002,-0.021,-0.000]Nm
t=33.3s | pos=[3.20,-0.01,-5.00] err=[-0.13,-0.01,0.00](0.13m) | vel=[0.10,0.01,-0.00] | | T=14.7N τ=[-0.001,-0.021,0.000]Nm
t=41.7s | pos=[4.05,-0.01,-4.99] err=[-0.12,-0.01,0.01](0.12m) | vel=[0.10,0.02,-0.01] | | T=14.7N τ=[-0.001,-0.021,-0.000]Nm
t=50.0s | pos=[4.89,-0.00,-5.00] err=[-0.11,-0.00,0.00](0.11m) | vel=[0.08,-0.00,-0.00] | | T=14.7N τ=[-0.001,-0.020,0.000]Nm
t=58.3s | pos=[5.71,0.02,-4.99] err=[-0.12,0.02,0.01](0.12m) | vel=[0.10,-0.01,-0.01] | | T=14.7N τ=[-0.002,-0.021,0.000]Nm
Control log saved to: log_p08b_nmpc.mat

=== Metrics (P08b - NMPC Tracking (Multi Shooting)) ===
RMSE [ex, ey, ez] = [0.131, 0.014, 0.008] m
RMSE of norm(e)       = 0.132 m
Max err [ex, ey, ez] = [0.176, 0.032, 0.028] m
Max of norm(e)        = 0.178 m

Average control effort  mean of (u)   = 14.732
Max control effort      max(u)    = 14.767
Average thrust          mean(T)       = 14.732 N
Max thrust              max(T)        = 14.767 N
Average torque norm     mean(tau) = 0.021 Nm
Max torque norm         max(tau)  = 0.026 Nm
Average CPU time per step  = 0.1093 s
Max CPU time per step      = 0.1177 s
======================================

```