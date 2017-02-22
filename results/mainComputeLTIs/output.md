```
>> mainComputeLTIs
```

```
Wingspan computed from Tornado aircraft geometry: 1.5
Mean chord read from Tornado aircraft geometry:   0.25
 
Computing alpha-sweep for longitudinal dynamics...
alpha 4 deg, beta 0 deg
alpha 4.5 deg, beta 0 deg
alpha 5 deg, beta 0 deg
alpha 5.5 deg, beta 0 deg
alpha 6 deg, beta 0 deg
 
Searching for trimmed operating point of longitudinal dynamics...
Number of iterations: 4
Operating point specifications were successfully met.
 
Trimmed longitudinal state: 
============================
1) u     : velocity in x-body                       (m/s): 11.5418
2) w     : velocity in z-body                       (m/s): 1.0562
3) q     : rotation velocity y-body               (deg/s): -2.6692e-19
4) theta : euler pitch (angle body-x to earth-x)    (deg): 3.5754
 
Other parameters in trimmed longitudinal state: 
================================================
Alpha : angle of attack (velocity vector to earth-x) (deg): 5.2286
Absolute velocity (longitudinal)                     (m/s): 11.5901
Velocity in x-earth                                  (m/s): 11.5852
Velocity in z-earth                                  (m/s): 0.33438
Glide ratio                                               : 34.6472
 
Computing beta-sweep for lateral dynamics...
alpha 5.2286 deg, beta -1 deg
alpha 5.2286 deg, beta -0.5 deg
alpha 5.2286 deg, beta 0 deg
alpha 5.2286 deg, beta 0.5 deg
alpha 5.2286 deg, beta 1 deg
 
Searching for trimmed operating point of lateral dynamics...
Number of iterations: 3
Operating point specifications were successfully met.
 
Trimmed lateral state: 
============================
1) v     : velocity in y-body           (m/s): -0.0069615
2) p     : rotation velocity x-body   (deg/s): 0.0014094
3) phi   : euler roll angle             (deg): -1.5419
4) r     : rotation velocity z-body    deg/s): -0.022565
 
Other parameters in trimmed lateral state: 
================================================
Beta     : sideslip                        (deg): -0.034558
Absolute velocity (longitudinal & lateral) (m/s): 11.5901
 
Short Period Mode Properties (longitudinal): 
                                                 Pole 1      Pole 2
==============================================================================
Damping ratio                                 : 1  1
Undampted, natural frequency             (1/s): 6.8464      4.8203
Period                                     (s): Inf  Inf
Num. cycles to damp to half the amplitude     : 0  0
 
Phugoid Mode Properties (longitudinal): 
                                                 Pole 1      Pole 2
==============================================================================
Damping ratio                                 : 0.10757     0.10757
Undampted, natural frequency             (1/s): 0.43627     0.43627
Period                                     (s): 14.4863      14.4863
Num. cycles to damp to half the amplitude     : 1.0196      1.0196
 
Roll Mode Properties (lateral): 
                                                 Pole 1
==============================================================================
Damping ratio                                 : 1
Undampted, natural frequency             (1/s): 2.1251
Period                                     (s): Inf
Num. cycles to damp to half the amplitude     : 0
 
Spiral Mode Properties (lateral): 
                                                 Pole 1
==============================================================================
Damping ratio                                 : 1
Undampted, natural frequency             (1/s): 0.1132
Period                                     (s): Inf
Num. cycles to damp to half the amplitude     : 0
 
Dutch Roll Mode Properties (lateral): 
                                                 Pole 1      Pole 2
==============================================================================
Damping ratio                                 : 0.018588    0.018588
Undampted, natural frequency             (1/s): 2.3622      2.3622
Period                                     (s): 2.6603      2.6603
Num. cycles to damp to half the amplitude     : 5.9339      5.9339
 
Number of unobservable states (longitudinal)  : 0
Number of unobservable states (lateral)       : 0
 
Number of uncontrollable states (longitudinal): 0
Number of uncontrollable states (lateral)     : 0

```
