# DC Motor System Identification and Closed-Loop Digital Control

Arduino and MATLAB code for speed control of a DC motor with a 100 PPR quadrature encoder (x1 decoding, 3.6° per tick), an L298N H-bridge and an Arduino Uno R3. Lab 6 builds the encoder interface and records the open-loop step response. Lab 7 fits a first-order model, designs three controllers by root locus, and runs them at 200 Hz.

Advanced Mechatronics Lab, Sessions 6 and 7, University of Tehran (Instructor: Dr. Ali Sadighi), February 2026. The full write-ups are in [`Mechatronics_Lab_6_Project_Report.pdf`](Mechatronics_Lab_6_Project_Report.pdf) and [`Mechatronics_Lab_7_Project_Report.pdf`](Mechatronics_Lab_7_Project_Report.pdf).

## Results

In Lab 6, going from PWM 100 to PWM 200 raised the speed from about 600 to 950 RPM, a 58% increase for twice the duty cycle. A full 12 V step (PWM 255) brought the motor to about 1260 RPM.

In Lab 7 the controllers were designed for the identified plant G(s) = 113.2 / (s + 18.05), in RPM per PWM unit. All three designs need more than 60° of phase, so each one is built from two identical stages that supply half of it.

| | Controller 1 | Controller 2 | Controller 3 |
|---|---|---|---|
| ζ, ωn (rad/s) | 0.4, 33 | 0.75, 42 | 0.24, 99 |
| Overshoot | 28 to 33% | 3 to 5% | unstable |
| Settling time | 290 to 320 ms | 150 to 170 ms | never settles |
| Steady-state error | 1.0 to 1.1% | 0.8 to 0.9% | 7 to 23% |

Controller 3 went into a sustained oscillation at 200 Hz. The Lab 7 report estimates it needs at least 328 Hz sampling.

## Repository layout

```
Arduino/
  lab6_task1_tick_counter/     tick counting with a hardware interrupt (50 µs debounce)
  lab6_task2_direction/        direction from the sign of the tick change
  lab6_task3_velocity/         speed by finite differencing every 100 ms
  lab6_task4_hbridge/          L298N forward / coast / reverse / brake test
  lab6_task5_pwm_speed/        open-loop PWM speed control, set with pwm=X
  lab6_task6_step_response/    100 Hz open-loop step response (PWM 0 to 255)
  lab7_closed_loop_control/    200 Hz closed-loop speed controller with data logging
Matlab/
  lab6_step_response_logger.m  logs the Task 6 stream, plots it, saves .mat and .csv
  lab7_identify_plant.m        fits a first-order model with tfest
  lab7_controller_design.m     root locus design, Tustin discretization, Arduino coefficients
  lab7_closed_loop_logger.m    reads the Lab 7 data block, plots it, computes overshoot and settling time
```

## Wiring

Every sketch uses the same pins:

```
Encoder A    -> D2 (INT0)
Encoder B    -> D4
Encoder VCC  -> 3.3 V
L298N ENA    -> D5 (PWM)
L298N IN1    -> D7
L298N IN2    -> D8
L298N supply -> 12 V, ground shared with the Arduino
```

## Workflow

1. Upload Lab 6 Tasks 1 to 5 one at a time and open the Serial Monitor at 115200 baud. Task 1 takes `help`, `show`, `ppr=X`, `inv=0|1` and `reset`. Task 5 takes `pwm=X`.
2. Upload `lab6_task6_step_response`, set `COM_PORT` in `lab6_step_response_logger.m` and run it. Opening the port resets the Uno, the step comes about 3 s later, and the script logs 2 s of data at 100 Hz.
3. Save a step response as `step_response_data.mat` with a vector `rpm`, set `Ts` and `STEP_PWM` at the top of `lab7_identify_plant.m`, and run it.
4. Run `lab7_controller_design.m`. For each controller it prints the difference equation and the `b0, b1, b2, a1, a2` lines to paste into `lab7_closed_loop_control`.
5. Set `TARGET_RPM` and the coefficients in `lab7_closed_loop_control` and upload it. Then set `serialPort` in `lab7_closed_loop_logger.m` and run it. Opening the port resets the Uno and starts the 2 s run. The script waits for the data block, plots the response and prints the overshoot and settling time.

## Requirements

- Arduino Uno and the Arduino IDE, with the TimerOne library (Library Manager) for `lab6_task6_step_response` and `lab7_closed_loop_control`
- MATLAB R2019b or newer (`serialport`), the Control System Toolbox for `lab7_controller_design.m`, and the System Identification Toolbox for `lab7_identify_plant.m`

## Author

Roozbeh Amini
