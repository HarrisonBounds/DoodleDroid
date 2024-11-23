# DoodleDroid

## Tasks: Bold name is main contributor
### Image processor: Harrison and Yanni
  - Takes in an image, and converts it to a line half-tone image
  - We may use this github repo:  https://github.com/GravO8/halftone-lines
  - Outputs the lines the robot should draw (along with their thicknesses)
### Calibrator: Yanni, Christian
  -  Determines the drawing surface plane by probing some points on the surface, similar to a 3D printer
### Motion Planner: Han, David, Harrison
  - Preliminary:
    - [ ] use MoveIt to generate test drawing sequences (Han: Due Nov 25th)
    - [x] Lines -> line route (David: Due Nov 25; Done Nov 22)
  - [ ] Calibration -> querable height map (David: no deadline).
  - [ ] Line route + height map + thicknesses -> XYZ route + thicknesses
  - [ ] XYZ+ thickness route -> joint sequence
  - [ ] (stretch-contigent) thickness sequence -> target forces.
### Path Executor: Han, Christian, David
  -  Executes the robot joint trajectories
### Stretch
#### Force Control (stretch): David
  - [ ] Pending evaluation of if single-width lines are sufficient.
#### Image input (stretch): Harrison and Yanni

## Secondary roles:
Dev-ops:  David
Reliability: Harrison
Code hygienist: Christian
Integration: Han
Hardware Engineer: Yanni
