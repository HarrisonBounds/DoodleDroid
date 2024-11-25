# DoodleDroid

## Tasks: Bold name is main contributor
### Image processor: Harrison and Yanni
  - [x] Takes in an image, and converts it to a line half-tone image
  - We may use this github repo:  https://github.com/GravO8/halftone-lines
  - Outputs the lines the robot should draw (along with their thicknesses)
### Calibrator: Yanni, Christian
  -  Determines the drawing surface plane by probing some points on the surface, similar to a 3D printer
### Motion Planner: David, Han, Harrison
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

## meeting notes
### 2024 11 25
- All 5 members in attendance
- Harrision:
  - [ ] will simplify number of lines in the line drawing.
  - [ ] will integrate into ros & debug imports
  - [ ] will share poly line outputs of the processed image
- Yanni:
  - has an april tag to ros reader with plane visualizer
  - [ ] will add multiple april tags and average readings
