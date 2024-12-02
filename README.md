# DoodleDroid

https://docs.google.com/document/d/1tCsjUFVBGqud-HylWmlEa1OkyHOy-n-hFuSvpV8IUJU/edit?usp=sharing
## Tasks: Bold name is main contributor
### Image processor: Harrison and Yanni
  - [x] Takes in an image, and converts it to a line half-tone image
  - We may use this github repo:  https://github.com/GravO8/halftone-lines
  - Outputs the lines the robot should draw (along with their thicknesses)
### Calibrator: Yanni, Christian
  -  Determines the drawing surface plane by probing some points on the surface, similar to a 3D printer
### route Planner: David, Han, Harrison
  - Preliminary:
    - [x] use MoveIt to generate test drawing sequences (Han: Due Nov 25th; done nov 25th)
    - [x] Lines -> line route (David: Due Nov 25; Done Nov 22)
    - [x] Calibration -> querable height map (David: Done Dec 1).
    - [x] Line route + height map -> xyz path (David: Done Dec 1)
  - [ ] integrate height calibration node into route planner
    - [ ] calibration node -> route planner service or topic schema
  - [ ] integrate image pipeline into route planner.
    - [x] image -> route planner hand-tested; (David, Harrison; Done Dec 1)
    - [ ] image node -> route planner Service schema
  - [ ] (stretch-contigent) line thicknesses.
  - [ ] (stretch-contigent) thickness sequence -> target forces.
### Path Executor: Han, Christian, David
  - [x] XYZ+  route -> joint sequence (Han: Done late Nov)
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

### Extra Notes 

```
sudo apt install ros-jazzy-usb-cam
```
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
