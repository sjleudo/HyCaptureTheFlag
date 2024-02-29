# Hybrid Capture The Flag
Simulation tool to implement the hybrid model and test controllers submitted as a regular paper to the conference ADHS 2024.

  <img src="https://github.com/sjleudo/HyCaptureTheFlag/blob/main/5.gif" width="400">

Simulation M-files for numerical simulation in ADHS'24 paper: A hybrid system and a zero-sum game
formulation for Capture-the-Flag games 

Author: Santiago Jimenez Leudo
Revision: 0.0.0.1 Date: 01/22/2024 01:19:00
https://github.com/sjimen28/HyCaptureTheFlag

Requirements: Matlab (Developed in R2020b)

Content: (Download as a folder and add it to the Path in Matlab)
  - HybridCaptureTheFlag.m
  - Robot_Dynamics.m
  - Tagging_Ability.m
  - pointInCircle.m
  - pointInRectangle.m
  - Angle_Control.m
  - Positions_in_Matrix.m
  - multiplyFirstElement.m
  - Closest_Opponent.m
  - calculateAngle.m


----------------------------------------------------------------------------
### Numerical Simulation (3v3 Capture the Flag Game)
Run 'HybridCaptureTheFlag.m'

Scenarios:

- **Tagging:** After the blue robot tags the red robot, the color of the latter changes to black (denoting it is deactivated), and its controller leads it to head back to the red base. The blue robot's color changes to green (denoting its timeout).

     <img src="https://github.com/sjleudo/HyCaptureTheFlag/blob/main/Tagging.gif" width="200">

- **Leaving the field and being reactivated:** When the blue robot exits the playing field, its color turns black (denoting it is deactivated). The controller leads it to the blue base, and as soon as it reaches it, it turns back blue (denoting it has been reactivated).
  
    <img src="https://github.com/sjleudo/HyCaptureTheFlag/blob/main/LeavingField-Reactivated.gif" width="200">

- **Capture the flag:** When any robot enters the opponent's flag base, and the flag is in position (denoted by an x marker at the center of the base), the attacking robot captures the flag, and its marker turns into a star. The controller leads it back to its base while avoiding defending opponents.
  
    <img src="https://github.com/sjleudo/HyCaptureTheFlag/blob/main/CaptureTheFlag.gif" width="400">

- **Dopping the flag:** While a robot carries the flag, it approaches its base. As soon as it enters it, the flag is dropped and returned to the opponents base. The robot's marker turns back to a circle. 

    <img src="https://github.com/sjleudo/HyCaptureTheFlag/blob/main/DroppingFlag.gif" width="400">


