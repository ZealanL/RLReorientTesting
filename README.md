# RLReorientTesting
A testing framework for the accuracy and speed of different air orientation methods in Rocket League

# Tested Methods
Currently, the following methods are tested:
 - [RLUtilities Hardcoded](https://github.com/samuelpmish/RLUtilities/blob/develop/src/mechanics/reorient.cc)
 - [RLUtilities ML](https://github.com/samuelpmish/RLUtilities/blob/develop/src/mechanics/reorient_ML.cc)

# Evaluation
 - Many test cases are generated, each with a random starting rotation, starting angular velocity, and target rotation
 - Each method is ran for every one of these cases
 - For each tick of each test case, error for that method is calculated using `QuatDist(currentRot, targetRot) * timeInTestCase`
 - Final error score is calculated with `totalErrorFromAllTests / testAmount`

# Simulation
 - To simulate aerial car control, [RocketSim](https://github.com/ZealanL/RocketSim) is used
