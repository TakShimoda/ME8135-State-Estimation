# State-Estimation
Practice programs for simple state estimation problems in robotics.

Reference: *State Estimation for Robotics (2020)* by Timothy D. Barfoot.

Link to youtube playlist: 

https://www.youtube.com/watch?v=wGZgDfA37xA&list=PLOxq1EUcxzrKmPDE9FOWAIaEXC9lrWu0h&ab_channel=GlennShimoda

# Implement Code

Clone the repository:

```bash
git clone https://github.com/TakShimoda/ME8135-State-Estimation.git
```

Create a virtual environment:

```bash
pip install virtualenv
```

```bash
cd ME8135-State-Estimation
```

```bash
virtualenv ME8135
```

```bash
ME8135\Scripts\activate
```

Install the dependencies:

```bash
install -r requirements.txt
```

# Assignments Overview

__HW1__: Basic Kalman Filter for a linear motion robot. Uses linear motion and observation models and the robot moves in a straight line diagonally.

<img src="Images/Kalman_Filter.JPG" width="500">

__HW2__: Extended Kalman Filter, using both the linear measurement model from HW1 and a new measurement model that uses distance and bearing between the robot and a landmark.

<img src="Images/EKF_b.JPG" width="500">
