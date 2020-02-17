# cs378_starter

## Using this Repo
You will be using a fork of this repo for all development in the class. This repo provides starter code for the ROS subscriptions, publishers and control loops you will be using.

### Fork the Repo
1. Make sure you're logged into your GitHub account.
1. Click the Fork button on the upper right-hand side of this page.

### Clone and Build
1. `mkdir projects`
1. `cd projects`
1. `git clone <this repository url>` (found in the upper right)
1. `cd <cloned_repo>`
1. `make -j`

### Run
1. `roscore`
1. `cd ~/f1tenth_course/`
1. `./bin/websocket & ./bin/simulator`
1. `cd ~/projects/cs378_starter`
1. `bin/navigation --distance=6.28 --curvature=1.0 --obstacle` 
