# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Algorithm Breakdown


##### main.cpp

`int main(int argc, char* argv[]) {`

1. Ensure program arguments correspond to valid files by inspecting `argv[1]` and `argv[2]` values

2. Assign the first argument `argv[1]` to the in_file_name_ (_data/sample-laser-radar-measurement-data-1.txt_) string 
variable

3. Intialize the `in_file_`'s input file stream (`ifstream`) variable to read the input data

4. Assign the second argument `argv[2]` to the `out_file_name_` string variable 

5. Initialize the `out_file_` output file stream (`ofstream`) variable to write output data to

6. Determine whether `in_file_` and `out_file_` are open. If one is, print an error message and exit the program via 
`exit(EXIT_FAILURE)`

7. Initialize the `std::vector`s `measurement_pack_list<MeasurementPackage>` and `gt_pack_list<GroundTruthPackage>`

8. Using [istream::getline](http://www.cplusplus.com/reference/istream/istream/getline/), prepare the measurement 
packages (each line represents a measurement at a timestamp) by iterating over `in_file_`

    a. Inside the while loop, the following occurs:
    
       i. declare the `sensor_type` variable which will be equal to either "L" or "R"
       
       ii. initialize the meas_package and gt_package variables
       
       iii. initialize the iss variable of type istringstream representing the current string line being processed
       
       iv. initialize a timestamp variable of type long representing the measurement time
       
       v. read in the the first character assigning it to sensor_type:

            1. if "L" (LIDAR measurement):
            
                a. Declare MeasurementPackage#sensor_type_ variable equal to MeasurementPackage::LASER
                
                b. Initialize MeasurementPackage#raw_measurements_ to a 2D VectorXd(2)
                
                c. Read in the next 2 sections corresponding to the raw position measurments px and py of type float
                
                d. Assign the MeasurementPackage#raw_measurements_ vector the measurement values px and py
                
                e. Read in the timestamp corresponding to the LIDAR measurement time and assign it to 
                   MeasurementPackage#timestamp_

                f. Push the meas_package variable declared in 8a(ii) onto the measurement_pack_list<MeasurementPackage> 
                   vector initialzied in 7.
                   
            2. if "R" (RADAR measurement):

                a. Declare MeasurementPackage#sensor_type_ variable equal to MeasurementPackage::RADAR
                
                b. Initialize MeasurementPackage#raw_measurements_ to a 3D VectorXd(3)
                
                c. Read in the next 3 sections corresponding to the raw position measurments rho (range), phi (bearing) 
                   and rho_dot (range rate) of type float
                   
                d. Assign the MeasurementPackage#raw_measurements_ vector the measurement values rho, phi and rho_dot
                
                e. Read in the timestamp corresponding to the RADAR measurement time and assign it to 
                   MeasurementPackage#timestamp_

                f. Push the meas_package variable declared in 8a(ii) onto the measurement_pack_list<MeasurementPackage> 
                   vector initialzied in 7.
                   
       vi. Read ground truth data to compare later
       
            1. Declares 4 variables corresponding to the ground truth position and velocity: 
               
               (px_gt, py_gt, vx_gt, vy_gt)^T
               
            2. Initialize GroundTruth#gt_values_ vector to a 4D VectorXd(4)
            
            3. Assign px_gt, py_gt, vx_gt and vy_gt to gt_values_ vector
            
9. Instantiate an instance of the `FusionEKF` class to the variable `fusionEKF`

10. Initialize our `estimations` and `ground_truth` RMSE results of type `vector<VectorXd>`. 
    __***We are ultimately graded on these numbers!***__

11. *Call the EKF-based fusion* by iterating over each item in `measurement_pack_list` 
    (of type `vector<MeasurementPackage>`). The following occurs inside this for loop:

    a. Start filtering from the second frame (the speed is unknown in the first frame)
        
    ```
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);
    ```

    b. Write to `out_file_` (e.g., _build/output.txt_) the estimated position and velocity values of the 
       `KalmanFilter#x_` vector representing the state of the vehicle `(px, py, vx, vy)^T` after performing the 
       prediction and measurement updates for both Lidar and Radar measurements. Note each item in 
       the `KalmanFilter#x_` vector is printed to the `ofstream` separated by tabs (via `"\t"` at the end of each line).
       
    c. Invoke `MeasurementPackage#estimations` and write out the new `px` and `py` estimations performed in *11a* above 
       (__VERIFY ACCURACY OF THIS STATEMENT__). 

        i. I added the MeasurementPacakage#estimations method to DRY up the code a bit and encapsulate the RADAR 
           sensor's polor-to-cartesian conversion inside the MeasurementPackage object itself.
        
        ii. Inside this method, if the current measurement's sensor type is RADAR, convert the RADAR mesurement's rho 
            and phi from polar coordinates to cartesian (x,y) coordinates like so:
            
            rho = px;
            phi = py;
            
            px = rho * sin(phi);  
            py = rho * cos(phi);

    d. Write to `outfile` (e.g., *build/output.txt*) the ground truth values for `(px, py, vx, vy)^T`. We will use these 
       ground truth values to compute the *RMSE* to determine how well our Extended Kalman Filter algorithm is doing in 
       terms of tracking our pedestrian at each LIDAR or RADAR measurement at time `t`.

    e. Push `fusionEKF.ekf.x_`, our 4D  VectorXd corresponding to our `(px, py, vx, vy)^T` estimation of position and 
       velocity at time `t`, on to our `estimations` stack (an instance of `vector<VectorXd>`).
       
    f. Push `gt_pack_list[k].gt_values_`, our 4D  VectorXd corresponding to the actual `(px, py, vx, vy)^T` position 
       and velocity of our pedestrian at time `t`, onto our `ground_truth` stack (an instance of `vector<VectorXd>`)

12. Compute the accuracy (*RMSE* - Root Mean Squared Error) of our estimations compared to ground truth by passing our 
    `estimations` and `ground_truth` `vector<VectorXd>`s to [Tools#CalculateRMSE](https://github.com/matthewzimmer/CarND-Extended-Kalman-Filter-Project/blob/7e0aa46c0508904f14e88757401b2cbd805ddf76/src/tools.cpp#L10-L41).

13. Close our `out_file_` and `in_file_` file streams.

14. Exit the program return 0 indicating success.

}

## Code Style

*I extracted the following from the open-source AirSim github repository because it is precisely how I think about Code 
Style.*



## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/12dd29d8-2755-4b1b-8e03-e8f16796bea8)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.
