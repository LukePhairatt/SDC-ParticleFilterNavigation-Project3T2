# **Particle Filter-Kidnapped Vehicle Project-3 Term-2**
![alt text][image0]

[//]: # (Image References)
[image0]: ./image/pf_sim.png "result"

## Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

The message from the simulator contains following information:


```sh
	Y
	^
        |                       
        |           y
        |           ^     
        |           |  [landmark] (sense_obs_x,sense_obs_y)
        |           |   /
        |           |  /
        |           | /    
        |           |/  
        |           ------------> x 
        |      a current robot position (sense_x,sense_y, sense_heading)
	|      (measurement given in the robot frame, measurement/observation is transformed to the global space)
        |
	------------> X
	Global coordibnate 

```

```sh
		# robot state
		time = time stamp
		sense_x = current x position 
		sense_y = current y position
		sense_theta = current heading position
		previous_velocity = (previous) velocity             (robot path control)
		previous_yawrate  = (previous) angular velocity     (robot path control)
		
		# observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
		sense_obs_x = landmark i x-position
		sense_obs_y = landmark i y-position
```

## **Particle Filter Summary**

**STEP 1- Init the particles:**  

In this step, we need to generate the new N instance particles with random Gaussian noise

```sh
	for i particles
		particles[i].id = i;
		particles[i].x = genGaussian.RandomNormalGaussian(x, std[0]);
		particles[i].y = genGaussian.RandomNormalGaussian(y, std[1]);
		particles[i].theta = genGaussian.RandomNormalGaussian(theta, std[2]);
		particles[i].weight = 1.0;

```



**STEP 2- Move the particles:**

_Motion Model_

```sh

		velxdt = velocity * delta_t;
		yawxdt = yaw_rate * delta_t;

		for each particle
			# no angular motion
			particles[i].x += velxdt * cos(particles[i].theta);
			particles[i].y += velxdt * sin(particles[i].theta);
		
			# with angular motion (yaw_rate not 0)
			theta_new = particles[i].theta + yawxdt;
			particles[i].x += velocity / yaw_rate * (sin(theta_new) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (-cos(theta_new) + cos(particles[i].theta));
			particles[i].theta = theta_new;

		

		// add noise around this predicted state
		particles[i].x = genGaussian.RandomNormalGaussian(particles[i].x, std_pos[0]);
		particles[i].y = genGaussian.RandomNormalGaussian(particles[i].y, std_pos[1]);
		particles[i].theta = genGaussian.RandomNormalGaussian(particles[i].theta, std_pos[2]);
	
```


**STEP 3- Compute the weight for each particle:**

We then compute the weight for each particle based on the actual measurements(sense_observations_x,sense_observations_y)  
and the global map ("../data/map_data.txt") for x,y landmark positions.

```sh
for each PARTICLE

	1- transform the given landmark measurement to the global space from this particle pose

		tx = observations[Obs].x;
		ty = observations[Obs].y;
		observations_world[Obs].x = tx*cos_heading - ty*sin_heading + px; // world coordinate
		observations_world[Obs].y = tx*sin_heading + ty*cos_heading + py; // world coordinate

	for each MEASUREMENT(in the global space)
		
		2- find the best match (Nearest Neighbour Search, see comments in particle_filter.cpp for further details)

			Minimum distance error between the given landmark measurement and the map
			(for all landmarks, find the nearest match measurement)


		3- compute corresponding likelihood/weight for all measurement

			sum_weights += diff_x * diff_x / var_x + diff_y * diff_y / var_y;

		   where
			var_x = std_landmark_x**2
			var_y = std_landmark_y**2                        
```	


**STEP 4- Resampling the particles:**  

Resampling wheel method 
```sh
		std::vector<Particle> new_particles;
		std::random_device rd;     
	    	std::mt19937 gen(rd()); 
		std::uniform_real_distribution<double> uni(0.0,1.0);     
	    	int index = int(uni(gen)*num_particles);
	    	double beta = 0.0;
	    	double mw = *std::max_element(weights.begin(), weights.end());
	    	for(int i=0;i<num_particles;++i){
		  beta += uni(gen) * 2.0 * mw;

		  while (beta > weights[index]){
		    beta -= weights[index];
		    index = fmod((index+1),num_particles);
		  }// end while	

		  new_particles.push_back(particles[index]);
		}// end for
	
		// new particles resampled
		particles = new_particles;

```

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

mkdir build  
cd build  
cmake ..  
make  
./particle_filter  

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"] 

["sense_y"] 

["sense_theta"] 

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"] 

["sense_observations_y"] 


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"] 

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label 

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass! 

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the specific values given by the simulation

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.


