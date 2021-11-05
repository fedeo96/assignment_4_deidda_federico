* Assignment #4 Particle filter localization:
    - Objectives: Localize the vehicle using the landmark information provided by sensors
    - Tasks evaluated (15 points):	
        + Particle filter working and localizing the vehicle during all the experiment (7 points) 
			* In the code you will find different TODOs. The main tasks are related to the inizialization, prediction, update, resampling and error computation of the algorithm.
			* Note: Please send me the source code of the best localization solution you are able to achieve.
        + Report describing the particle filter performance under different scenarios. Just one paragraph by scenario. Each paragraph should include a discussion of the trajectory estimated, error and execution time (8 points)
			* As we have seen during the lectures, the particle filter performance can be influenced by many factor (number of particles, error of the sensors or motion model...).
			* After executing the particle filter algorithm, a file called "res.txt" will be produced. This file will contain information regarding the estimation in X,Y coordinates of the best particle; the ground truth in X,Y coordinates; the root mean squared error in X and Y; and the execution time of your solution (from inizialization to resampling). All this information is used to compute the output that can be seen after executing "plotter.py"
			* The more the scenarios the higher the grade. Originality and quality of the report will be considered to determine grades (minimum number of scenarios 2, maximum 5)
			* Note: Please send me the plots produced by plotter.py, The report shall include the description of the scenario and the name of the plot that corresponds to that scenario.
			
* OS requirements:
    - To run the simulator and the particle filter please:
        + Execute all the instructions defined in the file install-ubuntu.sh (in the students_particle_filter folder) before compiling the program 
            ++ libuv1-dev libssl-dev gcc g++ cmake make and uWebSockets are needed so please follow and install are that libraries specified in the install-ubuntu.sh
        + Run the simulator
    
* Instructions to compile the code:
  
	mkdir build
	cd build
	cmake ..
	make
	./particle_filter 
	To generate the plot just write: "python3 plotter.py" (pay attention to the output file called res.txt with the performance in error and execution time of your implementation) 
	
* Important Note:
    - All the TODOs are in the file called "particle_filter.cpp" but you are more than free to modify any source file
    - If the simulator does not work for any strange reason, try to change the language of your linux dist. into English.

