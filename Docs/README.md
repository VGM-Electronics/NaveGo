# NaveGo

[![Releases](https://img.shields.io/badge/release-v1.0-green.svg?style=plastic)](https://github.com/rodralez/NaveGo/releases) [![DOI](https://zenodo.org/badge/12745155.svg)](https://zenodo.org/badge/latestdoi/12745155)

NaveGo: an open-source MATLAB/GNU-Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis.

NaveGo is an open-source framework for processing INS/GPS sensors that is freely available online. It is developed under MATLAB/GNU-Octave due to this programming language has become a *de facto* standard for simulation and mathematical computing. NaveGo has been verified by processing real-world data from a real trajectory and contrasting results with a commercial, closed-source software package. Difference between both solutions have shown to be negligible. For more information read (Gonzalez et al., 2017).

NaveGo is supported at the moment by three academic research groups: GridTics at the National University of Technology (Argentina), ITIC at the National University of Cuyo (Argentina), and DIATI at the Politecnico di Torino (Italy). 

## Features

Main features of NaveGo are:

* Processing of an inertial navigation system (INS).

* Processing of a loosely-coupled integrated navigation system (INS/GPS).

* Simulation of inertial sensors and GPS.

* Zero Velocity Update (ZUPT) detection algorithm (in an early stage).

* Implementation of the Allan variance procedure to characterize inertial sensors' typical errors.

## How to cite this work

Rodrigo Gonzalez, Carlos Catania, and Paolo Dabove (2017). NaveGo: an open-source MATLAB/GNU-Octave toolbox for processing integrated navigation systems and performing inertial sensors profiling analysis. DOI: 10.5281/zenodo.841872. URL: https://github.com/rodralez/NaveGo/.

## Contributions

We are looking for contributors for NaveGo! Since integrated navigation is a topic used in several fields as Geomatics, Geology, Mobile Mapping, Autonomous Driving, even Veterinary (yes, Veterinary!), we hope other communities than the navigation community compromise and contribute with this open-source project.

You can contribute in many ways: 

* Writing code. 
* Writing a manual. 
* Reporting bugs.
* Suggesting new features.

If you are interested, please feel free to contact Dr. Rodrigo Gonzalez at rodralez [at] frm [dot] utn [dot] edu [dot] ar.

## Publications

The underlying mathematical model of NaveGo is based on two articles which are recommended for reading: 

* (Gonzalez et al., 2015) R. Gonzalez, J.I. Giribet, and H.D. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. [Link](http://ceai.srait.ro/index.php?journal=ceai&page=article&op=view&path%5B%5D=2478).

* (Gonzalez et al., 2015a) R. Gonzalez, J.I. Giribet, and H.D. Patiño. An approach to benchmarking of loosely coupled low-cost navigation systems. Mathematical and Computer Modelling of Dynamical Systems, vol. 21, issue 3, pp. 272-287, 2015. [Link](http://www.tandfonline.com/doi/abs/10.1080/13873954.2014.952642).

Other publications:

* (Gonzalez et al., 2017a) R. Gonzalez, E.M. Martinez, and P. Dabove. Assessment of Discrete Stochastic Models of MEMS Inertial Sensors by Using the Allan Variance. In the III International Conference on Sensors and Electronics Instrumentation Advances (SEIA' 2017), 20-22 September 2017, Moscow, Russia.

* (Gonzalez et al., 2017) R. Gonzalez, C.A. Catania, P. Dabove, J.C. Taffernaberry, and M. Piras. Model validation of an open-source framework for post-processing INS/GNSS systems. III International Conference on Geographical Information Systems Theory, Applications and Management (GISTAM 2017). Porto, Portugal. April 2017.


## Roadmap

Future features of NaveGo will be:

* RTS smoother.

* Tightly-coupled INS/GPS. 


## Acknowledgments

We would like to thank to many people that have contribute to make NaveGo a better tool:

* Dr. Juan Ignacio Giribet (Universidad Nacional de Buenos Aires, Argentina) for this continuous support on theory aspects of INS/GPS systems.

* Dr. Charles K. Toth (The Ohio State University, USA), Dr. Allison Kealy, and M.Sc. Azmir Hasnur-Rabiain (both from The University of Melbourne, Australia) for generously sharing IMU and GPS datasets, and in particular, for Azmir's unselfish help.

* Prof. Zhu, Dr. Yang, and Mr. Bo Sun, all from the Laboratory of Precision Measuring Technology and Instruments, Tianjin University, Tianjin, China, for contributing with IMU static measurements to test Allan variance routines.

* Dr. Paolo Dabove and Dr. Marco Piras (both from DIATI, Politecnico di Torino, Italy) for helping to debug NaveGo and suggesting new features.

# Examples

## Allan variance example

Just execute the file `navego_allan_example.m`. It process 2-hours of static measurements from an Sensonor STIM300 IMU.

## INS/GPS example

The file `navego_example.m` tries to demonstrate how NaveGo works. It compares the performances of two simulated IMUs, ADIS16405 IMU and ADIS16488 IMU, both integrated with a simulated GPS.

### References

* R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a simulation framework for low-cost integrated navigation systems, Journal of Control Engineering and Applied Informatics, vol. 17, issue 2, pp. 110-120, 2015. Eq. 26.

* Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B. 
http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf

* Analog Devices. ADIS16488 datasheet. Tactical Grade Ten Degrees of Freedom Inertial Sensor. Rev. G. 
http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16488.pdf

* Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS. Revision D. October 2011. 
http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf
