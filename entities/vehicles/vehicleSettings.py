#!/usr/bin/python

nbStandardCars = 10
standardCarLength = 4.5
standardCarWidth  = 2.
standardCarHeight = 0. 

# set parameters for car-car repulsive forces from [Anvari et al.]
v0              = 10.0        # Average desired speed [m/s]
tauAlpha        = 0.5         # Relaxation time [s]
BAlphaU         = 0.3         # Potential damping parameter [m]
amplitudeAlphaU = 5.         # Car-car repulsive force amplitude [m^-1 s^-2]

carStopAcceptance  = 0.5    # Acceptance to stop at a destination point