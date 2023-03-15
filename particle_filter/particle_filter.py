from grid import CozGrid
from particle import Particle
from utils import add_gaussian_noise, add_marker_measurement_noise, grid_distance, rotate_point, diff_heading_deg, add_odometry_noise
from setting import *
import math
import numpy as np

#Author: Kelly Qiu and Ausaf Ahmed

def motion_update(particles, odom, grid):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*
        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used for boundary checking
        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
    dx, dy, dh = odom[0], odom[1], odom[2]
    for particle in particles:
            dx, dy = rotate_point(dx, dy, particle.h)
            x, y, h = add_odometry_noise((particle.x + dx, particle.y + dy, particle.h + dh), ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
            motion_particles.append(Particle(x, y, h))
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    """Finding weights"""
    if len(particles) == 0 or len(measured_marker_list) == 0:
            return particles
    weights = []
    for particle in particles:
            prob = 1.0
            weight = 0
            if grid.is_free(particle.x, particle.y):
                particleMarkers = particle.read_markers(grid)
                for measuredMarker in measured_marker_list:
                        max = 0
                        match = False
                        measuredMarker = add_marker_measurement_noise(measuredMarker, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA)
                        for particleMarker in particleMarkers:
                                a = diff_heading_deg(measuredMarker[2], particleMarker[2])
                                d = grid_distance(measuredMarker[0], measuredMarker[1], particleMarker[0], particleMarker[1])
                                newProb = math.exp(-((d**2/(2*MARKER_TRANS_SIGMA**2))+(a**2/(2*MARKER_ROT_SIGMA**2))))
                                if newProb > max:
                                        max = newProb
                                        match = True
                        if match:
                                prob*=max
                                weight = prob
            weights.append(weight)
    if sum(weights) != 0:
        weights = weights/np.sum(weights)
    else:
        weights = [1/len(weights)] * len(weights)
    """Resampling"""
    measured_particles = np.random.choice(particles, len(particles) - 250, True, p = weights).tolist()+ Particle.create_random(count=250, grid=grid)
    return measured_particles


