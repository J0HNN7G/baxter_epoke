#!/usr/bin/env python


import numpy as np


# ------------------------------ Constants ------------------------------

# rectangular workspace configuration (in metres) for poking in world frame
WSPACE_CENTRE_X = 0.6 
WSPACE_CENTRE_Y = -0.4
WSPACE_WIDTH = 0.2
WSPACE_HEIGHT = 0.2 


# ------------------------------ Conversion Helpers ------------------------------

def quat2arr(quaternion):
    """Convert geometry_msgs.msg quaternion to numpy array"""
    return np.array([quaternion.x,
                     quaternion.y,
                     quaternion.z,
                     quaternion.w])


def scale2scale(value, oMin=-1.0, oMax=1.0, nMin=-1.0, nMax=1.0):
    """
    Convert linear scale (min/max) to another linear scale (min/max)

    value: value to be converted
    oMin: old minimum value
    oMax: old maximum value
    nMin: new minimum value
    nMax: new maximum value
    return: value mapped from old range to new range
    """
    oSpan = oMax - oMin
    nSpan = nMax - nMin
    result = ( ( value - oMin) / oSpan) * nSpan + nMin
    return result


def prop2norm(value):
    """Convert from [0,1] to [-1,1]"""
    return scale2scale(value, oMin = 0)


def norm2width(value):
    """
    Convert from [-1,1] to x-axis value (in metres) w.r.t baxter base 
    frame origin
    """
    return scale2scale(value, 
                       nMin = WSPACE_CENTRE_X - (WSPACE_WIDTH / 2), 
                       nMax = WSPACE_CENTRE_X + (WSPACE_WIDTH / 2)) 


def width2norm(value):
    """
    Convert from x-axis value (in metres) w.r.t baxter base frame origin 
    to [-1,1]
    """
    return scale2scale(value, 
                       oMin = WSPACE_CENTRE_X - (WSPACE_WIDTH / 2), 
                       oMax = WSPACE_CENTRE_X + (WSPACE_WIDTH / 2)) 


def norm2height(value):
    """
    Convert from [-1,1] to y-axis value (in metres) w.r.t baxter base 
    frame origin
    """
    return scale2scale(value,                       
                       nMin = WSPACE_CENTRE_Y - (WSPACE_HEIGHT / 2), 
                       nMax = WSPACE_CENTRE_Y + (WSPACE_HEIGHT / 2)) 


def height2norm(value):
    """
    Convert from y-axis value (in metres) w.r.t baxter base frame origin 
    to [-1,1]
    """
    return scale2scale(value,                       
                       oMin = WSPACE_CENTRE_Y - (WSPACE_HEIGHT / 2), 
                       oMax = WSPACE_CENTRE_Y + (WSPACE_HEIGHT / 2)) 


def norm2angle(value):
    """Convert from [-1,1] to [-pi,pi]"""
    return value * np.pi


def angle2norm(value):
    """Convert from [-pi,pi] to [-1,1]"""
    return scale2scale(value,                       
                       oMin = -np.pi, 
                       oMax = np.pi) 


def discPoke2norm(discPoke, action_splits):
    """Convert discrete poke based on number of actions splits to poke"""
    contPoke = discPoke.astype(float) / action_splits
    contPoke[:3] = prop2norm(contPoke[:3])
    return contPoke


# ------------------------------ Actions ------------------------------


def randomAction():
    """
    Agent returns random action

    return: [x,y,t,l]; x is poke centre x-axis value (in metres);
    y is poke centre y-axis value (in metres); t is angle of 
    poke (in radians) w.r.t to baxter base frame; l is length of 
    poke (in metres)
    """
    x = prop2norm(np.random.rand())
    y = prop2norm(np.random.rand())
    t = prop2norm(np.random.rand())
    l = np.random.rand()
    return [x, y, t, l]
