import math

def normalize_rad(rad):
    # Normalize the angle to be within the range of -pi to +pi
    return rad % (2 * math.pi) - math.pi

def normalize_deg(deg):
    # Normalize the angle in degrees to be within the range of 0 to 360
    return deg % 360

def mps2kph(mps):
    return mps * 3.6

def kph2mps(kph):
    return kph / 3.6

def rad2deg(rad):
    return normalize_deg(math.degrees(rad))

def deg2rad(deg):
    return normalize_rad(math.radians(deg))

