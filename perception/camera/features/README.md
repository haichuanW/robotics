# image features

## image detector

### Harris corner
Corners are the important features in the image, and they are generally termed as interest points which are invariant to translation, rotation, and illumination. (flat region,edge,corner have different lambda values)

Problem(not scale invariant)

### FAST (Features from accelerated segment test)

- select a pixel [p](fast.png) in the image which can be identified as an insterest point or not 
- select approxipriate threshold value(it is 10 by default in opencv)
- consider a circle of 16 pixels around the pixel under test(Bresenham circle of radius 3)
- check whether at least n pixels are greater than p+threshold or less than p-threshold (n=12)
- repeat all procedure for all pixel

Problem(not scale invariant or rotation invariant)

### ORB (Oriented FAST and Rotated BRIEF)

- Scale invariant([Pyramid](orb.png))
- Rotation invariant(intensity centroid)

- Brief(Binary robust independent elementary feature)

### AKAZE

### SIFT(scale invariant feature transform)

- Detect scale-space extrema in difference-of-Gaussians pyramid
- Supress responses along edges



## image descriptor

### binary descriptor
A binary descriptor is made of three parts:
- A sampling pattern: where to sample points in the region around the descriptor.
- Orientation compensation: some mechanism to measure the orientation of the keypoint and rotate it to compensate for rotation changes.
- Sampling pairs: which pairs to compare when building the final descriptor.

|        | Sampling pattern             | Orientation calculation | Sampling pairs    
----------------------------------------------------------------------------------------
| BRIEF  |       None                   |           None          |       Random
----------------------------------------------------------------------------------------
| ORB    |       None                   |           None          |    Learned pairs
----------------------------------------------------------------------------------------
|        | Concentric circles with      | comparing gradients     |    Only short pairs
| BRISK  | more points on outer rings   | of long pairs           |
----------------------------------------------------------------------------------------
|        | Overlapping Concentric       |Comparing gradients of   |
| FREAK  | circles with more points     |preselected 45 pairs     |     Learned pairs
|        | on inner rings               |                         |
----------------------------------------------------------------------------------------


### HOG(Histogram of oriented Gradients)

(HOG descriptor for SIFT)[https://gilscvblog.com/2013/08/18/a-short-introduction-to-descriptors/]

## References
- Shaharyar Ahmed Khan Tareen,etc. "A Comparative Analysis of SIFT, SURF, KAZE,
    AKAZE, ORB, and BRISK" [CoMET 2018](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8346440).
