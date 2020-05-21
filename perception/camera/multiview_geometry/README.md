# multi-view geometry

## stereo vision

- 3d reconstruction from multiple views
Assumption: K,T,R are known
Goal: recover the 3d structure from images

- structure from motion
Assumption: K,T,R are unknown
Goal: recover simultaneously 3d scene structure and camera pose(up to scale) from multiple images

### Triangulation
basic idea: given reconstruction as intersection of two rays
requires: camera pose(calibration),[point correspondence](https://github.com/wanghaichuan941221/robotics/tree/master/perception/camera/features)

- linear approximation
- non linear approximation([Gauss-Netwon](http://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm),[levenberg–marquardt](http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm))


### correspondence
- epipoles
- epipolar lines
- epipolar plane

### stereo rectification
- StereoBM 
- StereoSGBM
- StereoVar



### The 8-point algorithm

- monocular vision. In monocular vision, it is impossible to recover absolute scale of the scene and only 5 degree of freedom are measurable(3 parameter from rotation and 2 parameters from translation)

- for calibrated camera
4*n knowns(n correspondences and each one (u1,v1),(u2,v2)...)
5+3*n unknowns(5 for motion up a scale and 3*n number of coordinates of n 3d points)

Epipolar constrains P2^T * E * P1 = 0 (E = [T]x R)

- for uncalibrated camera

P2^T * F * P1 = 0


## Sequential SFM (visual odometry)

- initialze structure and motion from 2 views(bootstrapping)
- for each additional view
    - determine pose (localization)
    - extend structure, extract and triangulate new features (mapping)
    - refine structure and motion through Bundle Adjustment(BA) (optimization)
