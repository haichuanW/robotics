# lidar segmentation with pcl
<img src="obstacle_detection/data/ObstacleDetectionFPS.gif" width="700" height="400" />


## Installation

```bash
$> sudo apt install libpcl-dev
```

## algorithm
    - ransac(random sample census): ground plane segmentation
    - down sampling filter: reduce compuation
    - Euclidean cluster: cluster object 