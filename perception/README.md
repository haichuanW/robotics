# Perception with pytorch 

## ssd (single shot MultiBox detector)
- Single Shot: this means that the tasks of object localization and classification are done in a single forward pass of the network.
- MultiBox: this is the name of a technique for bounding box regression developed by Szegedy et al. 
- Detector: The network is an object detector that also classifies those detected objects.

The [SSD](https://github.com/wanghaichuan941221/python/tree/master/deepLearning/torch/ssd.pytorch) use transfer learning to detect traffic sign and it is almost real-time in NVIDIA P1000 GPU.

## Faster RCNN

In Faster R-CNN, Selective search is replaced by RPN using CNN. And this CNN is shared with detection network.

The [Faster RCNN](https://github.com/wanghaichuan941221/python/tree/master/deepLearning/torch/faster_rcnn) use transfer learning to detect traffic sign. It is quite slow in NVIDIA P1000 GPU,but it can detect quite small object.