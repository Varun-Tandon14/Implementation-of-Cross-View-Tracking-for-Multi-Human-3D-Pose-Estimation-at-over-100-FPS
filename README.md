# Implementation-of-the-paper-Cross-View-Tracking-for-Multi-Human-3D-Pose-Estimation-at-over-100-FPS
(Unofficial) Implementation of the paper "Cross-View Tracking for Multi-Human 3D Pose Estimation at over 100 FPS" by Chen et al.
[Paper link](https://arxiv.org/abs/2003.03972).
Before starting please visit the [original repo](https://github.com/longcw/crossview_3d_pose_tracking/tree/master). Wherever possible I try to follow the same structure as the authors. I will highlight the changes in the structure when required. Mostly the dependencies are the same but most notably we use matplotlib rather than vispy for 3D visualization. I have provided a list of dependencies below. Thanks to the authors for their excellent work.

## Get dataset:
Currently, the code is only tested for the Campus dataset. But since the values are not hard coded this code should ideally run without errors for other datasets as well. 
[Campus dataset info](https://www.epfl.ch/labs/cvlab/data/data-pom-index-php/) - [Onedrive download link](https://onedrive.live.com/?authkey=%21AKW9YCvYTyBLxL8&id=415F4E596E8C76DB%213351&cid=415F4E596E8C76DB)

Please download the dataset in Onedrive and extract the zip into a folder named Campus_Seq1. The dataset folder should therefore look like [this](https://github.com/longcw/crossview_3d_pose_tracking/tree/master#data-structure).

## Dependencies:
1. numpy
2. pandas
3. opencv
4. scipy
5. tqdm
6. matplotlib

## Implementation details:
1. Thanks to the author of the original repo for making the visualization and calibration code available. The graph partitioning problem solver was also provided by the author [here](https://gist.github.com/longcw/654a86ffe11122079040a7615c99a627#file-bip_solver-py-L9). Kudos.
2. I have extended the original camera.py and calibration.py to support my implementation. To make the code very easy to use I put everything from config, other helper functions, algorithms and visualization inside a single notebook. 
3. The original repo suggests using vispy but installation is sometimes complicated. I thought it will be more convenient to use matplotlib animation therefore we need not worry about vispy here.
4. At the end of the run of the algorithm, we save the details in the log file. Please see it to get a feel of the algorithm. 
5. The code runs below 100 FPS as it is severely unoptimized now. This code was meant to quickly implement the paper to the best of my ability. 
6. The authors provide the original IDs for the 2D key points (detections) and 3D poses (targets) in the files annotation_2d and annotation_3d respectively. In this current implementation, we only use the 2D pose keypoints only and not the IDs as provided in the annotation_2d (as they are preassigned and you could directly use them for triangulation).
7. After looking at the IDs in the annotation_3D we see that the authors probably implemented ReID to get respective results. I have not implemented ReiD since it was not mentioned in the algo. 1 in the paper.    

## Qualitative results (campus dataset):

### Screenshot for a single timestamp: 
Please see the tracking results for a single timestamp (at 41.72 sec) from 3 different angles below:
<div align="center">
  <img src="https://github.com/Varun-Tandon14/Implementation-of-the-paper-Cross-View-Tracking-for-Multi-Human-3D-Pose-Estimation-at-over-100-FPS/blob/main/images/timestamp_41_12_capture.jpg" alt="Android Registration" width="600" />
</div>
<div align="center">
  <img src="https://github.com/Varun-Tandon14/Implementation-of-the-paper-Cross-View-Tracking-for-Multi-Human-3D-Pose-Estimation-at-over-100-FPS/blob/main/images/timestamp_41_12_capture_angle_2.jpg" alt="Android Registration" width="600"/>
</div>
<div align="center">
  <img src="https://github.com/Varun-Tandon14/Implementation-of-the-paper-Cross-View-Tracking-for-Multi-Human-3D-Pose-Estimation-at-over-100-FPS/blob/main/images/timestamp_41_12_capture_angle_3.jpg" alt="Android Registration" width="600"/>
</div>

### Complete animation:

<div align="center">
  <video src="https://github.com/Varun-Tandon14/Implementation-of-Cross-View-Tracking-for-Multi-Human-3D-Pose-Estimation-at-over-100-FPS/assets/24519234/992a1aa6-2971-4793-b88b-3938d664d602" alt="Android Registration" width="600" />
</div>
    
## Additional Note: 

The repo is currently unlicensed as the license information is unclear to me in the original repo. I will update this repo when that becomes clear. I have no affiliation with AiFi Inc.
