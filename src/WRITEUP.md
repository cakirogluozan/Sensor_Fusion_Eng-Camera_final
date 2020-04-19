## Sensor Fusion Engineering Nanodegree

### Camera Final Project

#### Milestones

* FP.1 Match 3D Objects
        
        FinalProject_Camera.cpp  line:225
        camFusion_Student.cpp    line:272-317

    Extraction of the optimum bounding box proposal (by yolo) amongst others by utilizing matched keypoints in the image. The bounding box that contains the most number of keypoints is selected as the optimum bounding box.

* FP.2 Compute Lidar-based TTC

        FinalProject_Camera.cpp  line:263
        camFusion_Student.cpp    line:208-269
    
    Lidar points of the optimum bounding box (extracted in FP.1) is filtered by their y dimensional values. If the point is not in lane width range, it is excluded.
    
* FP.3 Associate Keypoint Correspondences with Bounding Boxes
        
        FinalProject_Camera.cpp  line:267
        camFusion_Student.cpp    line:133-150

    If region of interest contains the keypoint and matched keypoint then the corresponding keypoints are added to bounding box keypoints and matched keypoints.

* FP.4 Compute Camera-based TTC   
         
        FinalProject_Camera.cpp  line:271
        camFusion_Student.cpp    line:154-215

    Distance between the previous and current keypoint is evaluated and and distance ratio is evaluated.

* FP.5-6 Performance Evaluation 1-2

    The log file of the selected 3 detector-descriptor combination can be found in log/ directory.
        
        0-0_LOG.txt - ["SHITOMASI" - "BRISK"]
        2-5_LOG.txt - ["FAST" - "SIFT"]
        5-1_LOG.txt - ["AKAZE" - "BRIEF"]
    
    Moreover their statistics is visualized on a jupyter notebook. 

  