From [3dv_tutorial](https://github.com/mint-lab/3dv_tutorial)

### Learning status
[ ] Object localization and measurement 

### Table of content (learning)
* __Single-view Geometry__
  * Camera Projection Model
    * Object Localization and Measurement: [object_localization.cpp][] (result: [image](https://drive.google.com/open?id=10Lche-1HHazDeohXEQK443ruDTAmIO4E))
    * Image Formation: [image_formation.cpp][] (result: [image0](https://drive.google.com/file/d/0B_iOV9kV0whLY2luc05jZGlkZ2s/view), [image1](https://drive.google.com/file/d/0B_iOV9kV0whLS3M4S09ZZHpjTkU/view), [image2](https://drive.google.com/file/d/0B_iOV9kV0whLV2dLZHd0MmVkd28/view), [image3](https://drive.google.com/file/d/0B_iOV9kV0whLS1ZBR25WekpMYjA/view), [image4](https://drive.google.com/file/d/0B_iOV9kV0whLYVB0dm9Fc0dvRzQ/view))
    * Geometric Distortion Correction: [distortion_correction.cpp][] (result: [video](https://www.youtube.com/watch?v=HKetupWh4V8))
  * General 2D-3D Geometry
    * Camera Calibration: [camera_calibration.cpp][] (result: [text](https://drive.google.com/file/d/0B_iOV9kV0whLZ0pDbWdXNWRrZ00/view))
    * Camera Pose Estimation (Chessboard): [pose_estimation_chessboard.cpp][] (result: [video](https://www.youtube.com/watch?v=4nA1OQGL-ig))
    * Camera Pose Estimation (Book): [pose_estimation_book1.cpp][]
    * Camera Pose Estimation and Calibration: [pose_estimation_book2.cpp][]
    * Camera Pose Estimation and Calibration w/o Initially Given Camera Parameters: [pose_estimation_book3.cpp][] (result: [video](https://www.youtube.com/watch?v=GYp4h0yyB3Y))
* __Two-view Geometry__
  * Planar 2D-2D Geometry (Projective Geometry)
    * Perspective Distortion Correction: [perspective_correction.cpp][] (result: [original](https://drive.google.com/file/d/0B_iOV9kV0whLVlFpeFBzYWVadlk/view), [rectified](https://drive.google.com/file/d/0B_iOV9kV0whLMi1UTjN5QXhnWFk/view))
    * Planar Image Stitching: [image_stitching.cpp][] (result: [image](https://drive.google.com/file/d/0B_iOV9kV0whLOEQzVmhGUGVEaW8/view))
    * 2D Video Stabilization: [video_stabilization.cpp][] (result: [video](https://www.youtube.com/watch?v=be_dzYicEzI))
  * General 2D-2D Geometry (Epipolar Geometry)
    * Visual Odometry (Monocular, Epipolar Version): [vo_epipolar.cpp][]
    * Triangulation (Two-view Reconstruction): [triangulation.cpp][]
* __Multi-view Geometry__
  * Bundle Adjustment
    * Global Version: [bundle_adjustment_global.cpp][]
    * Incremental Version: [bundle_adjustment_inc.cpp][]
  * Structure-from-Motion
    * Global SfM: [sfm_global.cpp][]
    * Incremental SfM: [sfm_inc.cpp][]
  * Feature-based Visual Odometry and SLAM
    * Visual Odometry (Monocular, Epipolar Version): [vo_epipolar.cpp][]
    * Visual Odometry (Stereo Version)
    * Visual Odometry (Monocular, PnP and BA Version)
    * Visual SLAM (Monocular Version)
  * Direct Visual Odometry and SLAM
    * Visual Odometry (Monocular, Direct Version)
  * c.f. The above examples need [Ceres Solver][] for bundle adjustment.
* __Correspondence Problem__
  * Line Fitting with RANSAC: [line_fitting_ransac.cpp][]
  * Line Fitting with M-estimators: [line_fitting_m_est.cpp][]
* **Appendix**
  * Line Fitting
  * Planar Homograph Estimation
  * Fundamental Matrix Estimation

