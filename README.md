# <center> Smart Markers

---

This project allows creating a **Smart Markers** map (offline) and using it in the camera localization (online). The code is the implementation of the paper: "**Smart Artificial Markers for Accurate Visual Mapping and Localization**." More information __[here](https://www.mdpi.com/1424-8220/21/2/625)__.

<img src="/images/camera_localization.gif" alt="camera_localization" width="1000" class="center"/>

## Installation:
---

Dependences (mandatory):

+ **Eigen3**
+ **OpenCV**: tested with version 3.4.1.
+ **Aruco**: tested with version 3.0.12.
+ **Marker Mapper**: tested with version 1.0.12.
+ **g2o**
+ **Cholmod**
+ **ZED API**: tested with version 2.3 (CUDA 10.0).

Download code and open a terminal `ctrl+t`:

    $ cd path 
    $ mkdir build & cd build 
    $ cmake .. 
    $ make

## How to use:
---

+ Make a set of **Smart Markers**.
    * Use the **Aruco** library to create a set of markers.
    * Print the markers and use a tape measurer to know their actual size. 
    * Build the **PMS** units and attach them to the markers.
    * <img src="/images/schematic.png" alt="schematic" width="300" class="center"/>
    * <img src="/images/marker_front.png" alt="marker_front" width="150" class="center"/> <img src="/images/marker_back.png" alt="marker_back" width="150" class="center"/>
    * Load code in folder `/pms` into an ESP32 device using Arduino IDE.
    * Generate `pms_data.txt`.
+ Place **Smart Markers** in the environment.
+ Record a video `.svo` with calibated __[Stereolabs ZED](https://www.stereolabs.com/zed/)__.

+ Run `./test_sm_mapping` and get the 3D accurate location of the markers:
    + Inputs:
        * `<video.svo>`: video for create **Smart Markers** map.
        * `<pms_data.txt>`: markers poses get by the **PMS** units.
        * `<aruco_config.yml>`: aruco parameters.
        * `<out_dir>`: path of the directory to save all outputs.
        * `<marker_size>`: real marker size mesuare by hand.
        * `<marker_ref>`: reference marker (origin of the markers map reference system).
        Note:
        + `0.1295`: marker size measure by hand.
        + `000000`: reference marker (origin of the markers map reference system).
    + Outputs:
        * `/depth`: contains depth images (in mm). A depth map is a 1-channel matrix with 16-bit float values for each pixel.
        * Example of how access to depth data:
        ```
        //Print the depth value at the center of the image:
        std::cout << depth_image.at<float>(depth_image.rows/2, depth_image.cols/2) << std::endl;
        ```
        * `depth.txt`: contains the index of the depth images, e.g., `timestamp depth/timestamp.png.`
        * `/rgb`: color images (left camera).
        * `rgb.txt`: contains the index of the color images, e.g., `timestamp rgb/timestamp.png.`
        * `/sm_data/sm_cam.yml`: contains the camera's intrinsic parameters.
        * `/sm_data/sm_map.yml`: contains the 3D id and position (in the marker reference system) of the four corners of each marker.
    + Example:
        ```
        ./test_sm_mapping <video.svo> <pms_data.txt> <aruco_config.yml> </Desktop> <0.1295> <00000>"
        ```

+ Use the map generated `sm_map.yml` with `./test_cam_localization` for localizing your camera:

    + Inputs:
        * `<video.svo>`: video for camera localization
        * `<sm_map.yml>`: contains the 3D id and position (in the markers map reference system) of the four corners of each marker
        * `<aruco_config.yml>`: aruco parameters 
        * `<out_dir>`: the path of the directory to save all outputs
    + Output:
        * `groundtruth.txt`: contains the timestamp and pose of the camera in the coordinate system of the markers map, e.g.:
        `#timestamp tx ty tz qx qy qz qw.` On this ground, truth are only optimized poses (i.e., only in frames where two or more markers), not for all frames.
    + Example:    
        ```
        ./test_cam_localization <video.svo> <sm_map.yml> <aruco_config.yml> </Desktop>
        ```

## **Note:**
---

+ All markers must have the same size. 
+ Make sure several markers appear in the images to create the marker graph. 

## References:
---

Please cite the following papers if use this code and **Smart Markers**:
```
@ARTICLE{Ortiz2021,
    AUTHOR  = {Ortiz-Fernandez, Luis E. and Cabrera-Avila, Elizabeth V. and Silva, Bruno M. F. da and Gonçalves, Luiz M. G.},
    TITLE   = {Smart Artificial Markers for Accurate Visual Mapping and Localization},
    JOURNAL = {Sensors},
    VOLUME  = {21},
    YEAR    = {2021},
    NUMBER  = {2},
    URL     = {https://www.mdpi.com/1424-8220/21/2/625},
    ISSN    = {1424-8220},
    DOI     = {10.3390/s21020625}
}
```   
```
@ARTICLE{Ortiz2018,
    TITLE   = {Depth data error modeling of the ZED 3D vision sensor from stereolabs},
    AUTHOR  = {Ortiz, Luis Enrique and Cabrera, Elizabeth V and Gon{\c{c}}alves, Luiz M},
    JOURNAL = {ELCVIA: electronic letters on computer vision and image analysis},
    VOLUME  = {17},
    NUMBER  = {1},
    PAGES   = {0001--15},
    YEAR    = {2018},
    DOI     = {https://doi.org/10.5565/rev/elcvia.1084},
    eISSN   = {1577-5097} 
}
```

## NOTE:

<p style="background:black">
<code style="background:black;color:red">If you find any of these codes helpful, please share my <a href="https://github.com/LuisOrtizF">GitHub</a> and STAR</code>:star:<code style="background:black;color:red">this repository to help other enthusiasts to find these tools. Remember, the knowledge must be shared. Otherwise, it is useless and lost in time.
</code>
</p>
