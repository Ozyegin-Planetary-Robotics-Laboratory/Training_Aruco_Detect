# Code Explanations

## 1- Integrated Aruco Detection Part

### ARUCO_DICT
    A dictionary stores markers' types.

### aruco_display
    Displays detected ArUco markers on the image with some new additions. Green lines between the corners, red dot on the center of the marker and marker id on the top left corner.

### ArucoDetectionNode
    Creating form of the ROS 2 node for ArUco marker detection.

*   #### __init__
    Setting the ArUco type, loading the dictionary and parameters, creating publishers and starting the video capturing via camera.

*   #### process_frame
    Process a single frame for ArUco marker detection. It detects the ArUco markers and checks if the marker ID has already been processed. If not, it publishes the 2D points of corners and the marker id.
    > I made it like this in order to make output more readable. Codes can be changed under any request of receiving continuous feedbacks about marker.

*   #### run
    Run the detection loop.

### main
    Main function to initialize and run the node.

<br />

## 2- Listener Node

### ListenerNode
    Creating form of the ROS 2 node for the listener.

*   #### __init__
    Creating subscription for the topics of detected_marker_id and detected_marker_corners.
*   #### id_callback
    Callback for received marker IDs. It writes received marker id.
*   #### corners_callback
    Callback for received marker corners. It writes how many corners reccieved and the positions of each corner.

### main
    Main function to initialize and run the node.

<br />

# How To Run

### Preparing The ArUco Type
*   From the ARUCO_DICT part, select the type of ArUco will be used.
*   Copy that name and paste it to self.aruco_type variable, which can be found in the __init__ part.

### Preparing Files
*   In setup.py, go to entry_points part. Between the brackets of console_scripts, write this 2 files in this format:
    *   "[the name you want] = [package name].[the name used in super().init]:main,"
    > !Do not use "[" and "]".
*   Go to package.xml and set up the dependencies. 

### Preparing Terminals
*   After completion of these steps, open 2 terminal and go to workshop where these files created.
*  Build them with the line "colcon build" and after that wrote the line "source install/setup.bash".
*   After that, in one terminal, open the aruco detection file by writing the line "ros2 run [package name] [the name you want (in setup.py)]" 
    > Again do not use "[" and "]".
*   And, open the listener node from the other terminal.

<br />

# The Output
*   integrated_aruco_detection.py recognizes the markers and prepares its id to publish. 
*   And from the camera, it calculates 2D corner positions of the marker in the image plane.
*   Then, it publish ids in Int32 format and corner positions in PoseArray format.
*   Our listener_node is subscriber of their topics. It listens and gathers informations.
*   With these informations, it writes marker id, how many corners captured and the positions of the corners into the terminal. 




