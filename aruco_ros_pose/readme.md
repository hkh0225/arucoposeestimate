# 1. TODO

## 1.1 class_detect_markers_node.cpp

- Change the "double MarkerSize" (if another MarkerSize is used) (line 75)
- Change the "cv::String video" to your path and the video you want to play (line 81)
- Change the "std::string save_path" to the path you want to save the data (leave empty if you don't want save the data; line 82) + Change File_names at the end of the script (line 475ff.)
- Change dictionary if another marker-dictionary is used (line 121)

## 1.2 inner_corners.cpp

- Set correct BitSize (line 53)
- Set correct marker_rotation (line 55) (in my video the marker_rotation are given in the file_name (20 for -20; m20 for 20; m40 for 40; m60 for 60)

## 1.2 detector_params.yaml

- Set InnerCorners to 1 to calculate the inner corners of a marker

<br/><br/>
# 2. Possible improvements

## 2.1 inner_corners.cpp

- Vary the number of corners you want to detect (line 80)
- Vary the minimal distance between two corners (the number in line 163) -> should be larger when distance between marker and camera is short, and smaller when the distance is far
- Vary the ROI of the corner (line 155)
- For further use (use in in car park) adjust the code in lne 237-239 (explanation is in the code)


