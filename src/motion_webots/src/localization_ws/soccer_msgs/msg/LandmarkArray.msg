# A frame's worth of projected landmark detections.
# header.stamp MUST be the CAPTURE stamp of the source image (not the publish
# time) so downstream fusion can look up the camera pose at the instant the
# frame was taken — critical while the neck is moving under active vision.
# header.frame_id = base_link (p_base frame).
std_msgs/Header header
Landmark[] landmarks
