# Target for a single stepping motion of a humanoid's leg

geometry_msgs/Pose2D pose   # step pose as relative offset to last leg
uint8 leg                   # which leg to use (left/right, see below)

uint8 RIGHT=0               # right leg constant , diubah jadi kapital
uint8 LEFT=1                # left leg constant     , ini juga