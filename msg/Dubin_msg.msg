#message to represent a single dubins curve. It contains 3 int to represent type of curve in each segment. 3 pose to represent the for nexus states.

#seg type:L_SEG (0),S_SEG (1),R_SEG (2)
#int8 seg0_type
#int8 seg1_type
#int8 seg2_type
int8 type
#four critical point: start, end, intermediate point 1, intermediate point 2
#geometry_msgs/Vector3 start
#geometry_msgs/Vector3 pt_i1
#geometry_msgs/Vector3 pt_i2
#geometry_msgs/Vector3 end
Quad_msg start
Quad_msg pt_i1
Quad_msg pt_i2
Quad_msg end
