# kinova_motion_subscriber.py is the the node connecting upper level algorithm and kinova Jaco.
# It subscribe a topic which gives a unified position feed in 6 dimensions, each dof (-1,1)
# this node will transform such position topic into the velocity of Jaco and give the velocity control topic to Jaco.
# this node will collect the following data:
#    action
#    cmd
#    torque
#    pose
#    orientation
#    joint_angle
#    joint_velocity
# and transform into np.ndarray
# this node is to run 500 rollouts, each rollout with length of 50
