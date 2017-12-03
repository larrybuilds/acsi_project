#!/usr/bin/env python
import numpy as np
# Optitrack
import socket
import optirx as rx
from optitrack.utils import get_ip_address, read_parameter
#import IPython
import struct as struct


class RigidBodiesPublisher(object):
  """
  Naming convention for the transformations:
    - wTo: Transformation between C{parent_frame} and C{optitrack_frame}
    - oTr: Transformation between C{optitrack_frame} and the rigid body
    - wTr: Transformation between C{parent_frame} and the rigid body
  """
  def __init__(self):
    # Read parameters to configure the node
    tf_publish_rate = read_parameter('~tf_publish_rate', 50.)
    tf_period = 1./tf_publish_rate if tf_publish_rate > 0 else float('inf')
    parent_frame = 'world''
    optitrack_frame = 'optitrack'
    rigid_bodies = read_parameter('~rigid_bodies', dict())
    names = []
    ids = []
    for name,value in rigid_bodies.items():
      names.append(name)
      ids.append(value)
    # Setup Publishers
    pose_pub = rospy.Publisher('/optitrack/rigid_bodies', RigidBodyArray, queue_size=3)
    point_pub = rospy.Publisher('/optitrack/upoints', UnidentifiedPointArray, queue_size=3)
    # Setup TF listener and broadcaster
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    # Connect to the optitrack system
    iface = read_parameter('~iface', 'eth1')
    version = (2, 7, 0, 0)  # the latest SDK version
    #IPython.embed()
    #optitrack = rx.mkdatasock(ip_address=get_ip_address(iface))
    #Modified by Nikhil
    # optitrack = rx.mkdatasock(ip_address=iface)
    optitrack = rx.mkcmdsock(ip_address=iface)
    msg = struct.pack("I", rx.NAT_PING)
    server_address = '192.168.1.205'
    result = optitrack.sendto(msg, (server_address, rx.PORT_COMMAND))


    rospy.loginfo('Successfully connected to optitrack')
    # Get transformation from the world to optitrack frame
    (parent, child) = (parent_frame, optitrack_frame)
    try:
      now = rospy.Time.now() + rospy.Duration(1.0)
      tf_listener.waitForTransform(parent, child, now, rospy.Duration(5.0))
      (pos,rot) = tf_listener.lookupTransform(parent, child, now)
      wTo = PoseConv.to_homo_mat(pos, rot)  # return 4x4 numpy mat
    except (tf.Exception, tf.LookupException, tf.ConnectivityException):
      rospy.logwarn('Failed to get transformation from %r to %r frame' % (parent, child))
      parent_frame = optitrack_frame
      wTo = np.eye(4)
    # Track up to 24 rigid bodies
    prevtime = np.ones(24)*rospy.get_time()
    # IPython.embed()
    while not rospy.is_shutdown():
      try:
        # data = optitrack.recv(rx.MAX_PACKETSIZE)
        data, address = optitrack.recvfrom(rx.MAX_PACKETSIZE + 4)
      except socket.error:
        if rospy.is_shutdown():  # exit gracefully
          return
        else:
          rospy.logwarn('Failed to receive packet from optitrack')
      msgtype, packet = rx.unpack(data, version=version)
      if type(packet) is rx.SenderData:
        version = packet.natnet_version
        rospy.loginfo('NatNet version received: ' + str(version))
      if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
        # Optitrack gives the position of the centroid.
        array_msg = RigidBodyArray()
        upoint_msg = UnidentifiedPointArray()
        # IPython.embed()
        if msgtype == rx.NAT_FRAMEOFDATA:
          # IPython.embed()
          for i, rigid_body in enumerate(packet.rigid_bodies):
            body_id = rigid_body.id
            pos_opt = np.array(rigid_body.position)
            rot_opt = np.array(rigid_body.orientation)
            # IPython.embed()
            oTr = PoseConv.to_homo_mat(pos_opt, rot_opt)
            # Transformation between world frame and the rigid body
            WTr = np.dot(wTo, oTr)
            wTW = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
            wTr = np.dot(wTW,WTr)

            ## New Change ##
            # Toffset = np.array( [[0, 1, 0, 0],[0, 0, 1, 0],[1, 0, 0, 0],[0, 0, 0, 1]] )
            # tf_wTr = np.dot(oTr,Toffset)
            # tf_pose = Pose()
            # tf_pose.position = Point(*tf_wTr[:3,3])
            # tf_pose.orientation = Quaternion(*tr.quaternion_from_matrix(tf_wTr))

            # IPython.embed()

            array_msg.header.stamp = rospy.Time.now()
            array_msg.header.frame_id = parent_frame
            body_msg = RigidBody()
            pose = Pose()
            pose.position = Point(*wTr[:3,3])
            # OTr = np.dot(oTr,Toffset)
            # pose.orientation = Quaternion(*tr.quaternion_from_matrix(oTr))

            # change 26 Feb. 2017
            # get the euler angle we want then compute the new quaternion
            euler = tr.euler_from_quaternion(rot_opt)
            quaternion = tr.quaternion_from_euler(euler[2],euler[0],euler[1])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]


            body_msg.id = body_id
            body_msg.tracking_valid = rigid_body.tracking_valid
            body_msg.mrk_mean_error = rigid_body.mrk_mean_error
            body_msg.pose = pose
            for marker in rigid_body.markers:
            # TODO: Should transform the markers as well
              body_msg.markers.append(Point(*marker))
            array_msg.bodies.append(body_msg)
            # Control the publish rate for the TF broadcaster
            if rigid_body.tracking_valid and (rospy.get_time()-prevtime[body_id] >= tf_period):
              body_name = 'rigid_body_%d' % (body_id)
              if body_id in ids:
                idx = ids.index(body_id)
                body_name = names[idx]
              ## no change ##
              # tf_broadcaster.sendTransform(pos_opt, rot_opt, rospy.Time.now(), body_name, optitrack_frame)
              # change 1 ##
              # pos2 = np.array([tf_pose.position.x, tf_pose.position.y, tf_pose.position.z])
              # rot2 = np.array([tf_pose.orientation.x, tf_pose.orientation.y, tf_pose.orientation.z, tf_pose.orientation.w])
              # tf_broadcaster.sendTransform(pos2, rot2, rospy.Time.now(), body_name, optitrack_frame)
              ## change 2 ## <fail>
              # pos2 = np.array([-pose.position.y, pose.position.x, pose.position.z])
              # rot2 = np.array([-pose.orientation.y, pose.orientation.x, pose.orientation.z, pose.orientation.w])
              # tf_broadcaster.sendTransform(pos2, rot2, rospy.Time.now(), body_name, optitrack_frame)
              ## change 3 ## <fail>
              # pos2 = np.array([-pos_opt[1],pos_opt[0],pos_opt[2]])
              # rot2 = np.array([-rot_opt[1],rot_opt[0],rot_opt[2],rot_opt[3]])
              # tf_broadcaster.sendTransform(pos2, rot2, rospy.Time.now(), body_name, optitrack_frame)
              ## change 4 ## <>
              pos2 = np.array([pose.position.x, pose.position.y, pose.position.z])
              rot2 = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w])
              tf_broadcaster.sendTransform(pos2, rot2, rospy.Time.now(), body_name, parent_frame)

              prevtime[body_id] = rospy.get_time()

	  # Extract all unidentified markers from data frame
          for marker in packet.other_markers:
	      # Append each marker to the array
              upoint_msg.upoints.append(Point(*marker))
	  # Fill in timestamp
          upoint_msg.header.stamp = rospy.Time.now()

        pose_pub.publish(array_msg)
        point_pub.publish(upoint_msg)

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  opti_node = RigidBodiesPublisher()
  rospy.loginfo('Shuting down [%s] node' % node_name)
