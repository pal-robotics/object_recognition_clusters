^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_recognition_clusters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2014-06-23)
------------------
* Adding minimum version of sensor_msgs package
* Commented out debugging TF publishing.
  Was giving a continuous warning looking like: Unable to transform object from frame 'object_frame' to planning frame '/odom' (Lookup would require extrapolation into the past.  Requested time 82.933000000 but the earliest data is at time 96.737000000, when looking up transform from frame [object_frame] to frame [odom])
* Moved get_param to __init__, to avoid creating zombie sockets with each call.
* Adding documentation to readme + image
* Added optional base frame to use to class init.
  Modified the return value of main clustering function to return pose message
* Migrated cluster_bounding_box_finder to hydro and set up test in node.
* Add package with node that tests recog message.
* Contributors: Bence Magyar, Sam Pfeiffer
