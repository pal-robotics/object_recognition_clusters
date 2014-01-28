object_recognition_clusters
===========================

Package to contain nodes and modules for publishing and processing clusters within the Object Recognition Kitchen.

<img align="right" src="https://raw.github.com/bmagyar/object_recognition_clusters/master/resource/reem_table_clusters_n_gazebo.png" />


###Modules

##Ecto

* io_clusters.PointCloudMsgAssembler that generates pointclouds based on a set of clusters
* ObjectClustersPublisher ecto plasm which connects to the tabletop pipeline with the PointCloudAssembler and publishes the

##Python

* object_recognition_clusters.ClusterBoundingBoxFinder which computes the pose of an object given it's point cluster using PCA

###Nodes

* clusters_to_pose that runs the clustering the python module, used mainly for testing and demonstration.

