^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointcloud_to_laserscan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2017-04-26)
------------------
* Merge pull request `#4 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/4>`_ from yoshimalucky/fix-miscalculation-in-angle-increment
  Fixed miscalculation in angle_increment in the launch files.
* fixed miscalculation in angle_increment in the launchfiles.
* Contributors: Paul Bovbel, yoshimalucky

1.3.0 (2015-06-09)
------------------
* Fix pointcloud to laserscan transform tolerance issues
* Move pointcloud_to_laserscan to new repository
* Contributors: Paul Bovbel

1.2.7 (2015-06-08)
------------------

* Cleanup pointcloud_to_laserscan launch files
* Contributors: Paul Bovbel

1.2.6 (2015-02-04)
------------------
* Fix default value for concurrency
* Fix multithreaded lazy pub sub
* Contributors: Paul Bovbel

1.2.5 (2015-01-20)
------------------
* Switch to tf_sensor_msgs for transform
* Set parameters in sample launch files to default
* Add tolerance parameter
* Contributors: Paul Bovbel

1.2.4 (2015-01-15)
------------------
* Remove stray dependencies
* Refactor with tf2 and message filters
* Remove roslaunch check
* Fix regressions
* Refactor to allow debug messages from node and nodelet
* Contributors: Paul Bovbel

1.2.3 (2015-01-10)
------------------
* add launch tests
* refactor naming and fix nodelet export
* set default target frame to empty
* clean up package.xml
* Contributors: Paul Bovbel

1.2.2 (2014-10-25)
------------------
* clean up package.xml
* Fix header reference
* Fix flow
* Fix pointer assertion
* Finalize pointcloud to laserscan
* Initial pointcloud to laserscan commit
* Contributors: Paul Bovbel
