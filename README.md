Tango output to rosbag files 
=================
This package contains scripts and nodes to convert output files created by Tango recordings to rosbag files with several messages.
The python script 'parse_tango_output.py' executes all scripts and nodes sequentially.
The usage of 'parse_tango_output.py' is:
        
        parse_tango_output.py /location/to/files outputbag.bag

'/location/to/files' must be of the kind produced by Tango mapper. The superframes subfolder should be there, as well.
The script produces **4** rosbag files:
  * '/tmp/tango_poses.bag',
  * '/tmp/tango_tfs.bag',
  * '/tmp/tango_superframes.bag',
  * and the ...
