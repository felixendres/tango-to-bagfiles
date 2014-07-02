Tango output to rosbag files 
=================
This package contains scripts and nodes to convert output files created by Tango recordings to rosbag files with several messages.
The python script 'combined_parse_tango_output.py' executes all scripts and nodes sequentially.
The usage of 'parse_tango_output.py' is:
        
        combined_parse_tango_output.py /location/to/files outputbag.bag

'/location/to/files' must be of the kind produced by Tango mapper. The superframes subfolder should be there, as well. The file outputbag.bag will have all the data. 

Please note that:

 - The images from the narrow field camera are black and white only
 - The messages in the bag file is not ordered. Sorting can be done afterfward.
 - This code requires the file superframe_v2.h from the Google Tango SDK.  Due to licencing issues we cannot redistribute this file. It should be copied in the include/superframe_parser folder.
