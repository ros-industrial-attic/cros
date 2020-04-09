This is the readme file for the cROS solution of MS Visual Studio.
------------------------------------------------------------------

This solution comprises the following projects:
- cros_library: it creates the cros.lib static library, which is used by cROS programs.
- ros_master_python: it creates a ROS master. Since one ROS master is required to have a ROS network working, this program (or other ROS master) must be run before executing any ROS program.
- sample_listener and sample_talker: they create sample_listener.exe and sample_talker.exe, which can be run together to try the operation of a topic publisher, a topic subscriber, a service provider and a service client.
- parameter_test: it checks the parameter subscription and parameter-value update. It must be used together with a ROS program able to update and check parameter values.
- performance_test: this program measures the communication performance of cROS.

Requirements
------------
This solution has been designed for Visual Studio 2017.
Windows 10 version 1703 or later ir required to set the socket options TCP_KEEPIDLE, TCP_KEEPINTVL and TCP_KEEPCNT.

Creation of new projects
------------------------
If you want to create a new (empty) project in the cros solution for building a new cROS program, the following project options must be set:

* In the new-project tree:
In references:
 Add reference: cros_library
In files:
 Add source (and header) files.

* In the new-project properties:
 First select the option: In configuration: "All configurations" and in platform: "All platforms". So the following option changes are applied to all configurations and platforms.
In General:
 Output folder: $(SolutionDir)$(Platform)\$(Configuration)\
 Intermediate folder: $(Platform)\$(Configuration)\
In Debug:
 Working directory: $(SolutionDir)$(Platform)\$(Configuration)\
In VC++ directories:
 Directory of header files: add: $(SolutionDir)..\include
In C/C++ -> Preprocessor:
 Preprocessor definitions: Add: _CRT_NONSTDC_NO_DEPRECATE;_CRT_SECURE_NO_WARNINGS
In Linker -> Input: Add: ;ws2_32.lib
In Linker -> System: Set Subsystem to Console

In Build events -> Post build events (only in cros_library project):
 Command line: xcopy /E /Y $(SolutionDir)..\samples\rosdb $(TargetDir)rosdb\
 Description: Copy ROS message definition files to working directory

You may also want to disable Win32 Control-C exception catching when debugging, so that this event is catched by the cROS program being run.

cROS library verbose option
---------------------------
When debugging the debug and information messages of the cROS library can be helpful. They can be activated by changing the value of CROS_DEBUG_LEVEL in cros_defs.h.
The default debug level value for console messages is 1 (#define CROS_DEBUG_LEVEL 1).
The maximum value (the most vebose level) is 4.
