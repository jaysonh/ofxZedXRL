# All variables and this file are optional, if they are not present the PG and the
# makefiles will try to parse the correct values from the file system.
#
# Variables that specify exclusions can use % as a wildcard to specify that anything in
# that position will match. A partial path can also be specified to, for example, exclude
# a whole folder from the parsed paths from the file system
#
# Variables can be specified using = or +=
# = will clear the contents of that variable both specified from the file or the ones parsed
# from the file system
# += will add the values to the previous ones in the file or the ones parsed from the file 
# system
# 
# The PG can be used to detect errors in this file, just create a new project with this addon 
# and the PG will write to the console the kind of error and in which line it is

meta:
	ADDON_NAME 	       = ofxZedXRL
	ADDON_DESCRIPTION  = Addon for using ZED stereo camera, created at XRL Lab, SCM City University of Hong Kong, 
	ADDON_DESCRIPTION += Requires latest version of ZED drivers and CUDA 7.5, tested on of11 on windows 10
	ADDON_DESCRIPTION += You will need to edit include paths to make sure they point to your CUDA path
	
	ADDON_AUTHOR = Jayson Haebich 
	ADDON_TAGS   = "computer vision" "opencv" "image processing"
	ADDON_URL    = https://github.com/jaysonh/ofxZedXRL

vs:
	ADDON_DEPENDENCIES  = ofxOpenCv
	ADDON_DEPENDENCIES += ofxCv
	
	ADDON_LIBS += C:\Program Files (x86)\ZED SDK\lib\sl_zed64.lib

	ADDON_INCLUDES  = libs/ofxCv/include
	ADDON_INCLUDES += libs/CLD/include/CLD
	ADDON_INCLUDES += src
	ADDON_INCLUDES += C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v7.5\include
	ADDON_INCLUDES += C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v7.5
	ADDON_INCLUDES += C:\Program Files (x86)\ZED SDK\include
	ADDON_INCLUDES += C:\Program Files (x86)\ZED SDK
	
