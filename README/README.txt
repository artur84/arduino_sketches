This project was created just to add general information that concerns
all the other sketches.

################################################
### FILE STRUCTURE
################################################
The projects in the arduino_sketches directory
are completely compatible with arduino IDE.

The folder sketchbook_eclipse is going to be removed and it was previously used 
to contain projects that were compatible with eclipse but not with arduino IDE.

The folder sketchbook_normal is going to be removed and it was previously used 
to contain projects that were compatible with  arduino IDE but not with eclipse.


################################################
### INSTALL ARDUINO IN ECLIPSE
################################################

I'm using:

- Install Eclipse Mars 2.0  from eclipse webpage.

- Install java 1.8 (Needed by the arduino plugin).
	You need to install at least java 1.8. (Check the instructions here).
	http://www.webupd8.org/2012/09/install-oracle-java-8-in-ubuntu-via-ppa.html
	
	sudo add-apt-repository ppa:webupd8team/java
	sudo apt-get update
	sudo apt-get install oracle-java8-installer

- Install Plugin from eclipse marketplace.
	
NOTES:
Don't forget to add used libraries as Wire.h, ros_lib etc..
Right Click Arduino in the top menu.
	
	Arduino -> Add a library to the selected project

