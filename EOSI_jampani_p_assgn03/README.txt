
PRADEEP CHOWDARY JAMPANI (1215335693)

Steps to run the program:

GENERAL SETUP:

1) unzip the package to any working directory in your machine.

2) Install and setup the board according to the instructions provided in the following link:

https://www.dropbox.com/sh/7t0tf61zkxmtxxw/AAB4HiFN5YmckFe6-l2J6kkTa/Galileo%20Gen%202%20SDK?dl=0&preview=Setting-Up_Galileo_2018.pdf&subfolder_nav_tracking=1


ASSIGNMENT SETUP:

1) Power-on the board

2) Now connect the IO pins 8 and 9 to the  trigger and Echo pins of the sensor HC-SR04 and connect the pins IO 11, 12 and 13 to the MOSI, Chip-select and SCK-CLK pins of MAX7219 LED Driver.

COMPILATION:

1) Open the  package, and type the "make" command. 

2) The created object file and the device driver module is automatically transferred to our board.

EXECUTION:

1) In the root directory of the board, type the following command to remove the already existing spi device driver:

	Type "rmmod spidev.ko" and Enter

2) Now install our device driver as follows:

	"insmod Socket_NLP.ko"

3) After installation, start the test program as 

	"/led_sock" and Enter

The test program prints the distance measured between the object and the sensor always.

4) Let's get into the Animation. Animation explains a part of the Evolution process for the Sub-Phylum Vertebrata upto Man.
	
We, Homosapiens, have gone through so many stages of evolution and thus a part of which is explained below. Consider the distance as some millions of years.

	Distance > 100 			-> Swimming Fish is displayed. Class Pisces.
	Distance between 100 and 70 	-> Jumping Frog is displayed. Class Amphibians.
	Distance between 70 and 50	-> Moving Tortoise is displayed. Class Reptilia.
	Distance between 50 and 25	-> Flying Bird is displayed. Class Ares.
	Distance less than 25		-> Dancing Humans. Class Mammals.
	
5) The program runs for 15 measurements and stops. 


6) Finally remove the installed driver by typing the following command.

	"rmmod Socket_NLP.ko"


