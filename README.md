The 'Oly Cow: FTC '13-'14
=========================

The code for The 'Oly Cow's 2013-2014 robot (team 6424). You are welcome to use our code, although there is no warranty :P

An explanation of the code will be added shortly--of course, it would help to actually have code :)
Also, our code is pretty specific to our robot, but the #pragma motor/servo assignments should make it reasonably clear.
If you dare to read on, remember that this is all still very preliminary :)

//TODO: update README.md

-------------------------
The "\Competition\" folder contains the ".rxe"s which should be loaded during competition if something goes wrong.
ONLY working stuff gets to go in here. And no modifying code during competition (of course there are exceptions ;).

In our "\RobotC\" folder, there are a few sub-folders: Documentation, Drivers, Headers, Libraries, and Resources.
All the "main" programs are found in the root of this folder as well.

- "Swerving Test.c"
	- A quick program whipped up to demonstrate a single swerve drive wheel pod; utilizes a single "normal" servo
	  instead of a continuous rotation servo (which we plan on using for our final robot).

## "\Documentation\"
Includes documentation on what everything does -  so when you're done reading this, go take a look at that :)

## "\Drivers\"
These are the headers (which we use) from Xander's Driver Suite. These are compiled separately to increase compatibility
between platforms, and to ensure our code is always using the same versions of drivers. All of these drivers should be
periodically updated when new versions of the driver suite come out; but never w/out extensive testing to make sure the
current codebase (esp. encapsulation) is updated correspondingly.

## "\Headers\"
Includes headers that some (or all) programs may use; if you don't want to read documentation you can try reading this.

## "\Libraries\"
Includes the actual ".c" files which some of the headers are for. Technically these are "header files" as well, since they
need to be #included into their respective programs. (This is because RobotC doesn't currently have a linker, and judging
by forum posts one won't be added any time soon.)

## "\Resources\"
Includes some sound files (".rso") that we might attempt to play. However, competitions are usually too noisy :(
