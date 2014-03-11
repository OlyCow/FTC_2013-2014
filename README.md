The 'Oly Cow: FTC '13-'14
=========================

This repository contains most of the code for team 'Oly Cow's Block Party robot (#6424). You are welcome to use our
code as-is, but there is no warranty. We hope you can learn from our mistakes and maybe even glean some inspiration.
Few will do this, but we would really appreciate an attribution if you found the code here helpful, useful, or cool.
Since our robot is constantly changing, this code will never be finished--until perhaps the World Championships--and
the documentation for the code will probably never be up-to-date. If you have specific questions regarding the code,
one of the "release" versions should be reasonably well-commented, or you can directly contact one of our programmers
(Ernest, Kieran, or Nathan). You can also email us at mooovingforward [at] gmail [dot] com.

-------------------------

## Directory Structure
In addition to the files in the "Documentation" folder, there should/will be additional READMEs inside each folder with
even more information/help.

### "Atmel"
This folder contains all the "solution" project files for the code that is on our microcontrollers. We use these MCUs
to collect all sorts of sensor data, and then relay that back to the NXT through the HiTechnic SuperPro prototyping
board, using a custom communication protocol.

### "Competition"
This folder contains .hex files for NXT bricks. These can be directly downloaded to any NXT with RobotC firmware on
them. (These should only be used in case of an emergency.) As a rule, these will be updated around a week before each
competition we attend. There should be minimal modification to the code during that last week (there should also be a
corresponding "release" of the code in preparation for the same competition). There will also be .hex files for MCUs
in here, but those should rarely (if ever) need to be used. You better know what you're doing if you tamper with those.

### "Documentation"
This folder will (might?) (someday) have documents regarding use of our code, and maybe even extra explanations.

### "RobotC"
This folder contains all of our RobotC code, as well as some programs we wrote for our sister team (Team #7973, The One
and Oly). This folder will be updated the most frequently. There are additional goodies in here, such as a medium-sized
library of convenience functions (now including basic 2-D vector operations!), and a good dose of "utilities" just for
testing random sensors and whatnot.

-------------------------

## Contact Information & Attribution

You can contact our team at our email: mooovingforward [at] gmail [dot] com.

Our Facebook page is here: j.mp/OlyCowRobotics

And we are very active on the FTC and RobotC forums. As always, the 'Oly Cow: Outstanding in our field.
