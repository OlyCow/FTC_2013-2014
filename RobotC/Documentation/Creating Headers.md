# Creating Headers
When you create header files (`.h`, or, occasionally, `.c`), you are adding to a _library_ of functions/constants.
Therefore, please take the time to read this and make your creations reusable! Thank you! :)

This might seem like a long read, but once you get the hang of it, it takes about 10 seconds to make sure you
do everything recommended here. :)
Of course, you also should probably look at some of our headers, and make sure yours has the same general "style".


## Files Types (and Structure)
A good programmer separates _implementation_ from _interface_. Your header file is your _interface_, while your
library file is your _implementation_. That's also why we have 2 separate folders--to force you to consider this.
Usually, you only define all of your constants (usually `const`s and _not_ `#define`s!) in your header, as well
as `enum`s. For functions, a forward declaration is sufficient. The _implementation_--the actual code-- should be
put in the `\Libraries\` folder, and the _interface_ in the `\Headers\` folder. We discuss `#include`ing your
_implementation_ in the "Other `#include`s" section.

So your interface should look something like this.

`foo.h`
	#ifndef FOO_H
	#define FOO_H
	#pragma systemFile
	#include "..\Libraries\foo.c"
	
	const double foo = 3.14;
	const short bar = 1984;
	typedef enum baz
	{
		BAZ_FOO	=0,
		BAZ_BAR	=1,
		BAZ_BAZ	=2,
	};
	
	void uqbar(baz fooBar);
	bool tlon(baz fooBar);
	
	#endif // FOO_H
_In the `\Headers\` folder, of course._

While your implementation might look something like this.

`foo.c`
	#ifndef FOO_C
	#define FOO_C
	#pragma systemFile
	
	void uqbar(baz fooBar);
	{
		fooBar = BAZ_BAZ;
	}
	
	bool tlon(baz fooBar)
	{
		if (fooBar==BAZ_FOO)
			return true;
		else
			return false;
	}
	
	#endif // FOO_C
_In the `\Libraries\` folder, of course._

Another thing to be aware of is--RobotC does not actually have a linker (!!!). This means that you cannot separate
your code into different files and then link them together when you compile. How then, you might ask, is
`JoystickDriver.c` compiled? It actually isn't intrinsic (which is the sort of thing you'd expect from RobotC :P),
but rather, a neat trick. If you attempt to compile `JoystickDriver.c` you will notice it can't actually compile.
This is because in reality, `JoystickDriver.c` is merely a header file, which is `#include`d into your code (just
like any other header file might be!).

Then why do we still insist on separating your `.h`s (interface) from your `.c`s (implementation)? Because it's
still more readable to separate them. Would you ever dream of figuring out an API just by looking at raw code?
The same applies here. And if RobotC ever gets a linker, your transition will be as simple as deleting a few lines
of code. (That's right, deleting, not adding.) See the section titled "Other `#include`s" for more information.


## Header Guards
To ensure that when your headers are included multiple times nothing bad happens (e.g. redefinitions), be sure to
_always_ use header guards. These (traditionally) consist of 2 lines of code at the beginning of the file and a line
at the end of the file. They would look something like the following.

`foo.h`:
	#ifndef FOO_H
	#define FOO_H
	
	// ... Your code here :)
	
	#endif // FOO_H
_The header guards are in all caps by convention (to minimize naming conflicts, I presume)._

The "variable" `FOO_H` should be a unique identifier for your header file, in all caps, and substituting underscores
for any puntuation. When the preprocessor first `#include`s your header, this identifier isn't defined yet, so the
preprocessor `#define`s it the next line. It also executes (`#include`s the rest of your header code), all the way
until it encounters the `#endif` at the end. (It is courteous to comment in what you are `#endif`ing, to maintain
readability.) Now, the subsequent times you `#include` this header, `FOO_H` will be `#define`d already, and your
header won't be `#included` over and over.


## `#pragma systemFile`
This is a handy `#pragma` which tells the RobotC compiler _not_ to give you warnings about unreferenced functions
and the like. Usually you will want to include this line, unless you have special reasons not to (all of the functions
in your library should be accessed, or you like all those yellow crosses, or something--I don't know!). Judge for
yourself on this one.


## Other `#include`s
You might have noticed a special header file (in the `\Headers\` folder) named `includes.h`--this is very crucial
if you want everything to "link". Ideally, in every program you write (`.c`), you will only need to include this
header (and any program specific headers if you must).

All you need to keep in mind is to `#include` headers only, and do so in order of dependency. (e.g., if `foo.h`
depends on a constant defined in `bar.h`, `#include` `bar.h` before `foo.h`.) Of course, if you end up having some
sort of circular dependency, that sucks :) (and cannot be resolved). So do _not_ include any of your `.c` `\Library\`
files, as those should be `#include`d in their corresponding headers (see below).

In each of your header files, remember to `#include` the corresponding `.c` `\Library\` file. And then, since you
already put the header files in order of dependency, you won't have any "foo is undefined" errors in your `.c` (or
`.h`) files. And it's all as simple as that.


## Content
Oh, and by the way: Who cares what your code does! Just make sure you keep it modular and reuseable--you don't know
who might be thanking you down the road; it might even be yourself. :D
