# Spatial Math 2D

![Image of application](/resources/screenshot.png)

This application is to aid in understanding, testing, and demonstrating 2D
spatial transformations.  Compiling this project also produces a static
`transform` library that can be statically linked and used in other projects.

It was inspired by a need for testing and validating transformation of data
between world and image coordinates and for transforming data between image
coordinates to local coordinates.

# Description and Use

Out of the box, the application displays a local axis system that is translated
`(400.0, 200.0)` and rotated 30 degrees counterclockwise from the image axis
system (origin at upper-left, with +x to the right and +y down).  The local
axis system's x-axis is in red and the y-axis is in green.

The grid is in increments of 5px in the x and y directions.

Pressing the left mous button will move the purple point to the current mouse
location.  If you hold the mouse down, the point will follow the mouse's
position.  The coordinates of the purple point are shown in both image ("World")
coordinates and relative to the local axis system ("Local").

