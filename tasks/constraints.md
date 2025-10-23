Here's a breakdown of the inputs, outputs, and limitations related to autonomy, specifically for the missions where autonomous functions are relevant:

### Equipment Servicing Mission (1.e)

  * **Inputs:**
      * A 3-6 letter launch key for the "Autonomous Typing" sub-task, which will be provided before the mission starts.
      * ArUco fiducial markers (2x2 cm) will be positioned at the keyboard's corners to aid alignment during autonomous typing.
      * A USB (type C) slot on the lander, connected to a standard memory card that contains a text file with GNSS coordinates.
      * ArUco fiducial markers (1x1 cm) will be placed at the corners of the USB slot.
  * **Outputs:**
      * The rover must autonomously input the launch key on a keyboard.
      * The rover must read GNSS coordinates from a text file on a memory card after inserting a USB cable.
  * **Limitations:**
      * For autonomous typing, correct typing is required for full points, but spelling mistakes or repeated letters can be corrected using the backspace/delete button or ignored for partial points.
      * Operator intervention during autonomous typing is only permitted to exit autonomous mode and to abort or restart the task from the beginning.

### Autonomous Navigation Mission (1.f)

  * **Inputs:**
      * Highly accurate GNSS-only coordinates for two specific locations.
      * GNSS coordinates provided for the vicinity of two posts, which will be marked with 3-sided visual markers displaying black and white ArUco tags (from the 4x4\_50 tag library, with 20x20 cm faces, positioned 0.5-1.5m off the ground).
      * GNSS coordinates (with less than 3 meters accuracy for the first two objects, and less than 10 meters accuracy for the third) for three ground objects: an orange rubber mallet, a rock pick hammer, and a standard 1-liter wide-mouthed plastic water bottle (color/markings unspecified).
  * **Outputs:**
      * An LED indicator on the back of the rover must signal:
          * Red: Autonomous operation
          * Blue: Teleoperation (Manual driving)
          * Flashing Green: Successful arrival at a target.
      * Upon successful arrival at a target, the rover must stop and display a large and obvious message or signal on the operator's display for the C2 station judge to observe.
      * The rover must clearly and autonomously highlight or designate detected objects on the display in the C2 station.
  * **Limitations:**
      * Teams have the flexibility to visit locations in any order.
      * Stopping within 3 meters of a GNSS-only location is considered a success.
      * Stopping within 2 meters of a post is considered a success.
      * The rovers are not required to interact with the objects.
      * The rover must be stopped to successfully indicate object recognition, but it can stop at any distance from the object as long as detection is successful.
      * Only one object can be highlighted on the display at a time.
      * Operators can send a signal at any point to abort the current attempt, causing the rover to autonomously return to any previous GNSS coordinate, post, or object and stop within 5 meters.
      * Teleoperation to return to a previously visited location will result in a 20% penalty for the points available for that location. Teleoperation should follow the most direct reasonable route and not be used for scouting. There is no penalty for an autonomous return.
      * During stops (after successful arrival or an abort), teams are permitted to perform programming, including entering GNSS points or waypoints, and making changes to controls and algorithms, but they are not allowed to drive the rover.
      * GNSS points provided in the vicinity of posts or objects do not count as targets for programming unless they are reached during an aborted attempt on another target.

### General Limitations (Applicable to Autonomy)

  * **Antenna Camera:** It is not permitted to mount a camera on top of the antenna for visual feedback (Section 3.b.iii).
