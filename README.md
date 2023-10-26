# FRC-2023-Public
Team 254's 2023 FRC robot code for [Breakdown](https://www.team254.com/first/2023/). Breakdown's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `FRC-2023-Public.ipr` file with IntelliJ

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `FRC-2023-Public` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Code Highlights
* Talon Config Verification

    The robot uses [config verification](src/main/java/com/team254/lib/drivers/TalonUtil.java) when a configuration fails to apply for a Talon. New configurations are passed through a checker that re-attempts a failed configuration (determined through Phoenix Pro’s Status Code) several times to ensure that a new config is applied. This is used in any config update for talons, such as [configuring drive motors](src/main/java/com/team254/lib/drivers/SwerveModule.java#L68) or [setting current limits for the laterator in limp mode](src/main/java/com/team254/frc2023/subsystems/Superstructure.java#L330).

* Vision Pose Correction

    The robot fuses odometry with vision updates to maintain an accurate pose for auto-align and autonomous mode. Vision updates include the timestamp, camera to target (calculated using the [Pinhole Camera Model](src/main/java/com/team254/frc2023/subsystems/Limelight.java#L285)), and the April tag ID to calculate the vision field to robot distance and robot to tag distance. [Faulty vision updates](src/main/java/com/team254/frc2023/limelight/VisionPoseAcceptor.java) are rejected based on the robot’s distance from the tags and its velocity. Odometry drift is corrected by accepted vision updates with an [Unscented Kalman Filter](src/main/java/com/team254/lib/kalman/UnscentedKalmanFilter.java) to estimate a [true pose of the robot’s location](src/main/java/com/team254/frc2023/RobotState.java#L206).

* Auto Align

    The robot uses Vision Pose Correction and Motion Profiling to optimally move to the closest scoring position. The [nearest alignment point for scoring](src/main/java/com/team254/frc2023/planners/AutoAlignPointSelector.java) is calculated using the vision-corrected robot pose. Three [profile followers](src/main/java/com/team254/lib/motion/ProfileFollower.java) for [x, y, and theta](src/main/java/com/team254/frc2023/planners/AutoAlignMotionPlanner.java#L43) are created to optimally reach the target point within a deadband. Auto align includes two modes: one to align x, y, and theta, and the other to only align y and theta.

* Limp Mode

    The robot sets low current limits for the laterator when [intaking from the feeder](src/main/java/com/team254/frc2023/subsystems/Superstructure.java#L326), and the [ground intake when intaking cubes](src/main/java/com/team254/frc2023/subsystems/Superstructure.java#L375). These will compress like a spring upon contact with the feeder station and walls, allowing cycle times to be minimized and ensuring that the laterator and ground intake will not break easily.

## Package Functions
- [`com.team254.frc2023`](src/main/java/com/team254/frc2023)

    Contains the robot's central functions and holds a class with all numerical constants used throughout the code (see [`Constants.java`](src/main/java/com/team254/frc2023/Constants.java)). For example, the [`Robot`](src/main/java/com/team254/frc2023/Robot.java) class controls all routines depending on the robot mode. In addition, the [`RobotState`](src/main/java/com/team254/frc2023/RobotState.java) class keeps track of the current position of the robot's various frames of reference.

- [`com.team254.frc2023.auto`](src/main/java/com/team254/frc2023/auto)

    Handles the execution of autonomous routines and contains the [`actions`](src/main/java/com/team254/frc2023/auto/actions) and [`modes`](src/main/java/com/team254/frc2023/auto/modes) packages.

- [`com.team254.frc2023.auto.actions`](src/main/java/com/team254/frc2023/auto/actions)

    Contains all actions used during the autonomous period, which all share a common interface, [`Action`](src/main/java/com/team254/frc2023/auto/actions/Action.java) (also in this package). Examples include driving paths, docking, and scoring game pieces. Actions interact with the subsystems, which in turn interact with the hardware.

- [`com.team254.frc2023.auto.modes`](src/main/java/com/team254/frc2023/auto/modes)

    Contains all autonomous modes. Autonomous modes consist of a list of autonomous actions executed in a specific order.

- [`com.team254.frc2023.controlboard`](src/main/java/com/team254/frc2023/controlboard)

    Contains code for the driver to use either joysticks or gamepad and the operator to use a gamepad. Also contains a wrapper class specifically for Xbox controllers (see [XboxController.java](src/main/java/com/team254/frc2023/controlboard/XboxController.java)).

- [`com.team254.frc2023.loops`](src/main/java/com/team254/frc2023/loops)

    Contains codes for loops, which are routines that run periodically on the robot, such as for calculating robot pose, processing vision feedback, or updating subsystems. All loops implement the [`Loop`](src/main/java/com/team254/frc2023/loops/Loop.java) interface and are handled (started, stopped, added) by the [`Looper`](src/main/java/com/team254/frc2023/loops/Looper.java) class, which runs at 100 Hz. The [`Robot`](src/main/java/com/team254/frc2023/Robot.java) class has one main looper, `mEnabledLooper`, that runs all loops when the robot is enabled.

- [`com.team254.frc2023.paths`](src/main/java/com/team254/frc2023/paths)

    Contains the [`TrajectoryGenerator`](src/main/java/com/team254/frc2023/paths/TrajectoryGenerator.java) class which contains the trajectories that the robot drives during autonomous mode. Each `Trajectory` is composed of a list of `Waypoint` objects and headings.

- [`com.team254.frc2023.planners`](src/main/java/com/team254/frc2023/planners)

    Contains the [`DriveMotionPlanner`](src/main/java/com/team254/frc2023/planners/DriveMotionPlanner.java) class which controls the drivebase as it follows a trajectory during the autonomous period.

- [`com.team254.frc2023.shooting`](src/main/java/com/team254/frc2023/shooting)

    Contains the [`ShootingUtil`](src/main/java/com/team254/frc2023/shooting/ShootingUtil.java) helper class which takes in the current target range and robot state and returns parameters for an ideal shot for both stationary and shooting on the move.

- [`com.team254.frc2023.states`](src/main/java/com/team254/frc2023/states)

    Contains multiple classes representing LED states used in the [`Superstructure`](src/main/java/com/team254/frc2023/subsystems/Superstructure.java) class. 

- [`com.team254.frc2023.subsystems`](src/main/java/com/team254/frc2023/subsystems)

    Contains code for subsystems, which are consolidated into one central class per subsystem, all of which extend the [`Subsystem`](src/main/java/com/team254/lib/drivers/Subsystem.java) abstract class. Each subsystem uses state machines for control and is a singleton, meaning that there is only one instance of each. Subsystems also contain an enabled loop, a read periodic inputs method, and a write periodic outputs method, which are controlled by the [`SubystemManager`](src/main/java/com/team254/frc2023/SubsystemManager.java) class.

- [`com.team254.lib.control`](src/main/java/com/team254/lib/control)

    Contains classes used for the robot's path following and alternative teleoperated driver modes.

- [`com.team254.lib.drivers`](src/main/java/com/team254/lib/drivers)

    Contains a set of custom classes for motor controllers, color sensors, and solenoids for simplifying motor configuration, reducing CAN Bus usage, and checking motors. 

- [`com.team254.lib.geometry`](src/main/java/com/team254/lib/geometry)

    Contains a set of classes that represent various geometric entities.

- [`com.team254.lib.kalman`](src/main/java/com/team254/lib/kalman)

    Contains classes used to construct an Unscented Kalman Filter for noise correction.

- [`com.team254.lib.motion`](src/main/java/com/team254/lib/motion)

    Contains all motion profiling code used for autonomous driving. Trapezoidal motion profiles are used for smooth acceleration and minimal slip.

- [`com.team254.lib.physics`](src/main/java/com/team254/lib/physics)

    Contains classes to represent physical states of a swerve drive, a swerve's effective wheelbase, and a DC motor.

- [`com.team254.lib.spline`](src/main/java/com/team254/lib/spline)

    Contains classes to generate and time parameterize splines for smooth autonomous paths.

- [`com.team254.lib.swerve`](src/main/java/com/team254/lib/swerve)

    Contains various drive controllers and classes used for forward and inverse kinematics. 

- [`com.team254.lib.trajectory`](src/main/java/com/team254/lib/trajectory)

    Contains multiple classes used for representing and following [`Trajectory`](src/main/java/com/team254/lib/trajectory/Trajectory/java) objects.

- [`com.team254.lib.trajectory.timing`](src/main/java/com/team254/lib/trajectory/timing)

    Contains multiple classes for generating time-parameterized trajectories that obey physical robot constraints.

- [`com.team254.lib.util`](src/main/java/com/team254/lib/util)

    Contains a collection of assorted utilities classes used in the robot code. Check each file for more information.

- [`com.team254.lib.vision`](src/main/java/com/team254/lib/vision)

    Contains various classes that help with tracking and storing information about vision targets.


## Variable Naming Conventions
- k*** (i.e. `kDriveWheelbaseMeters`): Final constants, especially those found in the [`Constants.java`](src/main/java/com/team254/frc2023/Constants.java) file
- m*** (i.e. `mPathFollower`): Private instance variables
