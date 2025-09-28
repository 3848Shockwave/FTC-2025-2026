## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Command-Based Programming Structure

This project uses a command-based programming structure provided by FTCLib library. This approach separates robot functionality into subsystems and commands, making the code more modular and easier to manage.

### What is Command-Based Programming?

Command-Based Programming is a design pattern that helps organize robot code by separating the robot's physical components (subsystems) from the actions that can be performed with those components (commands). This pattern makes the code more modular, reusable, and easier to test.

### Core Concepts

#### Subsystems
Subsystems represent physical components of the robot and encapsulate their behavior. Each subsystem should represent a single physical mechanism or a group of related mechanisms.

Key characteristics of subsystems:
- They represent physical parts of the robot (e.g., drivetrain, arm, claw)
- They contain all the methods needed to control that part of the robot
- They should not directly respond to user input (that's what commands are for)
- They should not depend on other subsystems directly

#### Commands
Commands represent actions that can be performed by subsystems. Each command specifies what a subsystem should do and when the action is complete.

Key characteristics of commands:
- They define actions that the robot can perform
- They can use one or more subsystems
- They have a clear start, execute, and end cycle
- They can be scheduled to run automatically

#### Command Scheduler
The command scheduler is responsible for managing which commands are running at any given time. It ensures that commands are properly initialized, executed, and cleaned up.

### How It Works

1. **Initialization**: When the robot starts, subsystems are created and default commands are set
2. **Scheduling**: Commands are scheduled to run based on triggers (like button presses)
3. **Execution**: The scheduler calls each running command's execute() method periodically
4. **Completion**: Commands end when isFinished() returns true or when interrupted by another command
5. **Cleanup**: The end() method is called to clean up resources

### Benefits of Command-Based Programming

1. **Modularity**: Each subsystem and command is self-contained, making code easier to understand and maintain
2. **Reusability**: Commands can be reused in different contexts and OpModes
3. **Parallel Operations**: Multiple commands can run simultaneously as long as they don't require the same subsystems
4. **Clear Structure**: The separation of concerns makes it easier to debug and extend functionality
5. **Testing**: Individual components can be tested in isolation

### Implementation in This Project

This project uses FTCLib's CommandOpMode as the base for teleoperated modes. The main components are:

#### Subsystems Package
Contains all the subsystem classes that represent physical parts of the robot:
- DriveSubsystem: Controls the robot's mecanum drive system
- ArmSubsystem: Controls the robot's arm mechanisms
- PanelsSubsystem: Controls telemetry and field visualization

#### CommandBase Package
Contains all the command classes that define actions the robot can perform:
- DriveCommand: Controls robot driving based on gamepad input
- ArmManualControlCommand: Provides manual control of the arm
- ArmStopCommand: Immediately stops the arm

#### CommandOpMode
The main teleoperated OpMode that initializes subsystems, sets default commands, and binds buttons to specific commands.

### Detailed Component Breakdown

#### Subsystems

1. **DriveSubsystem** - Controls the robot's mecanum drive system with field-centric control
   - Uses PedroPathing library for odometry and autonomous path following
   - Provides methods for field-centric driving and heading reset
   - Manages four drive motors (frontLeft, frontRight, backLeft, backRight)

2. **ArmSubsystem** - Controls the robot's arm mechanisms
   - Manages lift and extension motors
   - Provides methods for controlling arm movement and position tracking
   - Includes encoder initialization and motor configuration

3. **PanelsSubsystem** - Controls telemetry and field visualization
   - Integrates with Panels telemetry system for enhanced data display
   - Provides field visualization capabilities for robot position and targets
   - Manages telemetry updates and field drawing operations

#### Commands

1. **DriveCommand** - Controls robot driving
   - Takes input from driver gamepad for field-centric driving
   - Continuously executes while the robot is in teleop mode
   - Stops motors when interrupted

2. **ArmManualControlCommand** - Provides manual control of the arm
   - Takes input from operator gamepad for arm lift and extension
   - Continuous command that runs throughout teleop mode
   - Safely stops arm when interrupted

3. **ArmStopCommand** - Immediately stops the arm
   - Simple command that stops all arm motors
   - Finishes immediately after execution

### Main OpMode

**CommandBasedTeleOp** - The main teleoperated OpMode
- Initializes all subsystems and gamepads
- Sets default commands for each subsystem
- Binds gamepad buttons to specific commands
- Displays control information on the telemetry