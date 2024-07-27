#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# standard python imports

# project imports
import subsystems.drive.drivetrain
import subsystems.shooter.shootersubsystem
import constants

# wpi imports
import wpilib
import wpimath
import commands2
import commands2.cmd
import commands2.button


# vendor imports

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.

    """

    def __init__(self, isReal=True):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        # The robot's subsystems
        self.robotDrive = subsystems.drive.drivetrain.Drivetrain(isReal=isReal)
        self.shooter = subsystems.shooter.shootersubsystem.ShooterSubsystem()

        self.spinUpShooter = commands2.cmd.runOnce(self.shooter.enable, self.shooter)
        self.stopShooter = commands2.cmd.runOnce(self.shooter.disable, self.shooter)

        self.autonomousCommand = commands2.cmd.sequence(
            # Start the command by spinning up the shooter...
            commands2.cmd.runOnce(self.shooter.enable, self.shooter),
            # Wait until the shooter is at speed before feeding the frisbees
            commands2.cmd.waitUntil(lambda: self.shooter.getController().atSetpoint()),
            # Start running the feeder
            commands2.cmd.runOnce(self.shooter.runFeeder, self.shooter),
            # Shoot for the specified time
            commands2.cmd.waitSeconds(constants.AutoConstants.kAutoShootTimeSeconds)
            # Add a timeout (will end the command if, for instance, the shooter
            # never gets up to speed)
            .withTimeout(constants.AutoConstants.kAutoTimeoutSeconds)
            # When the command ends, turn off the shooter and the feeder
            .andThen(
                commands2.cmd.runOnce(
                    lambda: self.shooter.disable, self.shooter
                ).andThen(
                    commands2.cmd.runOnce(lambda: self.shooter.stopFeeder, self.shooter)
                )
            ),
        )

        # The driver's controller
        self.driverController = wpilib.XboxController(
            constants.kDriverControllerPort
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    xSpeed = -wpimath.applyDeadband(self.driverController.getRawAxis(1), 0.1) * subsystems.drive.drivetrain.constants.kMaxSpeed,
                    ySpeed = -wpimath.applyDeadband(self.driverController.getRawAxis(0), 0.1) * subsystems.drive.drivetrain.constants.kMaxSpeed,
                    rot = -wpimath.applyDeadband(self.driverController.getRawAxis(4), 0.1) * subsystems.drive.drivetrain.constants.kMaxAngularSpeed,
                    fieldRelative = True,
                    periodSeconds = commands2.TimedCommandRobot.kDefaultPeriod
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        self.driverController.a().onTrue(self.spinUpShooter)

        # Turn off the shooter when the 'B' button is pressed
        self.driverController.b().onTrue(self.stopShooter)

        # We can also write them as temporary variables outside the bindings

        # Shoots if the shooter wheel has reached the target speed
        shoot = commands2.cmd.either(
            # Run the feeder
            commands2.cmd.runOnce(self.shooter.runFeeder, self.shooter),
            # Do nothing
            commands2.cmd.none(),
            # Determine which of the above to do based on whether the shooter has reached the
            # desired speed
            lambda: self.shooter.getController().atSetpoint(),
        )

        stopFeeder = commands2.cmd.runOnce(self.shooter.stopFeeder, self.shooter)

        # Shoot when the 'X' button is pressed
        self.driverController.x().onTrue(shoot).onFalse(stopFeeder)

        # We can also define commands inline at the binding!

        # While holding the shoulder button, drive at half speed
        self.driverController.rightBumper().onTrue(
            commands2.cmd.runOnce(
                lambda: self.robotDrive.setMaxOutput(0.5), self.robotDrive
            )
        ).onFalse(
            commands2.cmd.runOnce(
                lambda: self.robotDrive.setMaxOutput(1), self.robotDrive
            )
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return self.autonomousCommand #or commands2.InstantCommand()