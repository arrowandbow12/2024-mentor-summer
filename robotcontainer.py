#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# standard python imports

# project imports
import subsystems.drive.drivetrain
import constants

# wpi imports
import wpilib
import wpimath.controller
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

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        # The robot's subsystems
        self.robotDrive = subsystems.subsystems.drive.Drivetrain()

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
                    xSpeed = self.driverController.getLeftX() * subsystems.drive.drivetrain.constants.kMaxSpeed,
                    ySpeed =self.driverController.getLeftY() * subsystems.drive.drivetrain.constants.kMaxSpeed,
                    rot = self.driverController.getRightX() * subsystems.drive.drivetrain.constants.kMaxAngularSpeed,
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

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return commands2.InstantCommand()