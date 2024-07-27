kDriverControllerPort = 0

class ShooterConstants:
    kEncoderPorts = (4, 5)
    kEncoderReversed = False
    kEncoderCPR = 1024
    kEncoderDistancePerPulse = 1 / kEncoderCPR

    kShooterMotorPort = 4
    kFeederMotorPort = 5

    kShooterFreeRPS = 5300
    kShooterTargetRPS = 4000
    kShooterToleranceRPS = 50

    # These are not real PID gains, and will have to be tuned for your specific robot.
    kP = 1
    kI = 0
    kD = 0

    # On a real robot the feedforward constants should be empirically determined; these are
    # reasonable guesses.
    kSVolts = 0.05

    # Should have value 12V at free speed...
    kVVoltSecondsPerRotation = 12.0 / kShooterFreeRPS

    kFeederSpeed = 0.5
