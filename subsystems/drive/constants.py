import math
from utils.units import unit

# module configs
frontLeft = {
    "driveMotorId": 5,
    "turningMotorId": 6,
    "turningEncoderId": 11,
}

frontRight = {
    "driveMotorId": 1,
    "turningMotorId": 2,
    "turningEncoderId": 9,
}

backLeft = {
    "driveMotorId": 7,
    "turningMotorId": 8,
    "turningEncoderId": 12,
}

backRight = {
    "driveMotorId": 3,
    "turningMotorId": 4,
    "turningEncoderId": 10,
}

# module parameters
kWheelRadius = 2.0 * unit.inch
kWheelCircumference = kWheelRadius * 2 * math.pi / unit.turn
kEncoderResolution = 4096 #counts per rotation
kModuleMaxAngularVelocity = math.pi * unit.radian / unit.second
kModuleMaxAngularAcceleration = math.tau * unit.radian / unit.second / unit.second

kTurnRatio = (150.0 / 7.0)
kDriveRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)

kDrive_p = .01
kDrive_i = 0
kDrive_d = 0
kDrive_v = 12.0 / (100.0 / kDriveRatio)

kTurn_p = 40
kTurn_i = 0
kTurn_d = 0

# drivetrain paramaters
kMaxSpeed = 3.0 * unit.meter / unit.second  # 3 meters per second
kMaxAngularSpeed = math.pi * unit.radian / unit.second  # 1/2 rotation per second
kChassisWidth = 28.0 * unit.inch
kChassisLength = 28.0 * unit.inch

kGyroId = 20