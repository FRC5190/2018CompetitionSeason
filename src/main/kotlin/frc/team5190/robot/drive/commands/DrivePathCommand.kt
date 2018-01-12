package frc.team5190.robot.drive.commands

import com.ctre.phoenix.motorcontrol.ControlMode
import frc.team5190.robot.auto.AutoPath
import frc.team5190.robot.auto.MotionProfileManager
import frc.team5190.robot.drive.DriveSubsystem

class DrivePathCommand(val autoPath: AutoPath) : DriveCommand() {

    private lateinit var leftMotionProfile: MotionProfileManager
    private lateinit var rightMotionProfile: MotionProfileManager

    override fun initialize() {
        leftMotionProfile = MotionProfileManager(DriveSubsystem.falconDrive.leftMaster, autoPath.trajectoryLeft)
        rightMotionProfile = MotionProfileManager(DriveSubsystem.falconDrive.rightMaster, autoPath.trajectoryRight)
    }

    override fun execute() {
        leftMotionProfile.control()
        rightMotionProfile.control()

        DriveSubsystem.falconDrive.leftMaster.set(ControlMode.MotionProfile, leftMotionProfile.setValue.value.toDouble())
        DriveSubsystem.falconDrive.rightMaster.set(ControlMode.MotionProfile, rightMotionProfile.setValue.value.toDouble())
    }

    override fun end() {
        leftMotionProfile.reset()
        rightMotionProfile.reset()
        DriveSubsystem.falconDrive.tankDrive(0.0, 0.0)
    }

    override fun isFinished() = false
}
