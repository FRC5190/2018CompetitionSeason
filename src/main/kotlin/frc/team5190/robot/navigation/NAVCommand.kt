package frc.team5190.robot.navigation

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.MainXbox
import frc.team5190.robot.drive.DriveSubsystem

class NAVCommand(private val path: NAVHelper) : Command() {

    init {
        requires(DriveSubsystem)
    }

    private lateinit var leftMotionProfile: NAVFeeder
    private lateinit var rightMotionProfile: NAVFeeder

    var paused = false

    override fun initialize() {
        leftMotionProfile = NAVFeeder(DriveSubsystem.falconDrive.leftMaster, path.trajectoryLeft)
        leftMotionProfile.startMotionProfile()

        rightMotionProfile = NAVFeeder(DriveSubsystem.falconDrive.rightMaster, path.trajectoryRight)
        rightMotionProfile.startMotionProfile()
    }

    override fun execute() {
        SmartDashboard.putBoolean("Obstacle Present", MainXbox.bButton)

        if (MainXbox.bButton) {
            leftMotionProfile.pauseMotionProfile()
            rightMotionProfile.pauseMotionProfile()
            paused = true
            return
        } else if (paused) {
            leftMotionProfile.resumeMotionProfile()
            rightMotionProfile.resumeMotionProfile()
        }

        leftMotionProfile.control()
        rightMotionProfile.control()

        DriveSubsystem.falconDrive.leftMaster.set(ControlMode.MotionProfile, leftMotionProfile.getSetValue().value.toDouble())
        DriveSubsystem.falconDrive.rightMaster.set(ControlMode.MotionProfile, rightMotionProfile.getSetValue().value.toDouble())
    }

    override fun end() {
        leftMotionProfile.reset()
        rightMotionProfile.reset()
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    override fun isFinished() = leftMotionProfile.hasFinished() && rightMotionProfile.hasFinished()
}