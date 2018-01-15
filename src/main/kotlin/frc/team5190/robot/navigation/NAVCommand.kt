package frc.team5190.robot.navigation

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.OI
import frc.team5190.robot.drive.DriveTrain

class NAVCommand(private val path: NAVHelper) : Command() {

    private lateinit var leftMotionProfile: NAVFeeder
    private lateinit var rightMotionProfile: NAVFeeder

    var wasPaused = false

    init {
        requires(DriveTrain)
    }

    override fun initialize() {
        leftMotionProfile = NAVFeeder(DriveTrain.frontLeft, path.trajectoryLeft)
        leftMotionProfile.startMotionProfile()

        rightMotionProfile = NAVFeeder(DriveTrain.frontRight, path.trajectoryRight)
        rightMotionProfile.startMotionProfile()
    }

    override fun execute() {

        SmartDashboard.putBoolean("Obstacle Present", OI.xbox.bButton)

        if (OI.xbox.bButton) {
            leftMotionProfile.pauseMotionProfile()
            rightMotionProfile.pauseMotionProfile()
            wasPaused = true
            return
        } else if (wasPaused) {
            leftMotionProfile.resumeMotionProfile()
            rightMotionProfile.resumeMotionProfile()
        }

        leftMotionProfile.control()
        rightMotionProfile.control()

        DriveTrain.frontLeft.set(ControlMode.MotionProfile, leftMotionProfile.getSetValue().value.toDouble())
        DriveTrain.frontRight.set(ControlMode.MotionProfile, rightMotionProfile.getSetValue().value.toDouble())
    }

    override fun end() {
        leftMotionProfile.reset()
        rightMotionProfile.reset()
        DriveTrain.tankDrive(0.0, 0.0, ControlMode.PercentOutput)
    }

    override fun interrupted() {
        this.end()
    }

    override fun isFinished() = leftMotionProfile.hasFinished() && rightMotionProfile.hasFinished()
}