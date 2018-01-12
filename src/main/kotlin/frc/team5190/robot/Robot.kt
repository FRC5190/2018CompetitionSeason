package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import frc.team5190.robot.drive.DriveTrain
import frc.team5190.robot.navigation.NAVFeeder
import frc.team5190.robot.navigation.NAVHelper
import frc.team5190.robot.navigation.NAVHelper.Companion.leftPoints
import frc.team5190.robot.navigation.NAVHelper.Companion.rightPoints

class Robot : IterativeRobot() {

    init {
        DriveTrain
    }

    private val leftMotionProfile by lazy {
        NAVFeeder(DriveTrain.frontLeft, NAVFeeder.Side.LEFT)
    }
    private val rightMotionProfile by lazy {
        NAVFeeder(DriveTrain.frontRight, NAVFeeder.Side.RIGHT)
    }

    override fun autonomousPeriodic() {
        leftMotionProfile.control()
        rightMotionProfile.control()

        val leftOutput = leftMotionProfile.getSetValue()
        val rightOutput = rightMotionProfile.getSetValue()

        DriveTrain.frontLeft.set(ControlMode.MotionProfile, leftOutput.value.toDouble())
        DriveTrain.frontRight.set(ControlMode.MotionProfile, rightOutput.value.toDouble())

        leftMotionProfile.startMotionProfile()
        rightMotionProfile.startMotionProfile()
    }

    override fun disabledInit() {
        leftMotionProfile.reset()
        rightMotionProfile.reset()

        Reset().start()
    }

    override fun robotInit() {
        val autoGenerator = NAVHelper.Auto.CENTER

        leftPoints  = autoGenerator.trajectoryLeft
        rightPoints = autoGenerator.trajectoryRight
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }
}

