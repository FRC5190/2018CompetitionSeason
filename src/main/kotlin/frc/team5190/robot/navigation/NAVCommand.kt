package frc.team5190.robot.navigation

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveTrain
import frc.team5190.robot.util.Hardware

class NAVCommand(private val path: NAVHelper) : Command() {
    private lateinit var leftMotionProfile: NAVFeeder
    private lateinit var rightMotionProfile: NAVFeeder

    val leftPos = DriveTrain.frontLeft.sensorCollection.quadraturePosition
    val rightPos = DriveTrain.frontRight.sensorCollection.quadraturePosition

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
        leftMotionProfile.control()
        rightMotionProfile.control()

        DriveTrain.frontLeft.set(ControlMode.MotionProfile, leftMotionProfile.getSetValue().value.toDouble())
        DriveTrain.frontRight.set(ControlMode.MotionProfile, rightMotionProfile.getSetValue().value.toDouble())
    }

    override fun end() {
        leftMotionProfile.reset()
        rightMotionProfile.reset()
        DriveTrain.tankDrive(0.0, 0.0, ControlMode.PercentOutput)

        val leftPosDifference = DriveTrain.frontLeft.sensorCollection.quadraturePosition - leftPos
        val rightPosDifference = DriveTrain.frontRight.sensorCollection.quadraturePosition - rightPos

        val leftFeet = ((leftPosDifference / Hardware.NATIVE_UNITS_PER_ROTATION) * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12
        val rightFeet = ((rightPosDifference / Hardware.NATIVE_UNITS_PER_ROTATION) * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12

        println("Left Distance Traveled: $leftFeet, Right Distance Traveled: $rightFeet")
    }

    override fun isFinished(): Boolean = leftMotionProfile.hasFinished() && rightMotionProfile.hasFinished()
}