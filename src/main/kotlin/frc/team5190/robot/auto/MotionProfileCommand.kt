/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.DriveConstants
import frc.team5190.robot.util.Maths
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.followers.EncoderFollower

open class MotionProfileCommand(folder: String, file: String, private val robotReversed: Boolean, isMirrored: Boolean, val useGyro: Boolean = true, val gyroMultiplier: Double = 1.0) : Command() {

    private val syncNotifier = Object()
    private var stopNotifier = false

    private val leftPath: Trajectory
    private val rightPath: Trajectory

    private val leftEncoderFollower: EncoderFollower
    private val rightEncoderFollower: EncoderFollower

    val mpTime
        get() = leftPath!!.length() * DriveConstants.MOTION_DT

    private val notifier: Notifier

    private var startTime: Double? = null

    init {
        val trajectories = Pathreader.getPath(folder, file)
        leftPath = trajectories[0]
        rightPath = trajectories[1]

        this.requires(DriveSubsystem)

        val leftTrajectory = if (isMirrored) rightPath else leftPath
        val rightTrajectory = if (isMirrored) leftPath else rightPath

        leftEncoderFollower = EncoderFollower(if (robotReversed) rightTrajectory else leftTrajectory).apply {
            configureEncoder(DriveSubsystem.falconDrive.leftEncoderPosition, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS / 6.0)
            configurePIDVA(2.0, 0.0, 0.0, 1 / Maths.rpmToFeetPerSecond(DriveConstants.MAX_RPM_HIGH, DriveConstants.WHEEL_RADIUS), 0.0)
        }

        rightEncoderFollower = EncoderFollower(if (robotReversed) leftTrajectory else rightTrajectory).apply {
            configureEncoder(DriveSubsystem.falconDrive.rightEncoderPosition, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS / 6.0)
            configurePIDVA(2.0, 0.0, 0.0, 1 / Maths.rpmToFeetPerSecond(DriveConstants.MAX_RPM_HIGH, DriveConstants.WHEEL_RADIUS), 0.0)
        }

        notifier = Notifier {
            synchronized(syncNotifier) {
                if (stopNotifier) {
                    println("Oof MotionProfile Notifier still running!")
                    return@Notifier
                }

                val leftOutput = leftEncoderFollower.calculate(DriveSubsystem.falconDrive.leftEncoderPosition)
                val rightOutput = rightEncoderFollower.calculate(DriveSubsystem.falconDrive.rightEncoderPosition)

                var actualHeading = (if (isMirrored) 1 else -1) * NavX.angle

                if (gyroMultiplier == -1.0) {
                    actualHeading = Pathfinder.boundHalfDegrees(actualHeading + 180.0)
                }

                val desiredHeading = Pathfinder.r2d(leftEncoderFollower.heading)

                val angleDifference = Pathfinder.boundHalfDegrees((desiredHeading) - (actualHeading))
                var turn = (if (useGyro) 1.6 else 0.0) * (-1 / 80.0) * angleDifference * (if (robotReversed) -1 else 1)
                turn *= (if (isMirrored) -1 else 1)
                turn = turn.coerceIn(-1.0, 1.0)

                println("Actual Heading: $actualHeading, Desired Heading: $desiredHeading, Turn: $turn")
                DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, leftOutput + turn, rightOutput - turn, squaredInputs = false)
            }
        }
    }

    override fun initialize() {
        DriveSubsystem.resetEncoders()


        DriveSubsystem.falconDrive.leftMotors.forEach {
            it.inverted = robotReversed
            it.setSensorPhase(!DriveConstants.IS_RACE_ROBOT)
        }
        DriveSubsystem.falconDrive.rightMotors.forEach {
            it.inverted = !robotReversed
            it.setSensorPhase(!DriveConstants.IS_RACE_ROBOT)
        }

        startTime = Timer.getFPGATimestamp()
        stopNotifier = false
        notifier.startPeriodic(DriveConstants.MOTION_DT)
    }

    override fun end() {
        println("has ended")

        synchronized(syncNotifier) {
            stopNotifier = true
            notifier.stop()
            DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)

            DriveSubsystem.falconDrive.leftMotors.forEach { it.inverted = false }
            DriveSubsystem.falconDrive.rightMotors.forEach { it.inverted = true }
        }
    }

    override fun isFinished() = (Timer.getFPGATimestamp() - startTime!!) > (mpTime - 0.1)
}
