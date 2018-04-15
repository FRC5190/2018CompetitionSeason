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
import frc.team5190.robot.sensors.Pigeon
import frc.team5190.robot.util.DriveConstants
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.followers.EncoderFollower

open class MotionProfileCommand(folder: String, file: String,
                                private val robotReversed: Boolean = false, private val pathReversed: Boolean = false,
                                private val pathMirrored: Boolean = false, private val useGyro: Boolean = true) : Command() {

    companion object {
        var robotPosition: Pair<Double, Double>? = null
    }

    private val syncNotifier = Object()
    private var stopNotifier = false

    private val leftPath: Trajectory
    private val rightPath: Trajectory

    private lateinit var leftEncoderFollower: EncoderFollower
    private lateinit var rightEncoderFollower: EncoderFollower

    private val currentRobotPosition: Pair<Double, Double>?
        get() {
            if(leftEncoderFollower.isFinished || rightEncoderFollower.isFinished) return null
            val x1 = leftEncoderFollower.segment.x
            val y1 = leftEncoderFollower.segment.y
            val x2 = rightEncoderFollower.segment.x
            val y2 = rightEncoderFollower.segment.y
            return (x1 + x2) / 2.0 to ((y1 + y2) / 2.0).let {
                if(pathMirrored) 27.0 - it
                else it
            }
        }

    val pathDuration
        get() = leftPath.length() * DriveConstants.MOTION_DT

    private lateinit var notifier: Notifier

    private var startTime: Double? = null

    init {
        val trajectories = Pathreader.getPath(folder, file)
        leftPath = trajectories[0].let { if (pathReversed) reverseTrajectory(it) else it }
        rightPath = trajectories[1].let { if (pathReversed) reverseTrajectory(it) else it }

        this.requires(DriveSubsystem)

        val leftTrajectory = if (pathMirrored) rightPath else leftPath
        val rightTrajectory = if (pathMirrored) leftPath else rightPath

        val robotReversedMul = if (robotReversed) -1 else 1

        leftEncoderFollower = EncoderFollower(if (robotReversed xor pathReversed) rightTrajectory else leftTrajectory).apply {
            configureEncoder(0, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS / 6.0)
            configurePIDVA(DriveConstants.P_HIGH, DriveConstants.I_HIGH, DriveConstants.D_HIGH, 1 / 15.0, 0.0)
        }

        rightEncoderFollower = EncoderFollower(if (robotReversed xor pathReversed) leftTrajectory else rightTrajectory).apply {
            configureEncoder(0, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS / 6.0)
            configurePIDVA(DriveConstants.P_HIGH, DriveConstants.I_HIGH, DriveConstants.D_HIGH, 1 / 15.0, 0.0)
        }

        notifier = Notifier {
            synchronized(syncNotifier) {
                if (stopNotifier) {
                    println("Oof MotionProfile Notifier still running!")
                    return@Notifier
                }

                val leftOutput = leftEncoderFollower.calculate(DriveSubsystem.falconDrive.leftEncoderPosition * robotReversedMul).coerceIn(if(DriveConstants.IS_RACE_ROBOT) -0.1 else -0.2, 1.0) * robotReversedMul
                val rightOutput = rightEncoderFollower.calculate(DriveSubsystem.falconDrive.rightEncoderPosition * robotReversedMul).coerceIn(if(DriveConstants.IS_RACE_ROBOT) -0.1 else -0.2, 1.0) * robotReversedMul

                val actualHeading = Pathfinder.boundHalfDegrees((Pigeon.correctedAngle + if (robotReversed xor pathReversed) 180 else 0))
                val desiredHeading = (if (pathMirrored) -1 else 1) * Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftEncoderFollower.heading))

                val angleDifference = Pathfinder.boundHalfDegrees(actualHeading - desiredHeading)

                var turn = 0.75 * (1 / 80.0) * angleDifference
                turn = if (useGyro) turn else 0.0

                DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, leftOutput + turn, rightOutput - turn, squaredInputs = false)
                robotPosition = currentRobotPosition
            }
        }

    }

    private fun reverseTrajectory(trajectory: Trajectory): Trajectory {
        val newTrajectory = trajectory.copy()
        val distance = newTrajectory.segments.last().position
        newTrajectory.segments.reverse()
        newTrajectory.segments.forEach {
            it.position = distance - it.position
        }
        return newTrajectory
    }

    override fun initialize() {
        DriveSubsystem.resetEncoders()


        startTime = Timer.getFPGATimestamp()
        stopNotifier = false
        notifier.startPeriodic(DriveConstants.MOTION_DT)
    }

    override fun end() {
        synchronized(syncNotifier) {
            stopNotifier = true
            notifier.stop()
            robotPosition = null
            DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
        }
    }

    override fun isFinished() = leftEncoderFollower.isFinished && rightEncoderFollower.isFinished
}
