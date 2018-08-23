package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.lib.extensions.l
import frc.team5190.lib.extensions.r
import frc.team5190.lib.math.geometry.Pose2dWithCurvature
import frc.team5190.lib.math.trajectory.Trajectory
import frc.team5190.lib.math.trajectory.TrajectoryUtil
import frc.team5190.lib.math.trajectory.followers.NonLinearController
import frc.team5190.lib.math.trajectory.followers.TrajectoryFollower
import frc.team5190.lib.math.trajectory.timing.TimedState
import frc.team5190.lib.math.units.FeetPerSecond
import frc.team5190.robot.Kinematics
import frc.team5190.robot.Localization
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.util.DriveConstants
import kotlin.math.sign

class MotionProfileCommand2(val trajectory: Trajectory<TimedState<Pose2dWithCurvature>>, mirrored: Boolean) : Command() {

    private val trajectoryFollower: TrajectoryFollower

    private val syncNotifier = Object()
    private var stopNotifier = false
    private val notifier: Notifier

    private var lastVelocity = 0.0 to 0.0

    init {
        val finalTrajectory = if (mirrored) {
            TrajectoryUtil.mirrorTimed(trajectory)
        } else {
            trajectory
        }
        trajectoryFollower = NonLinearController(finalTrajectory)

        notifier = Notifier {
            synchronized(syncNotifier) {
                if (stopNotifier) {
                    println("Oof MotionProfile Notifier still running!")
                    return@Notifier
                }

                val position = Localization.robotPosition
                val kinematics = trajectoryFollower.getSteering(position)
                val output = Kinematics.inverseKinematics(kinematics)

                val lVelocitySTU = FeetPerSecond(output.l).STU.toDouble()
                val rVelocitySTU = FeetPerSecond(output.r).STU.toDouble()

                val lAccelerationSTU = FeetPerSecond((output.l - lastVelocity.l) * 50).STU / 1000.0 // Why CTRE
                val rAccelerationSTU = FeetPerSecond((output.r - lastVelocity.r) * 50).STU / 1000.0 // Why CTRE

                DriveSubsystem.falconDrive.setTrajectoryVelocity(Output(
                        lSetpoint = lVelocitySTU, lAdditiveFF = DriveConstants.A_HIGH * lAccelerationSTU + DriveConstants.S_HIGH * sign(lVelocitySTU),
                        rSetpoint = rVelocitySTU, rAdditiveFF = DriveConstants.A_HIGH * rAccelerationSTU + DriveConstants.S_HIGH * sign(rVelocitySTU)
                ))

                updateDashboard()

                lastVelocity = output
            }
        }
    }

    override fun initialize() {
        stopNotifier = false
        notifier.startPeriodic(0.02)
    }

    override fun end() {
        synchronized(syncNotifier) {
            stopNotifier = true
            notifier.stop()
            DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
        }
    }

    override fun isFinished(): Boolean {
        return trajectoryFollower.isFinished
    }

    private fun updateDashboard() {
        pathX = trajectoryFollower.pose.translation.x
        pathY = trajectoryFollower.pose.translation.y
        pathHdg = trajectoryFollower.pose.rotation.radians

        lookaheadX = pathX
        lookaheadY = pathY
    }


    companion object {
        var pathX = 0.0
            private set
        var pathY = 0.0
            private set
        var pathHdg = 0.0
            private set

        var lookaheadX = 0.0
            private set
        var lookaheadY = 0.0
            private set
    }

    class Output(val lSetpoint: Double, val rSetpoint: Double, val lAdditiveFF: Double, val rAdditiveFF: Double)
}