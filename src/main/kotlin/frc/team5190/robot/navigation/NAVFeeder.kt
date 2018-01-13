package frc.team5190.robot.navigation

import com.ctre.phoenix.motion.MotionProfileStatus
import com.ctre.phoenix.motion.SetValueMotionProfile
import com.ctre.phoenix.motion.TrajectoryPoint
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier

class NAVFeeder(constTalon: TalonSRX, constTrajectories: TrajectoryList) {

    private var talon = constTalon

    private var status = MotionProfileStatus()
    private var state = 0
    private var loopTimeout = -1
    private var start = false
    private var setValue = SetValueMotionProfile.Disable

    private val minPointsInTalon = 5
    private val numLoopsTimeout = 10

    private val trajectories = constTrajectories

    private val notifier = Notifier(talon::processMotionProfileBuffer)

    init {
        talon.changeMotionControlFramePeriod(10)
        notifier.startPeriodic(0.01)
    }

    fun reset() {
        talon.clearMotionProfileTrajectories()
        setValue = SetValueMotionProfile.Disable
        state = 0
        loopTimeout = -1
        start = false
    }

    fun control() {
        talon.getMotionProfileStatus(status)

        if (loopTimeout < 0) {
        } else {
            if (loopTimeout == 0) {
                // TODO
            } else {
                loopTimeout--;
            }
        }

        if (talon.controlMode != ControlMode.MotionProfile) {
            state = 0
            loopTimeout = -1

        } else {
            when (state) {
                0 -> {
                    if (start) {
                        start = false
                        setValue = SetValueMotionProfile.Disable
                        startFilling()
                        state = 1
                        loopTimeout = numLoopsTimeout
                    }
                }

                1 -> {
                    if (status.btmBufferCnt > minPointsInTalon) {
                        setValue = SetValueMotionProfile.Enable
                        state = 2
                        loopTimeout = numLoopsTimeout
                    }
                }

                2 -> {
                    if (!status.isUnderrun) {
                        loopTimeout = numLoopsTimeout
                    }
                    if (status.activePointValid && status.isLast) {
                        setValue = SetValueMotionProfile.Hold
                        state = 0
                        loopTimeout = -1
                    }
                }
            }
        }
    }

    private fun startFilling() {
        val point = TrajectoryPoint()

        if (status.hasUnderrun) {
            talon.clearMotionProfileHasUnderrun(0)
        }

        talon.clearMotionProfileTrajectories()
        talon.configMotionProfileTrajectoryPeriod(0, 10)

        trajectories.forEachIndexed { index, trajectory ->

            point.position = trajectory.rotations //* Hardware.SENSOR_UNITS_PER_ROTATION
            point.velocity = trajectory.rpm //* Hardware.SENSOR_UNITS_PER_ROTATION / 600

            point.headingDeg = 0.0
            point.profileSlotSelect0 = 0
            point.profileSlotSelect1 = 0
            point.timeDur = getTrajectoryDuration(trajectory.duration.toInt())

            point.zeroPos = index == 0
            point.isLastPoint = index + 1 == trajectories.size

            // point.timeDurMs = trajectory.duration()

            talon.pushMotionProfileTrajectory(point)
        }
    }

    private fun getTrajectoryDuration(durationMs: Int): TrajectoryPoint.TrajectoryDuration {
        var retval = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_0ms
        retval = retval.valueOf(durationMs)

        if (retval.value != durationMs) {
            DriverStation.reportError("Trajectory Duration not supported", false)
        }

        return retval
    }

    fun startMotionProfile() {
        start = true
    }

    fun getSetValue(): SetValueMotionProfile {
        return setValue
    }

    fun hasFinished(): Boolean {
        return status.isLast
    }
}