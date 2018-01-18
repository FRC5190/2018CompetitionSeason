/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import com.ctre.phoenix.motion.MotionProfileStatus
import com.ctre.phoenix.motion.SetValueMotionProfile
import com.ctre.phoenix.motion.TrajectoryPoint
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier

/**
 * Class that feeds the talon trajectory values to follow.
 */
class AutoPath(constTalon: TalonSRX, constTrajectory: TrajectoryList, constDetailedTrajectory: DetailedTrajectoryList, calc: Boolean = false,
               helper: AutoHelper) {

    // The talon to apply the trajectory to
    private var talon = constTalon

    // Status of the motion profile
    private var status = MotionProfileStatus()

    // Current state of the motion profile
    private var state = 0

    // Timeout
    private var loopTimeout = -1

    // Value used to determine whether to start the trajectory
    private var start = false

    // Set value of the talon
    private var setValue = SetValueMotionProfile.Disable

    // Minimum number of points in the talon before enabling
    private val minPointsInTalon = 5

    // Number of loops until timeout
    private val numLoopsTimeout = 10

    // The trajectory to load into the talon
    private var trajectory = constTrajectory

    // Notifier
    private val notifier = Notifier(talon::processMotionProfileBuffer)

    // Has been paused
    private var paused = false

    // Use this instance to calculate the new trajectory or not
    private val calculate = calc

    // Instance of the helper being used
    private val autoHelper = helper

    /**
     * Called when the class is instantiated
     */
    init {
        talon.changeMotionControlFramePeriod(5)
        notifier.startPeriodic(0.005)
        talon.setSensorPhase(true)
    }

    /**
     * Resets the trajectory when MP ends or is canceled.
     */
    fun reset() {
        talon.clearMotionProfileTrajectories()
        setValue = SetValueMotionProfile.Disable
        state = 0
        loopTimeout = -1
        start = false
    }

    /**
     * Called periodically and is used to control the behavior of the MP
     */
    fun control() {
        talon.getMotionProfileStatus(status)

        if (loopTimeout < 0) {
        } else {
            if (loopTimeout == 0) {
                // TODO
            } else {
                loopTimeout--
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
                3 -> setValue = SetValueMotionProfile.Hold
            }
            talon.getMotionProfileStatus(status)
        }
    }

    /**
     * Fills the talon with the trajectory points.
     */
    private fun startFilling() {
        val point = TrajectoryPoint()

        if (status.hasUnderrun) {
            talon.clearMotionProfileHasUnderrun(0)
        }

        talon.clearMotionProfileTrajectories()
        talon.configMotionProfileTrajectoryPeriod(0, 10)

        trajectory.forEachIndexed { index, tPoint ->
            point.position = tPoint.nativeUnits
            point.velocity = tPoint.nativeUnitsPer100Ms

            point.headingDeg = 0.0
            point.profileSlotSelect0 = 0
            point.profileSlotSelect1 = 0
            point.timeDur = getTrajectoryDuration(tPoint.duration)

            point.zeroPos = index == 0
            point.isLastPoint = index + 1 == trajectory.size

            talon.pushMotionProfileTrajectory(point)
        }
    }

    /**
     * Returns the duration of the trajectory
     * @param durationMs Duration of each trajectory point
     * @return the duration of each point in TrajectoryDuration point
     */
    private fun getTrajectoryDuration(durationMs: Int): TrajectoryPoint.TrajectoryDuration {
        var retval = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_0ms
        retval = retval.valueOf(durationMs)

        if (retval.value != durationMs) {
            DriverStation.reportError("Trajectory Duration not supported", false)
        }

        return retval
    }

    /**
     * Starts the motion profile.
     */
    fun startMotionProfile() {
        start = true
    }

    /**
     * Pauses the motion profile
     */
    fun pauseMotionProfile() {
        state = 3

        if (!paused && calculate) {
            val index = trajectory.indexOf(trajectory.find { talon.activeTrajectoryPosition > it.nativeUnits })
            paused = false
            AutoHelper.generateNewPaths(index, autoHelper)
        }
    }

    /**
     * Resumes the motion profile
     */
    fun resumeMotionProfile() {
        trajectory = if (calculate) {
            AutoHelper.newTrajectories!![0]
        } else {
            AutoHelper.newTrajectories!![1]
        }

        this.reset()
        this.startMotionProfile()
    }

    /**
     * Getter for the set value property.
     */
    fun getSetValue(): SetValueMotionProfile {
        return setValue
    }

    /**
     * Returns if the MP has finished.
     */
    fun hasFinished(): Boolean {
        return status.isLast
    }
}