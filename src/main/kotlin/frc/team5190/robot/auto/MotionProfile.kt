/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.auto

import com.ctre.phoenix.motion.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.pathfinder.MotionProfileTrajectory

/**
 * Class that feeds the talon trajectory values to follow.
 */
class MotionProfile(private var leftTalon: TalonSRX, leftTrajectory: MotionProfileTrajectory, private var rightTalon: TalonSRX, rightTrajectory: MotionProfileTrajectory, isReversed: Boolean) {

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
    private var leftTrajectory: MotionProfileTrajectory
    private var rightTrajectory: MotionProfileTrajectory

    // Notifier
    private val leftNotifier = Notifier(leftTalon::processMotionProfileBuffer)
    private val rightNotifier = Notifier(rightTalon::processMotionProfileBuffer)

    // Boolean value to check if the motion profile has triggerState
    private var isFinished = false

    /**
     * Called when the class is instantiated
     */
    init {
        leftTalon.changeMotionControlFramePeriod(5)
        rightTalon.changeMotionControlFramePeriod(5)

        leftNotifier.startPeriodic(0.005)
        rightNotifier.startPeriodic(0.005)

        DriveSubsystem.falconDrive.leftMotors.forEach {
            it.inverted = isReversed
            it.setSensorPhase(true)
        }
        DriveSubsystem.falconDrive.rightMotors.forEach {
            it.inverted = !isReversed
            it.setSensorPhase(true)
        }

        if (isReversed) {
            this.leftTrajectory = rightTrajectory
            this.rightTrajectory = leftTrajectory
        } else {
            this.leftTrajectory = leftTrajectory
            this.rightTrajectory = rightTrajectory
        }
    }

    /**
     * Resets the trajectory when MP ends or is canceled.
     */
    fun reset() {
        leftTalon.clearMotionProfileTrajectories()
        rightTalon.clearMotionProfileTrajectories()

        setValue = SetValueMotionProfile.Disable
        state = 0
        loopTimeout = -1
        start = false
        isFinished = false
    }

    /**
     * Called periodically and is used to control the behavior of the MP
     */
    fun control() {
        leftTalon.getMotionProfileStatus(status)
        rightTalon.getMotionProfileStatus(status)

        if (loopTimeout < 0) {
        } else {
            if (loopTimeout == 0) {
                // TODO
            } else {
                loopTimeout--
            }
        }

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
                    isFinished = true
                }
            }
        }

        leftTalon.getMotionProfileStatus(status)
        rightTalon.getMotionProfileStatus(status)

    }

    /**
     * Fills the talon with the trajectory points.
     */
    private fun startFilling() {
        val leftPoint = TrajectoryPoint()
        val rightPoint = TrajectoryPoint()

        if (status.hasUnderrun) {
            leftTalon.clearMotionProfileHasUnderrun(0)
            rightTalon.clearMotionProfileHasUnderrun(0)
        }

        leftTalon.clearMotionProfileTrajectories()
        rightTalon.clearMotionProfileTrajectories()

        leftTalon.configMotionProfileTrajectoryPeriod(0, 10)
        rightTalon.configMotionProfileTrajectoryPeriod(0, 10)

        leftTrajectory.forEachIndexed { index, tPoint ->
            leftPoint.position = tPoint.nativeUnits
            leftPoint.velocity = tPoint.nativeUnitsPer100Ms

            leftPoint.headingDeg = 0.0
            leftPoint.profileSlotSelect0 = 0
            leftPoint.profileSlotSelect1 = 0
            leftPoint.timeDur = getTrajectoryDuration(tPoint.duration)

            leftPoint.zeroPos = index == 0
            leftPoint.isLastPoint = index + 1 == leftTrajectory.size

            leftTalon.pushMotionProfileTrajectory(leftPoint)
        }

        rightTrajectory.forEachIndexed { index, tPoint ->
            rightPoint.position = tPoint.nativeUnits
            rightPoint.velocity = tPoint.nativeUnitsPer100Ms

            rightPoint.headingDeg = 0.0
            rightPoint.profileSlotSelect0 = 0
            rightPoint.profileSlotSelect0 = 0
            rightPoint.timeDur = getTrajectoryDuration(tPoint.duration)

            rightPoint.zeroPos = index == 0
            rightPoint.isLastPoint = index + 1 == rightTrajectory.size

            rightTalon.pushMotionProfileTrajectory(rightPoint)
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
     * Getter for the set value property.
     */
    fun getSetValue() = setValue

    /**
     * Returns if the MP has triggerState.
     */
    fun hasFinished() = isFinished
}