package frc.team5190.robot.auto

import com.ctre.phoenix.motion.MotionProfileStatus
import com.ctre.phoenix.motion.SetValueMotionProfile
import com.ctre.phoenix.motion.TrajectoryPoint
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Notifier

class MotionProfileManager(private val _talon: TalonSRX) {

    private val _status = MotionProfileStatus()
    private var _state = 0
    private var _loopTimeout = -1

    private var _bStart = false
    private var trajectoryList: TrajectoryList = emptyList()

    var setValue = SetValueMotionProfile.Disable
        private set
    private var _notifer = Notifier(_talon::processMotionProfileBuffer)

    init {
        _talon.changeMotionControlFramePeriod(5)
        _notifer.startPeriodic(0.005)
    }

    constructor(_talon: TalonSRX, trajectories: TrajectoryList) : this(_talon) {
        startMotionProfile(trajectories)
    }

    fun reset() {
        _talon.clearMotionProfileTrajectories()
        setValue = SetValueMotionProfile.Disable
        _state = 0
        _loopTimeout = -1
        _bStart = false
    }

    fun control() {
        _talon.getMotionProfileStatus(_status)
        if (_loopTimeout < 0) {
            /* do nothing, timeout is disabled */
        } else {
            if (_loopTimeout == 0) {
                println("[MotionProfile] something is wrong. Talon is not present, unplugged, breaker tripped")
            } else {
                --_loopTimeout
            }
        }

        if (_talon.controlMode != ControlMode.MotionProfile) {
            _state = 0
            _loopTimeout = -1
        } else {
            when (_state) {
                0 -> if (_bStart) {
                    _bStart = false
                    setValue = SetValueMotionProfile.Disable
                    startFilling()
                    _state = 1
                    _loopTimeout = kNumLoopsTimeout
                }
                1 -> if (_status.btmBufferCnt > kMinPointsInTalon) {
                    setValue = SetValueMotionProfile.Enable
                    _state = 2
                    _loopTimeout = kNumLoopsTimeout
                }
                2 -> {
                    if (!_status.isUnderrun)
                        _loopTimeout = kNumLoopsTimeout

                    if (_status.activePointValid && _status.isLast) {
                        setValue = SetValueMotionProfile.Hold
                        _state = 0
                        _loopTimeout = -1
                    }
                }
            }
        }
    }

    private fun startFilling() {
        val point = TrajectoryPoint()

        if (_status.hasUnderrun) {
            println("[MotionProfile] Underrun")
            _talon.clearMotionProfileHasUnderrun(0)
        }

        _talon.clearMotionProfileTrajectories()

        trajectoryList.forEachIndexed { index, trajectory ->
            /* for each point, fill our structure and pass it to API */
            point.position = trajectory.position
            point.velocity = trajectory.velocity
            point.profileSlotSelect = 0 /* which set of gains would you like to use? */
            point.zeroPos = false
            if (index == 0)
                point.zeroPos = true /* set this to true on the first point */

            point.isLastPoint = false
            if (index + 1 == trajectoryList.size)
                point.isLastPoint = true /* set this to true on the last point  */

            _talon.pushMotionProfileTrajectory(point)
        }
    }

    fun startMotionProfile(trajectories: TrajectoryList) {
        this.trajectoryList = trajectories.takeIf { it.isNotEmpty() }!!
        _bStart = true
    }

    companion object {
        private val kMinPointsInTalon = 5
        private val kNumLoopsTimeout = 10
    }
}