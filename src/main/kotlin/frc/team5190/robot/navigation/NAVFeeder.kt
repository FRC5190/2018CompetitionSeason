package frc.team5190.robot.navigation

import com.ctre.phoenix.motion.MotionProfileStatus
import com.ctre.phoenix.motion.SetValueMotionProfile
import com.ctre.phoenix.motion.TrajectoryPoint
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Notifier

class NAVFeeder(constTalon: TalonSRX, motorSide: Side) {

    private var talon = constTalon

    private var status = MotionProfileStatus()
    private var state = 0
    private var loopTimeout = -1
    private var start = false
    private var setValue = SetValueMotionProfile.Disable

    private var side = motorSide

    private val minPointsInTalon = 5
    private val numLoopsTimeout = 10

    internal inner class PeriodicRunnable : java.lang.Runnable {
        override fun run() {
            talon.processMotionProfileBuffer()
        }
    }

    private val notifier: Notifier = Notifier(PeriodicRunnable())

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

        if (loopTimeout < 0 ) {
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
        when (side) {
            Side.LEFT -> startFilling(NAVHelper.leftPoints, NAVHelper.numPoints)
            Side.RIGHT -> startFilling(NAVHelper.rightPoints, NAVHelper.numPoints)
        }
    }

    private fun startFilling(profile : Array<Array<Double>>, totalCnt : Int) {
        val point = TrajectoryPoint()

        if (status.hasUnderrun) {
            talon.clearMotionProfileHasUnderrun(0)
        }

        talon.clearMotionProfileTrajectories()

        for (a in 0 until totalCnt) {
            point.position = profile[a][0]
            point.velocity = profile[a][1]
            // point.timeDurMs = profile[a][2]

            point.profileSlotSelect = 0
            // point.velocityOnly = false

            point.zeroPos = false
            if ((a + 1) == totalCnt) {
                point.isLastPoint = true
            }

            talon.pushMotionProfileTrajectory(point)
        }
    }

    fun startMotionProfile() {
        start = true
    }

    fun getSetValue(): SetValueMotionProfile {
        return setValue
    }

    enum class Side {
        LEFT, RIGHT
    }
}