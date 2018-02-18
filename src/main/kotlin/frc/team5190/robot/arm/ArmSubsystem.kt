package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.*

object ArmSubsystem : Subsystem() {

    private val masterArmMotor = TalonSRX(MotorIDs.ARM)
    private val currentBuffer = CircularBuffer(25)

    private var stalled = false
    private var state = MotorState.OK

    val motorAmps
        get() = masterArmMotor.outputCurrent

    val currentPosition
        get() = masterArmMotor.getSelectedSensorPosition(0)

    init {
        // hardware for this subsystem includes one motor and an absolute encoder
        masterArmMotor.inverted = ArmConstants.INVERTED
        masterArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10)
        masterArmMotor.setSensorPhase(ArmConstants.SENSOR_PHASE)
        masterArmMotor.configReverseSoftLimitEnable(false, 10)
        masterArmMotor.configReverseSoftLimitThreshold(ArmPosition.DOWN.ticks, 10)

        // break mode
        masterArmMotor.setNeutralMode(NeutralMode.Brake)

        // current limiting
        currentBuffer.configureForTalon(ArmConstants.LOW_PEAK, ArmConstants.HIGH_PEAK, ArmConstants.DUR)

        // closed loop configuration
        masterArmMotor.configPID(ArmConstants.PID_SLOT, ArmConstants.P, ArmConstants.I, ArmConstants.D, 10)
        masterArmMotor.configNominalOutput(ArmConstants.NOMINAL_OUT, -ArmConstants.NOMINAL_OUT, 10)
        masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT, -ArmConstants.PEAK_OUT, 10)
        masterArmMotor.configAllowableClosedloopError(0, ArmConstants.TOLERANCE, 10)

        // motion magic settings
        masterArmMotor.configMotionCruiseVelocity(ArmConstants.MOTION_VELOCITY, 10)
        masterArmMotor.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION, 10)
    }

    fun set(controlMode: ControlMode, output: Double) {
        masterArmMotor.set(controlMode, output)
    }

    private fun currentLimiting() {
        currentBuffer.add(masterArmMotor.outputCurrent)
        state = masterArmMotor.limitCurrent(currentBuffer)

        when (state) {
            MotorState.OK -> {
                if (stalled) {
                    masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, -ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, 10)
                } else {
                    masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT, -ArmConstants.PEAK_OUT, 10)
                }
            }
            MotorState.STALL -> {
                masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, -ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, 10)
                stalled = true
            }
            MotorState.GOOD -> {
                masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT, -ArmConstants.PEAK_OUT, 10)
                stalled = false
            }
        }
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ManualArmCommand()
    }

    override fun periodic() {
        this.currentLimiting()
    }
}

enum class ArmPosition (val ticks: Int) {
    BEHIND(ArmConstants.DOWN_TICKS + 1300),
    UP(ArmConstants.DOWN_TICKS + 800),
    MIDDLE(ArmConstants.DOWN_TICKS + 150),
    DOWN(ArmConstants.DOWN_TICKS);
}