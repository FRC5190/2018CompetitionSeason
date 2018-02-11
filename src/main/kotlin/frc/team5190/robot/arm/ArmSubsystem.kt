package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.util.*

object ArmSubsystem : Subsystem() {

    private val currentBuffer =  CircularBuffer(25)
    private val masterArmMotor = TalonSRX(MotorIDs.ARM)

    val currentPosition
        get() = masterArmMotor.getSelectedSensorPosition(0)

    val closedLoopError
        get() = masterArmMotor.getClosedLoopError(0)

    private val motorAmperage
        get() = masterArmMotor.outputCurrent

    private var state = MotorState.OK
    private var time = 0.0
    private var stalled = false

    init {
        // hardware for this subsystem includes one motor and an absolute encoder
        masterArmMotor.inverted = false
        masterArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10)
        masterArmMotor.setSensorPhase(false)
        masterArmMotor.configReverseSoftLimitEnable(false, 10)
        masterArmMotor.configReverseSoftLimitThreshold(ArmPosition.DOWN.ticks, 10)

        // break mode
        masterArmMotor.setNeutralMode(NeutralMode.Brake)

        // closed loop configuration
        masterArmMotor.configPID(ArmConstants.PID_SLOT, ArmConstants.P, ArmConstants.I, ArmConstants.D, 10)
        masterArmMotor.configNominalOutput(ArmConstants.NOMINAL_OUT, -ArmConstants.NOMINAL_OUT, 10)
        masterArmMotor.configPeakOutput(1.0, -ArmConstants.PEAK_OUT, 10)
        masterArmMotor.configAllowableClosedloopError(0, ArmConstants.TOLERANCE, 10)

        // motion magic settings
        masterArmMotor.configMotionCruiseVelocity(ArmConstants.MOTION_VELOCITY, 10)
        masterArmMotor.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION, 10)

        currentBuffer.configureForTalon(20, 5, 500)
    }

    fun set(controlMode: ControlMode, output: Double) {
        masterArmMotor.set(controlMode, output)
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ManualArmCommand()
    }

    override fun periodic() {

        currentBuffer.add(masterArmMotor.outputCurrent)
        state = masterArmMotor.limitCurrent(currentBuffer)

        time = Timer.getFPGATimestamp()

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
            MotorState.GOOD -> masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT, -ArmConstants.PEAK_OUT, 10)
        }

        SmartDashboard.putNumber("Out Current", masterArmMotor.outputCurrent)
        SmartDashboard.putNumber("Out Percent", masterArmMotor.motorOutputPercent)
    }
}

enum class ArmPosition (val ticks: Int) {
    BEHIND(1950), UP(1950), MIDDLE(1150), DOWN(900);
}