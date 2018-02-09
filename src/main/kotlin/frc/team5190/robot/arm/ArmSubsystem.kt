package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.*

object ArmSubsystem : Subsystem() {

    private val masterArmMotor = TalonSRX(MotorIDs.ARM)

    init {
        // hardware for this subsystem includes one motor and an absolute encoder
        masterArmMotor.inverted = false
        masterArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10)
        masterArmMotor.setSensorPhase(false)
        masterArmMotor.configReverseSoftLimitEnable(false, 10)
        masterArmMotor.configReverseSoftLimitThreshold(ArmPosition.DOWN.ticks, 10)

        // current limiting
        masterArmMotor.configCurrentLimiting(40, 2000, 20, 10)

        // TODO: what about break mode?
        masterArmMotor.setNeutralMode(NeutralMode.Brake)

        // closed loop configuration
        // TODO: Tune constants and move them into a robot specific hardware file
        masterArmMotor.configPID(0, 1.5, 0.0, 0.0, 10)
        masterArmMotor.configNominalOutput(0.0, 0.0, 10)
        masterArmMotor.configPeakOutput(0.4, -0.4, 10)
        masterArmMotor.configAllowableClosedloopError(0, 0, 10)

        // motion magic settings
        // TODO: Fix these values
        masterArmMotor.configMotionCruiseVelocity(1000000, 10)
        masterArmMotor.configMotionAcceleration(400, 10)

        // other settings
        reset()
    }

    fun reset() {
        // nothing more to do
    }

    val currentPosition
        get() = masterArmMotor.getSelectedSensorPosition(0)

    val closedLoopError
        get() = masterArmMotor.getClosedLoopError(0)

    val armMotorAmperage
        get() = masterArmMotor.outputCurrent

    fun set(controlMode: ControlMode, output: Double) {
        masterArmMotor.set(controlMode, output)
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ManualArmCommand()
    }
}

// TODO: Move hardcoded values to Hardware module
enum class ArmPosition (val ticks: Int) {
    BEHIND(1950), // When placing scale backwards
    UP(1950), // Arm is always up, basically where it starts in auto
    MIDDLE(1150), // Angled a little up to help placement on scale and switch
    DOWN(900); // Lowest position, used for intaking the cube
}