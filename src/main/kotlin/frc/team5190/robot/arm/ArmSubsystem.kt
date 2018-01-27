package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.*


object ArmSubsystem : Subsystem() {

    private val masterArmMotor = TalonSRX(MotorIDs.ARM)

    init {
        masterArmMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10)

        masterArmMotor.configNominalOutput(0.0, 0.0, 10)
        masterArmMotor.configPeakOutput(0.4, -0.4, 10)

        // TODO Find values.
        masterArmMotor.configPID(0, 0.05, 0.0, 0.0, 10)

    }
    val currentPosition
        get() = masterArmMotor.getSelectedSensorPosition(0)

    val closedLoopError
        get() = masterArmMotor.getClosedLoopError(0)

    fun set(controlMode: ControlMode, output: Double) {
        masterArmMotor.set(controlMode, output)
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ManualArmCommand()
    }
}