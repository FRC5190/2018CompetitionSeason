package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal
import com.ctre.phoenix.motorcontrol.LimitSwitchSource
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox

object ElevatorSubsystem : Subsystem() {

    private val masterElevatorMotor = TalonSRX(20)

    init {
        val slaveElevatorMotor = TalonSRX(21)

        masterElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)

        masterElevatorMotor.setSensorPhase(false)
        slaveElevatorMotor.inverted = true
        slaveElevatorMotor.follow(masterElevatorMotor)
        masterElevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 10)
        masterElevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 10)
        masterElevatorMotor.overrideLimitSwitchesEnable(true)

        masterElevatorMotor.configNominalOutputForward(0.85 ,10)
        masterElevatorMotor.configPeakOutputForward(0.85, 10)
        masterElevatorMotor.configNominalOutputReverse(-0.2, 10)
        masterElevatorMotor.configPeakOutputReverse(-0.2, 10)

        masterElevatorMotor.config_kP(0, 0.04, 10)     // 0.03
        masterElevatorMotor.config_kI(0, 0.001, 10)    // 0.001
        masterElevatorMotor.config_kD(0, 6.0, 10)      // 6.0


        masterElevatorMotor.configAllowableClosedloopError(0, 500, 10) //500


    }

    fun moveToPosition(position: Int) {
        masterElevatorMotor.set(ControlMode.Position, position.toDouble())
    }

    fun set(output: Double) {
        masterElevatorMotor.set(ControlMode.PercentOutput, output)
    }

    fun resetEncoders() {
        masterElevatorMotor.setSelectedSensorPosition(0, 0, 10)
    }

    override fun periodic() {
        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) ||  MainXbox.getBumper(GenericHID.Hand.kRight) -> ElevatorCommand().start()
        }
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ElevatorCommand()
    }
}