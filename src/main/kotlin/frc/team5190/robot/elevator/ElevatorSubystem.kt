package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal
import com.ctre.phoenix.motorcontrol.LimitSwitchSource
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.util.*

object ElevatorSubsystem : Subsystem() {

    private val masterElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_MASTER)

    init {
        val slaveElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_SLAVE)

        masterElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)

        masterElevatorMotor.setSensorPhase(false)
        slaveElevatorMotor.inverted = true
        slaveElevatorMotor.follow(masterElevatorMotor)
        masterElevatorMotor.configLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 10)
        masterElevatorMotor.overrideLimitSwitchesEnable(true)

        masterElevatorMotor.configNominalOutput(0.0, 0.0, 10)
        masterElevatorMotor.configPeakOutput(1.0, -0.5, 10)

        masterElevatorMotor.config_kPID(0, 0.5, 0.01, 6.0, 10)     // 0.03, 0.001, 6.0

        masterElevatorMotor.configAllowableClosedloopError(0, inchesToNativeUnits(0.25), 10) //500
    }

    fun set(output: Number) = set(ControlMode.PercentOutput, output)

    fun set(controlMode: ControlMode = ControlMode.PercentOutput, output: Number) {
        masterElevatorMotor.set(controlMode, output.toDouble())
    }

    val position
        get() = masterElevatorMotor.sensorCollection.quadraturePosition

    val closedLoopErrorInches
        get() = nativeUnitsToInches(masterElevatorMotor.getClosedLoopError(0))

    fun nativeUnitsToInches(nativeUnits: Int) = Maths.nativeUnitsToFeet(nativeUnits, 1440, 1.3 / 2.0) * 12.0
    fun inchesToNativeUnits(inches: Double) = Maths.feetToNativeUnits(inches / 12.0, 1440, 1.3 / 2.0)

    fun resetEncoders() {
        masterElevatorMotor.setSelectedSensorPosition(0, 0, 10)
    }

    override fun periodic() {
        // TODO maybe combine the Manual code and this somehow
        println("ERROR $closedLoopErrorInches")
        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) || MainXbox.getBumper(GenericHID.Hand.kRight) -> ManualElevatorCommand().start()
        }
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ManualElevatorCommand()
    }
}