package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.util.*

object ElevatorSubsystem : Subsystem() {

    private val masterElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_MASTER)

    init {
        val slaveElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_SLAVE)

        masterElevatorMotor.setSensorPhase(true)

        masterElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)
        masterElevatorMotor.inverted = true
        slaveElevatorMotor.inverted = false

        slaveElevatorMotor.follow(masterElevatorMotor)
        masterElevatorMotor.configLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 10)
        masterElevatorMotor.overrideLimitSwitchesEnable(true)

        masterElevatorMotor.configNominalOutput(0.0, 0.0, 10)
        masterElevatorMotor.configPID(0, 0.8, 0.0, 0.0, 10)
        masterElevatorMotor.configPeakOutput(0.85, -0.85, 10)
        masterElevatorMotor.configAllowableClosedloopError(0, inchesToNativeUnits(0.25), 10) //500

        masterElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10)
        masterElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10)

//        masterElevatorMotor.configOpenloopRamp(1.5, 10)

        masterElevatorMotor.configMotionCruiseVelocity(1000000000, 10)
        masterElevatorMotor.configMotionAcceleration(inchesToNativeUnits(80.0) / 10, 10)
    }

    fun isElevatorAtBottom() = masterElevatorMotor.sensorCollection.isRevLimitSwitchClosed

    fun set(controlMode: ControlMode, output: Number) {
        masterElevatorMotor.set(controlMode, output.toDouble())
    }

    val currentPosition
        get() = masterElevatorMotor.sensorCollection.quadraturePosition

    val closedLoopErrorInches: Double
        get() {
            val error = nativeUnitsToInches(masterElevatorMotor.getClosedLoopError(0))
            println("ERROR: $error")
            return error
        }

    fun resetEncoders() = masterElevatorMotor.setSelectedSensorPosition(0, 0, 10)!!

    override fun periodic() {
        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) || MainXbox.getBumper(GenericHID.Hand.kRight) -> this.defaultCommand.start()
        }
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ManualElevatorCommand()
    }

    fun nativeUnitsToInches(nativeUnits: Int) = Maths.nativeUnitsToFeet(nativeUnits, 1440, 1.3 / 2.0) * 12.0
    fun inchesToNativeUnits(inches: Double) = Maths.feetToNativeUnits(inches / 12.0, 1440, 1.3 / 2.0)
}