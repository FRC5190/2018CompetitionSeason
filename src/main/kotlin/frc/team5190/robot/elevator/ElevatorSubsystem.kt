package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.MainXbox
import frc.team5190.robot.arm.*
import frc.team5190.robot.getTriggerPressed
import frc.team5190.robot.util.*

object ElevatorSubsystem : Subsystem() {

    private val currentBuffer = CircularBuffer(50)
    private val masterElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_MASTER)

    val isElevatorAtBottom
        get() = masterElevatorMotor.sensorCollection.isRevLimitSwitchClosed

    val currentPosition
        get() = masterElevatorMotor.sensorCollection.quadraturePosition


    internal var hasReset = false

    private var state = MotorState.OK
    private var currentCommandGroup: CommandGroup? = null
    private var stalled = false

    init {
        masterElevatorMotor.inverted = false
        masterElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, ElevatorConstants.PID_SLOT, 10)
        masterElevatorMotor.setSensorPhase(false)
        masterElevatorMotor.configLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 10)
        masterElevatorMotor.overrideLimitSwitchesEnable(true)

        val slaveElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_SLAVE)
        slaveElevatorMotor.inverted = true
        slaveElevatorMotor.follow(masterElevatorMotor)

        // brake mode
        masterElevatorMotor.setNeutralMode(NeutralMode.Brake)
        slaveElevatorMotor.setNeutralMode(NeutralMode.Brake)

        // Closed loop operation and output shaping
        masterElevatorMotor.configPID(ElevatorConstants.PID_SLOT, ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, 10)
        masterElevatorMotor.configAllowableClosedloopError(ElevatorConstants.PID_SLOT, inchesToNativeUnits(ElevatorConstants.TOLERANCE_INCHES), 10)

        masterElevatorMotor.configNominalOutput(ElevatorConstants.NOMINAL_OUT, -ElevatorConstants.NOMINAL_OUT, 10)
        masterElevatorMotor.configPeakOutput(ElevatorConstants.PEAK_OUT, -ElevatorConstants.PEAK_OUT, 10)

        // motion magic settings
        masterElevatorMotor.configMotionCruiseVelocity(ElevatorConstants.MOTION_VELOCITY, 10)
        masterElevatorMotor.configMotionAcceleration(inchesToNativeUnits(ElevatorConstants.MOTION_ACCELERATION_INCHES) / 10, 10)

        currentBuffer.configureForTalon(5,30, 1000)

        // more settings
        reset()
    }

    private fun reset() {
        // these cannot be in the constructor since the status frame periods are reset every time the talon is reset
        masterElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10)
        masterElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10)
    }


    fun set(controlMode: ControlMode, output: Double) {
        masterElevatorMotor.set(controlMode, output)
    }

    private fun resetEncoders() = masterElevatorMotor.setSelectedSensorPosition(0, ElevatorConstants.PID_SLOT, 10)

    override fun initDefaultCommand() {
        this.defaultCommand = ManualElevatorCommand()
    }

    override fun periodic() {

        currentBuffer.add(masterElevatorMotor.outputCurrent)
        state = masterElevatorMotor.limitCurrent(currentBuffer)



        when (state) {
            MotorState.OK -> {
                if (stalled) {
                    masterElevatorMotor.configPeakOutput(ElevatorConstants.PEAK_OUT * ElevatorConstants.LIMITING_REDUCTION_FACTOR, -ElevatorConstants.PEAK_OUT * ElevatorConstants.LIMITING_REDUCTION_FACTOR, 10)
                } else {
                    masterElevatorMotor.configPeakOutput(ElevatorConstants.PEAK_OUT, -ElevatorConstants.PEAK_OUT, 10)
                }
            }
            MotorState.STALL -> {
                masterElevatorMotor.configPeakOutput(ElevatorConstants.PEAK_OUT * ElevatorConstants.LIMITING_REDUCTION_FACTOR, -ElevatorConstants.PEAK_OUT * ElevatorConstants.LIMITING_REDUCTION_FACTOR, 10)
                stalled = true
            }
            MotorState.GOOD -> masterElevatorMotor.configPeakOutput(ElevatorConstants.PEAK_OUT, -ElevatorConstants.PEAK_OUT, 10)
        }

        SmartDashboard.putNumber("Motor Amps", masterElevatorMotor.outputCurrent)
        SmartDashboard.putNumber("Motor Out", masterElevatorMotor.motorOutputPercent)

        if (this.isElevatorAtBottom) {
            this.resetEncoders()
        }
        when {
            MainXbox.getTriggerPressed(GenericHID.Hand.kRight) || MainXbox.getBumper(GenericHID.Hand.kRight) -> this.defaultCommand.start()
        }
        when (MainXbox.pov) {
        // Up - Scale
            0 -> commandGroup {
                addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
            }
        // Right - Switch
            90 -> commandGroup {
                // Just incase its in the behind position
                if (ArmSubsystem.currentPosition > ArmPosition.UP.ticks - 100)
                    addSequential(AutoArmCommand(ArmPosition.MIDDLE))
                addSequential(commandGroup {
                    addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                    addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                })
            }
        // Down - Intake
            180 -> commandGroup {
                // Just incase its in the behind position
                if (ArmSubsystem.currentPosition > ArmPosition.UP.ticks - 100)
                    addSequential(AutoArmCommand(ArmPosition.MIDDLE))
                addSequential(commandGroup {
                    addParallel(AutoArmCommand(ArmPosition.DOWN))
                    addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                })
            }
        // Left - Scale Backwards
            270 -> commandGroup {
                addSequential(commandGroup {
                    addParallel(AutoArmCommand(ArmPosition.UP))
                    addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                })
                // Go behind once we know its all the way up
                addSequential(AutoArmCommand(ArmPosition.BEHIND))
            }
            else -> null
        }?.let {
            currentCommandGroup?.cancel()
            currentCommandGroup = it
            it.start()
        }
    }

    fun nativeUnitsToInches(nativeUnits: Int) = Maths.nativeUnitsToFeet(nativeUnits, ElevatorConstants.SENSOR_UNITS_PER_ROTATION, 1.3 / 2.0) * 12.0
    fun inchesToNativeUnits(inches: Double) = Maths.feetToNativeUnits(inches / 12.0, ElevatorConstants.SENSOR_UNITS_PER_ROTATION, 1.3 / 2.0)
}

enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(24.0)),
    SCALE(ElevatorSubsystem.inchesToNativeUnits(60.0)),
    SCALE_UP(ElevatorSubsystem.inchesToNativeUnits(60.0)),
    INTAKE((0.35 * ElevatorConstants.SENSOR_UNITS_PER_ROTATION).toInt())
}