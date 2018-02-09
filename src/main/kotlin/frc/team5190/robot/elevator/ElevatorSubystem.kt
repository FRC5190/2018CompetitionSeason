package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.arm.*
import frc.team5190.robot.getTriggerPressed
import frc.team5190.robot.util.*

object ElevatorSubsystem : Subsystem() {

    private val masterElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_MASTER)

    internal var hasReset = false

    init {
        // hardware for this subsystem includes two motors in master-slave config, a quad encoder, and limit switches
        masterElevatorMotor.inverted = false
        masterElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)
        masterElevatorMotor.setSensorPhase(false)
        masterElevatorMotor.configLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 10)
        masterElevatorMotor.overrideLimitSwitchesEnable(true)

        val slaveElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_SLAVE)
        slaveElevatorMotor.inverted = true
        slaveElevatorMotor.follow(masterElevatorMotor)

        // current limiting
        masterElevatorMotor.configCurrentLimiting(40, 2000, 20, 10)
        slaveElevatorMotor.configCurrentLimiting(40, 2000, 20, 10)

        // Closed loop operation and output shaping
        masterElevatorMotor.configPID(0, 0.8, 0.0, 0.0, 10)
        masterElevatorMotor.configNominalOutput(0.0, 0.0, 10)
        masterElevatorMotor.configPeakOutput(0.70, -0.70, 10)
        masterElevatorMotor.configAllowableClosedloopError(0, inchesToNativeUnits(0.25), 10) //500

        // motion magic settings
        // TODO: Fix these values
        masterElevatorMotor.configMotionCruiseVelocity(1000000000, 10)
        masterElevatorMotor.configMotionAcceleration(inchesToNativeUnits(80.0) / 10, 10)

        // more settings
        reset()
    }

    private fun reset() {
        // these cannot be in the constructor since the status frame periods are reset every time the talon is reset
        masterElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10)
        masterElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10)
    }

    fun isElevatorAtBottom() = masterElevatorMotor.sensorCollection.isRevLimitSwitchClosed

    fun set(controlMode: ControlMode, output: Number) {
        masterElevatorMotor.set(controlMode, output.toDouble())
    }

    val currentPosition
        get() = masterElevatorMotor.sensorCollection.quadraturePosition

    val motorAmperage
        get() = masterElevatorMotor.outputCurrent

    fun resetEncoders() = masterElevatorMotor.setSelectedSensorPosition(0, 0, 10)!!

    private var currentCommandGroup: CommandGroup? = null

    override fun periodic() {
        if (this.isElevatorAtBottom()) {
            this.resetEncoders()
        }
        when {
            MainXbox.getTriggerPressed(GenericHID.Hand.kRight) || MainXbox.getBumper(GenericHID.Hand.kRight) -> this.defaultCommand.start()
        }
        when(MainXbox.pov) {
            // Up - Scale
            0 -> commandGroup {
                addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
            }
            // Right - Switch
            90 -> commandGroup {
                // Just incase its in the behind position
                if(ArmSubsystem.currentPosition > ArmPosition.UP.ticks - 100)
                    addSequential(AutoArmCommand(ArmPosition.MIDDLE))
                addSequential(commandGroup {
                    addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                    addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                })
            }
            // Down - Intake
            180 -> commandGroup {
                // Just incase its in the behind position
                if(ArmSubsystem.currentPosition > ArmPosition.UP.ticks - 100)
                    addSequential(AutoArmCommand(ArmPosition.MIDDLE))
                addSequential(commandGroup {
                    addParallel(AutoArmCommand(ArmPosition.DOWN))
                    addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                })
            }
            // Left - Scale Backwards
            270 ->  commandGroup {
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

    override fun initDefaultCommand() {
        this.defaultCommand = ManualElevatorCommand()
    }

    // TODO: Why is the wheel radius 0.65 inches?
    fun nativeUnitsToInches(nativeUnits: Int) = Maths.nativeUnitsToFeet(nativeUnits, 1440, 1.3 / 2.0) * 12.0
    fun inchesToNativeUnits(inches: Double) = Maths.feetToNativeUnits(inches / 12.0, 1440, 1.3 / 2.0)
}
