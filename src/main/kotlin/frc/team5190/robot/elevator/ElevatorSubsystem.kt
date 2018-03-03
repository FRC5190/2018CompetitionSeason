/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

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

    // Master elevator talon
    private val masterElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_MASTER)

    // Buffer used to hold amperage values for current limiing
    private val currentBuffer = CircularBuffer(25)

    // Returns if the elevator is hitting the limit switch
    private val isElevatorAtBottom
        get() = masterElevatorMotor.sensorCollection.isRevLimitSwitchClosed

    // Returns the current encoder position of the elevator
    val currentPosition
        get() = masterElevatorMotor.sensorCollection.quadraturePosition

    // Returns the amperage of the motor
    val amperage
        get() = masterElevatorMotor.outputCurrent


    // Variables used for current limiting
    private var state = MotorState.OK
    private var currentCommandGroup: CommandGroup? = null
    private var stalled = false

    var peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT

    init {
        with(masterElevatorMotor) {
            // Motor Inversion
            inverted = false

            // Sensors and Safety
            configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, ElevatorConstants.PID_SLOT, 10)
            setSensorPhase(false)
            configLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 10)
            overrideLimitSwitchesEnable(true)
            configForwardSoftLimitThreshold(ElevatorConstants.SOFT_LIMIT_FWD, 10)
            configForwardSoftLimitEnable(true, 10)

            // Brake Mode
            setNeutralMode(NeutralMode.Brake)

            // Closed Loop Control
            configPID(ElevatorConstants.PID_SLOT, ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, 10)
            configAllowableClosedloopError(ElevatorConstants.PID_SLOT, inchesToNativeUnits(ElevatorConstants.TOLERANCE_INCHES), 10)
            configNominalOutput(ElevatorConstants.NOMINAL_OUT, -ElevatorConstants.NOMINAL_OUT, 10)
            configPeakOutput(ElevatorConstants.IDLE_PEAK_OUT, -ElevatorConstants.IDLE_PEAK_OUT, 10)

            // Motion Magic Control
            configMotionCruiseVelocity(ElevatorConstants.MOTION_VELOCITY, 10)
            configMotionAcceleration(inchesToNativeUnits(ElevatorConstants.MOTION_ACCELERATION_INCHES) / 10, 10)
            setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10)
            setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10)

            configClosedloopRamp(0.3, 10)
            configOpenloopRamp(0.5, 10)

        }

        // Configure Slave Motor
        with(TalonSRX(MotorIDs.ELEVATOR_SLAVE)) {
            inverted = true
            follow(masterElevatorMotor)
            setNeutralMode(NeutralMode.Brake)
        }

        // Configure current limiting
        currentBuffer.configureForTalon(ElevatorConstants.LOW_PEAK, ElevatorConstants.HIGH_PEAK, ElevatorConstants.DUR)
    }

    /**
     * Sets the motor output
     * @param controlMode Control Mode for the Talon
     * @param output Output to the motor
     */
    fun set(controlMode: ControlMode, output: Double) {
        masterElevatorMotor.set(controlMode, output)
    }


    /**
     * Resets encoders on the elevator
     */
    private fun resetEncoders() = masterElevatorMotor.setSelectedSensorPosition(0, ElevatorConstants.PID_SLOT, 10)!!

    /**
     * Enables current limiting on the motor so we don't stall it
     */
    private fun currentLimiting() {
        currentBuffer.add(masterElevatorMotor.outputCurrent)
        state = limitCurrent(currentBuffer)

        when (state) {
            MotorState.OK -> {
                if (stalled) {
                    masterElevatorMotor.configPeakOutput(peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, -peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, 10)
                } else {
                    masterElevatorMotor.configPeakOutput(peakElevatorOutput, -peakElevatorOutput, 10)
                }
            }
            MotorState.STALL -> {
                masterElevatorMotor.configPeakOutput(peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, -peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, 10)
                stalled = true
            }
            MotorState.GOOD -> {
                masterElevatorMotor.configPeakOutput(peakElevatorOutput, -peakElevatorOutput, 10)
                stalled = false
            }
        }
    }

    /**
     * Sets the default command for the subsystem
     */
    override fun initDefaultCommand() {
        defaultCommand = ManualElevatorCommand()
    }

    /**
     * Executed periodically
     */
    override fun periodic() {
        if (ElevatorSubsystem.isElevatorAtBottom) {
            this.resetEncoders()
        }

        SmartDashboard.putBoolean("Reset Limit Switch", isElevatorAtBottom)

        currentLimiting()

        when {
            MainXbox.getTriggerPressed(GenericHID.Hand.kRight) || MainXbox.getBumper(GenericHID.Hand.kRight) -> this.defaultCommand.start()
        }
        when (MainXbox.pov) {
        // Up - Scale
            0 -> commandGroup {
                addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                addParallel(AutoElevatorCommand(ElevatorPosition.SCALE_HIGH))
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

    fun inchesToNativeUnits(inches: Double) = Maths.feetToNativeUnits(inches / 12.0, ElevatorConstants.SENSOR_UNITS_PER_ROTATION, 1.25 / 2.0)
}

/**
 * Enum that contains elevator positions
 */
enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(17.0)),
    FIRST_STAGE(ElevatorSubsystem.inchesToNativeUnits(34.0)),
    SCALE(ElevatorSubsystem.inchesToNativeUnits(50.0)),
    SCALE_HIGH(ElevatorSubsystem.inchesToNativeUnits(57.0)),
    INTAKE(500);
}