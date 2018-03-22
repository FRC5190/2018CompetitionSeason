/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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

    // Variable used to reset encoder position
    private var hasBeenReset = false

    // Variables used for current limiting
    private var state = MotorState.OK
    private var stalled = false

    var peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT

    init {
        with(masterElevatorMotor) {
            // Motor Inversion
            inverted = false

            // Sensors and Safety
            configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, ElevatorConstants.PID_SLOT, TIMEOUT)
            setSensorPhase(false)
            configLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TIMEOUT)
            overrideLimitSwitchesEnable(true)
            configForwardSoftLimitThreshold(ElevatorConstants.SOFT_LIMIT_FWD, TIMEOUT)
            configForwardSoftLimitEnable(true, TIMEOUT)

            // Brake Mode
            setNeutralMode(NeutralMode.Brake)

            // Closed Loop Control
            configPID(ElevatorConstants.PID_SLOT, ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, TIMEOUT)
            configAllowableClosedloopError(ElevatorConstants.PID_SLOT, inchesToNativeUnits(ElevatorConstants.TOLERANCE_INCHES), TIMEOUT)
            configNominalOutput(ElevatorConstants.NOMINAL_OUT, -ElevatorConstants.NOMINAL_OUT, TIMEOUT)
            configPeakOutput(ElevatorConstants.IDLE_PEAK_OUT, -ElevatorConstants.IDLE_PEAK_OUT, TIMEOUT)

            // Motion Magic Control
            configMotionCruiseVelocity(ElevatorConstants.MOTION_VELOCITY, 10)
            configMotionAcceleration(inchesToNativeUnits(ElevatorConstants.MOTION_ACCELERATION_INCHES) / 10, TIMEOUT)
            setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT)
            setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT)

            configClosedloopRamp(0.3, TIMEOUT)
            configOpenloopRamp(0.5, TIMEOUT)

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
    private fun resetEncoders() = masterElevatorMotor.setSelectedSensorPosition(0, ElevatorConstants.PID_SLOT, TIMEOUT)!!

    /**
     * Enables current limiting on the motor so we don't stall it
     */
    private fun currentLimiting() {
        currentBuffer.add(masterElevatorMotor.outputCurrent)
        state = limitCurrent(currentBuffer)

        when (state) {
            MotorState.OK -> {
                if (stalled) {
                    masterElevatorMotor.configPeakOutput(peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, -peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, TIMEOUT)
                } else {
                    masterElevatorMotor.configPeakOutput(peakElevatorOutput, -peakElevatorOutput, TIMEOUT)
                }
            }
            MotorState.STALL -> {
                masterElevatorMotor.configPeakOutput(peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, -peakElevatorOutput * ElevatorConstants.LIMITING_REDUCTION_FACTOR, TIMEOUT)
                stalled = true
            }
            MotorState.GOOD -> {
                masterElevatorMotor.configPeakOutput(peakElevatorOutput, -peakElevatorOutput, TIMEOUT)
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
        SmartDashboard.putNumber("Elevator Power", masterElevatorMotor.motorOutputPercent)
        if (ElevatorSubsystem.isElevatorAtBottom && !hasBeenReset) {
            this.resetEncoders()
            hasBeenReset = true
        }
        if (hasBeenReset && !ElevatorSubsystem.isElevatorAtBottom) {
            hasBeenReset = false
        }

        SmartDashboard.putBoolean("Reset Limit Switch", isElevatorAtBottom)

        currentLimiting()

        Controls.elevatorSubsystem()
    }

    fun inchesToNativeUnits(inches: Double) = Maths.feetToNativeUnits(inches / 12.0, ElevatorConstants.SENSOR_UNITS_PER_ROTATION, 1.25 / 2.0)
}

/**
 * Enum that contains elevator positions
 */
enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(20.0)),
    FIRST_STAGE(ElevatorSubsystem.inchesToNativeUnits(30.0)),
    SCALE(17000),
    SCALE_HIGH(ElevatorSubsystem.inchesToNativeUnits(57.0)),
    INTAKE(500);
}