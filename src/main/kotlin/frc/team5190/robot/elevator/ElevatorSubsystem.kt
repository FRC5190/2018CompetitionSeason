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
        get() = masterElevatorMotor.getSelectedSensorPosition(0)

    // Variable used to reset encoder position
    private var hasBeenReset = false

    var peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT
    var closedLpControl = true

    init {
        enableSensorControl()

        // Configure Slave Motor
        val slaveElevatorMotor = TalonSRX(MotorIDs.ELEVATOR_SLAVE).apply {
            inverted = true
            follow(masterElevatorMotor)

            configPeakOutput(ElevatorConstants.ACTIVE_PEAK_OUT, -ElevatorConstants.ACTIVE_PEAK_OUT, TIMEOUT)
        }

        arrayOf(masterElevatorMotor, slaveElevatorMotor).forEach {
            it.configContinuousCurrentLimit(30, TIMEOUT)
            it.configPeakCurrentDuration(0, TIMEOUT)
            it.configPeakCurrentLimit(0, TIMEOUT)
            it.enableCurrentLimit(true)

            it.configNominalOutput(ElevatorConstants.NOMINAL_OUT, -ElevatorConstants.NOMINAL_OUT, TIMEOUT)

            it.setNeutralMode(NeutralMode.Brake)
        }
    }

    fun enableSensorControl() {
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

            // Closed Loop Control
            configPID(ElevatorConstants.PID_SLOT, ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, TIMEOUT)
            configAllowableClosedloopError(ElevatorConstants.PID_SLOT, inchesToNativeUnits(ElevatorConstants.TOLERANCE_INCHES), TIMEOUT)

            // Motion Magic Control
            configMotionCruiseVelocity(inchesToNativeUnits(ElevatorConstants.MOTION_VELOCITY) / 10, 10)
            configMotionAcceleration(inchesToNativeUnits(ElevatorConstants.MOTION_ACCELERATION_INCHES) / 10, TIMEOUT)
            setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT)
            setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT)

            configClosedloopRamp(0.3, TIMEOUT)
            configOpenloopRamp(0.5, TIMEOUT)

            configPeakOutput(ElevatorConstants.IDLE_PEAK_OUT, -ElevatorConstants.IDLE_PEAK_OUT, TIMEOUT)

            closedLpControl = true
        }
    }

    fun disableSensorControl() {
        with(masterElevatorMotor) {
            overrideLimitSwitchesEnable(false)
            configForwardSoftLimitEnable(false, TIMEOUT)
            closedLpControl = false
        }

        set(ControlMode.PercentOutput, 0.0)
    }

    fun set(controlMode: ControlMode, output: Double) {
        masterElevatorMotor.set(controlMode, output)
    }

    private fun resetEncoders() = masterElevatorMotor.setSelectedSensorPosition(0, ElevatorConstants.PID_SLOT, TIMEOUT)!!

    override fun initDefaultCommand() {
        defaultCommand = ManualElevatorCommand()
    }

    override fun periodic() {
        SmartDashboard.putNumber("Elevator Power", masterElevatorMotor.motorOutputPercent)
        SmartDashboard.putNumber("Elevator Encoder Position", currentPosition.toDouble())


        if (ElevatorSubsystem.isElevatorAtBottom && !hasBeenReset) {
            this.resetEncoders()
            hasBeenReset = true
        }
        if (hasBeenReset && !ElevatorSubsystem.isElevatorAtBottom) {
            hasBeenReset = false
        }

        SmartDashboard.putBoolean("Reset Limit Switch", isElevatorAtBottom)

        Controls.elevatorSubsystem()
    }

    fun inchesToNativeUnits(inches: Double) = Maths.feetToNativeUnits(inches / 12.0, ElevatorConstants.SENSOR_UNITS_PER_ROTATION, 1.25 / 2.0)
}

enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(27.0)),
    FIRST_STAGE(ElevatorSubsystem.inchesToNativeUnits(32.0)),
    SCALE(17000),
    SCALE_HIGH(ElevatorSubsystem.inchesToNativeUnits(60.0)),
    INTAKE(500);
}
