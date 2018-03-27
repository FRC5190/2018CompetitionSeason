/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.*

/**
 * Subsystem for controlling the arm mechanism
 */
object ArmSubsystem : Subsystem() {

    // Master arm talon
    private val masterArmMotor = TalonSRX(MotorIDs.ARM)

    // Buffer to hold amperage values for current limiting
    private val currentBuffer = CircularBuffer(25)

    // Variables for current limiting
    private var stalled = false
    private var state = MotorState.OK

    // Variable that stores the mode the subsyste,
    var closedLpControl = true

    // Returns the amperage of the motor
    val amperage
        get() = masterArmMotor.outputCurrent

    // Returns the current encoder position of the motor
    val currentPosition
        get() = masterArmMotor.getSelectedSensorPosition(0)

    init {
        enableSensorControl()

        // Configure current limiting
        currentBuffer.configureForTalon(ArmConstants.LOW_PEAK, ArmConstants.HIGH_PEAK, ArmConstants.DUR)
    }

    fun enableSensorControl() {
        with(masterArmMotor) {
            // Motor Inversion
            inverted = ArmConstants.INVERTED

            // Sensors and Safety
            configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, TIMEOUT)
            setSensorPhase(ArmConstants.SENSOR_PHASE)
            configReverseSoftLimitEnable(true, TIMEOUT)
            configReverseSoftLimitThreshold(ArmPosition.DOWN.ticks - 100, TIMEOUT)

            // Brake Mode
            setNeutralMode(NeutralMode.Brake)

            // Closed Loop Control
            configPID(ArmConstants.PID_SLOT, ArmConstants.P, ArmConstants.I, ArmConstants.D, TIMEOUT)
            configNominalOutput(ArmConstants.NOMINAL_OUT, -ArmConstants.NOMINAL_OUT, TIMEOUT)
            configPeakOutput(ArmConstants.PEAK_OUT, -ArmConstants.PEAK_OUT, TIMEOUT)
            configAllowableClosedloopError(0, ArmConstants.TOLERANCE, TIMEOUT)

            // Motion Magic Control
            configMotionCruiseVelocity(ArmConstants.MOTION_VELOCITY, TIMEOUT)
            configMotionAcceleration(ArmConstants.MOTION_ACCELERATION, TIMEOUT)
            setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT)
            setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT)

            configClosedloopRamp(0.3, TIMEOUT)
            configOpenloopRamp(0.5, TIMEOUT)

            closedLpControl = true
        }
    }

    fun disableSensorControl() {
        with(masterArmMotor) {
            configReverseSoftLimitEnable(false, TIMEOUT)
            closedLpControl = false
        }
    }

    /**
     * Sets the motor output.
     * @param controlMode Control Mode for the Talon
     * @param output Output to the motor
     */
    fun set(controlMode: ControlMode, output: Double) {
        masterArmMotor.set(controlMode, output)
    }

    /**
     * Enables current limiting on the motor so we don't stall it
     */
    private fun currentLimiting() {
        currentBuffer.add(masterArmMotor.outputCurrent)
        state = limitCurrent(currentBuffer)

        when (state) {
            MotorState.OK -> {
                if (stalled) {
                    masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, -ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, TIMEOUT)
                } else {
                    masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT, -ArmConstants.PEAK_OUT, TIMEOUT)
                }
            }
            MotorState.STALL -> {
                masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, -ArmConstants.PEAK_OUT * ArmConstants.LIMITING_REDUCTION_FACTOR, TIMEOUT)
                stalled = true
            }
            MotorState.GOOD -> {
                masterArmMotor.configPeakOutput(ArmConstants.PEAK_OUT, -ArmConstants.PEAK_OUT, TIMEOUT)
                stalled = false
            }
        }
    }

    /**
     * Sets the default command for the subsytem
     */
    override fun initDefaultCommand() {
        defaultCommand = ManualArmCommand()
    }

    /**
     * Runs periodically
     */
    override fun periodic() {

        if (currentPosition > 4096 || currentPosition < 0) {
            masterArmMotor.setSelectedSensorPosition(currentPosition % 4096, 0, TIMEOUT)
        }

        Controls.armSubsystem()

        currentLimiting()
    }
}

/**
 * Enum class that holds the different arm positions.
 */
enum class ArmPosition(val ticks: Int) {
    BEHIND(ArmConstants.DOWN_TICKS + 1550),
    ALL_UP(ArmConstants.DOWN_TICKS + 1250),
    UP(ArmConstants.DOWN_TICKS + 800),
    MIDDLE(ArmConstants.DOWN_TICKS + 350),
    DOWN(ArmConstants.DOWN_TICKS);
}