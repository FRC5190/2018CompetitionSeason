/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
            configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, TIMEOUT)
            setSensorPhase(ArmConstants.SENSOR_PHASE)
            configReverseSoftLimitEnable(false, TIMEOUT)
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

            configContinuousCurrentLimit(20, TIMEOUT)
            configPeakCurrentDuration(0, TIMEOUT)
            configPeakCurrentLimit(0, TIMEOUT)
            enableCurrentLimit(true)
        }
    }

    fun disableSensorControl() {
        with(masterArmMotor) {
            configReverseSoftLimitEnable(false, TIMEOUT)
        }

        set(ControlMode.PercentOutput, 0.0)
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
        SmartDashboard.putNumber("Absolute", (currentPosition % 1440).toDouble())
        SmartDashboard.putNumber("Arm Encoder Position", ArmSubsystem.currentPosition.toDouble())

        Controls.armSubsystem()
        currentLimiting()
    }
}

/**
 * Enum class that holds the different arm positions.
 */
enum class ArmPosition(val ticks: Int) {
    BEHIND((ArmConstants.DOWN_TICKS + 380)),
    ALL_UP(ArmConstants.DOWN_TICKS + 250),
    UP(ArmConstants.DOWN_TICKS + 200),
    MIDDLE(ArmConstants.DOWN_TICKS + 100),
    DOWN(ArmConstants.DOWN_TICKS);
}