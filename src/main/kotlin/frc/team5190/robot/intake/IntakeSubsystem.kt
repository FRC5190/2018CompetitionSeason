/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.Robot
import frc.team5190.robot.util.*

object IntakeSubsystem : Subsystem() {

    // Master intake talon
    private val masterIntakeMotor = TalonSRX(MotorIDs.INTAKE_LEFT)

    private val leftCubeSensor = AnalogInput(ChannelIDs.LEFT_CUBE_SENSOR)
    private val rightCubeSensor = AnalogInput(ChannelIDs.RIGHT_CUBE_SENSOR)

    @Suppress("ConstantConditionIf")
    val isCubeIn
        get() = if (DriveConstants.IS_RACE_ROBOT) leftCubeSensor.voltage > 0.9 && rightCubeSensor.voltage > 0.9
        else leftCubeSensor.voltage > 0.9 && rightCubeSensor.voltage > 0.9

    val amperage
        get() = masterIntakeMotor.outputCurrent


    // Solenoid
    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

    init {
        masterIntakeMotor.apply {
            inverted = false
            configVoltageCompSaturation(12.0, TIMEOUT)
            enableVoltageCompensation(true)
        }
        val slaveIntakeMotor = TalonSRX(MotorIDs.INTAKE_RIGHT).apply{
            follow(masterIntakeMotor)
            inverted = true
        }
        arrayOf(masterIntakeMotor, slaveIntakeMotor).forEach {
            it.configContinuousCurrentLimit(18, TIMEOUT)
            it.configPeakCurrentDuration(0, TIMEOUT)
            it.configPeakCurrentLimit(0, TIMEOUT)
            it.enableCurrentLimit(true)
        }
    }

    fun disableVoltageCompensation() {
        masterIntakeMotor.enableVoltageCompensation(false)
    }

    fun enableVoltageCompensation() {
        masterIntakeMotor.enableVoltageCompensation(true)
    }

    /**
     * Sets motor output
     * @param controlMode Control Mode of the Talon
     * @param motorOutput Output to the talon
     */
    fun set(controlMode: ControlMode, motorOutput: Double) {
        masterIntakeMotor.set(controlMode, motorOutput)
    }

    /**
     * Sets the default command
     */
    override fun initDefaultCommand() {
        defaultCommand = IntakeHoldCommand()
    }

    /**
     * Executed periodcally
     */
    override fun periodic() {

        SmartDashboard.putBoolean("Cube In", IntakeSubsystem.isCubeIn)
        if (!Robot.INSTANCE!!.isOperatorControl) return


        Controls.intakeSubsystem()

    }
}

/**
 * Enum that holds intake directions
 */
enum class IntakeDirection {
    IN, OUT
}