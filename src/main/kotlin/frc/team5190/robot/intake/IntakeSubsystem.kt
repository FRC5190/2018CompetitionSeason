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
        get() = leftCubeSensor.voltage > 0.9 && rightCubeSensor.voltage > 0.9

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

    fun set(controlMode: ControlMode, motorOutput: Double) {
        masterIntakeMotor.set(controlMode, motorOutput)
    }

    override fun initDefaultCommand() {
        defaultCommand = IntakeHoldCommand()
    }

    override fun periodic() {

        SmartDashboard.putBoolean("Cube In", IntakeSubsystem.isCubeIn)
        if (!Robot.INSTANCE!!.isOperatorControl) return


        Controls.intakeSubsystem()

    }
}

enum class IntakeDirection {
    IN, OUT
}