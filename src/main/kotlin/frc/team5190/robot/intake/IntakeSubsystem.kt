/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.Robot
import frc.team5190.robot.util.*

object IntakeSubsystem : Subsystem() {

    // Master intake talon
    private val masterIntakeMotor = TalonSRX(MotorIDs.INTAKE_LEFT)

    private val leftCubeSensor = DigitalInput(ChannelIDs.LEFT_CUBE_SENSOR)
    private val rightCubeSensor = DigitalInput(ChannelIDs.RIGHT_CUBE_SENSOR)

    val isCubeIn
        get() = !leftCubeSensor.get() || !rightCubeSensor.get()

    val amperage
        get() = masterIntakeMotor.outputCurrent


    // Solenoid
    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

    init {
        with(masterIntakeMotor) {
            inverted = false
            configVoltageCompSaturation(12.0, TIMEOUT)
            enableVoltageCompensation(false)
        }
        with(TalonSRX(MotorIDs.INTAKE_RIGHT)) {
            follow(masterIntakeMotor)
            inverted = true
        }
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