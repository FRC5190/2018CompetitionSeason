/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.Robot
import frc.team5190.robot.climb.WinchSubsystem
import frc.team5190.robot.util.*

object IntakeSubsystem : Subsystem() {

    // Master intake talon
    private val masterIntakeMotor = TalonSRX(MotorIDs.INTAKE_LEFT)
    
    // Buffer to hold amperage values for current limiting
    private val currentBuffer = CircularBuffer(25)

    // Returns the amperage of the motor
    val amperage
        get() = currentBuffer.average

    // Solenoid
    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

    // Latch Boolean
    private var teleIntake = false

    init {
        with(masterIntakeMotor) {
            inverted = false
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

        currentBuffer.add(masterIntakeMotor.outputCurrent)

        if (!Robot.INSTANCE!!.isOperatorControl) return

        val winchState = WinchSubsystem.winchState

        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) && !winchState -> {
                IntakeCommand(IntakeDirection.IN).start()
                teleIntake = true
            }
            MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5 && !winchState-> {
                IntakeCommand(IntakeDirection.OUT).start()
                teleIntake = true
            }
            teleIntake -> {
                currentCommand?.cancel()
                teleIntake = false
            }
        }
    }
}

/**
 * Enum that holds intake directions
 */
enum class IntakeDirection {
    IN, OUT
}