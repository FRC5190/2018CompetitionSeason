/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan S, Prateek M
 */

package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.MainXbox
import frc.team5190.robot.Robot
import frc.team5190.robot.util.*

object IntakeSubsystem : Subsystem() {

    private val masterIntakeMotor = TalonSRX(MotorIDs.INTAKE_LEFT)
    private val currentBuffer = CircularBuffer(25)

    val outputCurrent
        get() = currentBuffer.average

    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

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

    fun set(controlMode: ControlMode, motorOutput: Double) {
        masterIntakeMotor.set(controlMode, motorOutput)
    }

    override fun initDefaultCommand() {
        defaultCommand = IntakeHoldCommand()
    }

    override fun periodic() {

        currentBuffer.add(masterIntakeMotor.outputCurrent)

        if (!Robot.INSTANCE!!.isOperatorControl) return

        SmartDashboard.putNumber("Intake Motor Amps", this.outputCurrent)

        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) -> {
                IntakeCommand(IntakeDirection.IN).start()
                teleIntake = true
            }
            MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5 -> {
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

enum class IntakeDirection {
    IN, OUT
}