/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.IntakeConstants
import kotlin.math.absoluteValue

/**
 *  Command that either intakes or outputs the cube
 */
open class IntakeCommand(private val direction: IntakeDirection, private val timeout: Double = -.1,
                    private val inSpeed: Double = IntakeConstants.DEFAULT_IN_SPEED,
                    private val outSpeed: Double = IntakeConstants.DEFAULT_OUT_SPEED) : Command() {

    init {
        this.requires(IntakeSubsystem)
    }

    /**
     * Initializes the command
     */
    override fun initialize() {
        IntakeSubsystem.intakeSolenoid.set(false)

        if (timeout > 0) setTimeout(timeout)

        val motorOutput = when (direction) {
            IntakeDirection.IN -> -(inSpeed.absoluteValue)
            IntakeDirection.OUT -> outSpeed.absoluteValue
        }

        IntakeSubsystem.set(ControlMode.PercentOutput, motorOutput)
    }

    /**
     * Checks if the intake has finished outtaking or intaking based on amperage values and timeouts
     */
    override fun isFinished() = (timeout > 0 && isTimedOut) ||
            (direction == IntakeDirection.IN && IntakeSubsystem.isCubeIn)
}
