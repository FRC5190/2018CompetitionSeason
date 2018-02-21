package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.IntakeConstants

/**
 *  Command that either intakes or outputs the cube
 */
class IntakeCommand(private val direction: IntakeDirection, private val timeout: Double = -.1,
                    private val inSpeed: Double = IntakeConstants.DEFAULT_SPEED,
                    private val outSpeed: Double = IntakeConstants.DEFAULT_SPEED) : Command() {

    init {
        requires(IntakeSubsystem)
    }


    override fun initialize() {
        IntakeSubsystem.intakeSolenoid.set(false)

        if (timeout > 0) setTimeout(timeout)

        val motorOutput = when (direction) {
            IntakeDirection.IN -> -inSpeed
            IntakeDirection.OUT -> outSpeed
        }

        IntakeSubsystem.set(ControlMode.PercentOutput, motorOutput)
    }

    override fun isFinished() = (timeout > 0 && isTimedOut) ||
            (direction == IntakeDirection.IN && IntakeSubsystem.outputCurrent > IntakeConstants.AMP_THRESHOLD)
}
