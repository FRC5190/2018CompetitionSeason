package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand

/**
 *  Command that either intakes or outputs the cube
 */
class IntakeCommand(private val direction: IntakeDirection, private val auto: Boolean = false, private val outSpeed: Double = 0.95,
                    timeout: Double = 0.5) : TimedCommand(timeout) {

    private val inSpeed = -0.95

    init {
        requires(IntakeSubsystem)
    }

    override fun initialize() {
        IntakeSubsystem.intakeSolenoid.set(false)

        val motorOutput = when (direction) {
            IntakeDirection.IN -> inSpeed
            IntakeDirection.OUT -> outSpeed
        }

        IntakeSubsystem.set(ControlMode.PercentOutput, motorOutput)
    }

    override fun isFinished() = auto && isTimedOut
}
