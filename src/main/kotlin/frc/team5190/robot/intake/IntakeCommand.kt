package frc.team5190.robot.intake

import edu.wpi.first.wpilibj.command.TimedCommand

/**
 *  Command that either intakes or outputs the cube
 */
class IntakeCommand(private val direction: IntakeDirection, val auto: Boolean = false, timeout: Double = 0.5) : TimedCommand(timeout) {

    private val outSpeed = 0.95
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
        IntakeSubsystem.intakeTalon.set(motorOutput)
    }

    override fun isFinished() = auto && isTimedOut
}

enum class IntakeDirection {
    IN, OUT
}