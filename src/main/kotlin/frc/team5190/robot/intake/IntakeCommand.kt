package frc.team5190.robot.intake

import edu.wpi.first.wpilibj.command.Command

/**
 *  Command that either intakes or outputs the cube
 */
class IntakeCommand(private val direction: IntakeDirection) : Command() {

    private val outSpeed = 0.5
    private val inSpeed = -0.5
    private val maxCurrent = 10

    init {
        requires(IntakeSubsystem)
    }

    override fun initialize() {
        IntakeSubsystem.intakeSolenoid.set(false)
    }

    private var currentHistory = mutableListOf<Double>()

    override fun execute() {
        val motorOutput = when (direction) {
            IntakeDirection.IN -> inSpeed
            IntakeDirection.OUT -> outSpeed
        }
        with(IntakeSubsystem.intakeTalon) {
            set(motorOutput)

            currentHistory.add(0, outputCurrent)
            currentHistory = currentHistory.subList(0, Math.min(50, currentHistory.size))
        }
    }

    override fun end() {
        IntakeSubsystem.intakeTalon.set(0.0)
    }

    override fun isFinished() = currentHistory.average() > maxCurrent
}


enum class IntakeDirection {
    IN, OUT
}