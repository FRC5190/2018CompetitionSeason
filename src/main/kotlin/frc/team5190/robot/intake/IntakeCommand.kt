package frc.team5190.robot.intake

import edu.wpi.first.wpilibj.command.Command

class IntakeCommand(private val direction: IntakeDirection) : Command() {

    private val outSpeed = 0.5
    private val inSpeed = -0.5
    private val maxCurrent = 10

    init {
        requires(IntakeSubsystem)
    }

    private var currentHistory = mutableListOf<Double>()

    override fun execute() {
        val motorOutput = when (direction) {
            IntakeDirection.IN -> inSpeed
            IntakeDirection.OUT -> outSpeed
            IntakeDirection.NOTHING -> 0.0
        }
        with(IntakeSubsystem.intakeTalon) {
            set(motorOutput)

            currentHistory.add(0, outputCurrent)
            currentHistory = currentHistory.subList(0, Math.min(50, currentHistory.size))
        }
    }

    override fun isFinished() = direction != IntakeDirection.NOTHING && currentHistory.average() > maxCurrent

}