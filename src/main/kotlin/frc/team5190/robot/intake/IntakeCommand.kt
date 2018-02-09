package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand

/**
 *  Command that either intakes or outputs the cube
 */
class IntakeCommand(private val direction: IntakeDirection, private val state: Int = 0, private val timeout: Double = -.1) : TimedCommand(timeout) {

    constructor(direction: IntakeDirection, timeout: Double = -.1, intakeSpeed: Double = -0.8, outtakeSpeed: Double = 0.8, state: Int = 0) : this(direction, state, timeout) {
        inSpeed = intakeSpeed
        outSpeed = outtakeSpeed
    }

    private var inSpeed = -0.8
    private var outSpeed = 0.8

    init {
        requires(IntakeSubsystem)
    }

    override fun initialize() {
        if (state == -1) end()

        IntakeSubsystem.intakeSolenoid.set(false)

        if (timeout > 0) setTimeout(timeout)

        val motorOutput = when (direction) {
            IntakeDirection.IN -> inSpeed
            IntakeDirection.OUT -> outSpeed
        }

        IntakeSubsystem.set(ControlMode.PercentOutput, if (state == 1) motorOutput else motorOutput / 2)
    }

    override fun isFinished() = (timeout > 0 && isTimedOut) || state == -1
}
