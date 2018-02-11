package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.util.IntakeConstants

/**
 *  Command that either intakes or outputs the cube
 */
class IntakeCommand(private val direction: IntakeDirection, private val timeout: Double = -.1) : TimedCommand(timeout) {

    constructor(direction: IntakeDirection,
                timeout: Double = -.1,
                intakeSpeed: Double = -IntakeConstants.DEFAULT_SPEED,
                outtakeSpeed: Double = IntakeConstants.DEFAULT_SPEED) : this(direction, timeout) {

        inSpeed = intakeSpeed
        outSpeed = outtakeSpeed
    }

    private var inSpeed = -IntakeConstants.DEFAULT_SPEED
    private var outSpeed = IntakeConstants.DEFAULT_SPEED

    init {
        requires(IntakeSubsystem)
    }

    override fun initialize() {
        IntakeSubsystem.intakeSolenoid.set(false)

        if (timeout > 0) setTimeout(timeout)

        val motorOutput = when (direction) {
            IntakeDirection.IN -> inSpeed
            IntakeDirection.OUT -> outSpeed
        }

        IntakeSubsystem.set(ControlMode.PercentOutput, motorOutput)
    }

    override fun isFinished() = timeout > 0 && isTimedOut
}
