package frc.team5190.robot.listener

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.util.IntakeConstants

/**
 *  Command that either intakes or outputs the cube
 */
class GetPathCommand(private val path: String, private val obstruction: Boolean = false, private val index: Double = 0.0) : TimedCommand(0.5) {

    init {
        requires(ListenerSubsystem)
    }

    override fun initialize() {
        ListenerSubsystem.getPath(path, obstruction, index)
    }

    override fun isFinished() = isTimedOut
}
