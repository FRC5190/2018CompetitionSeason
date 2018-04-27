package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls
import frc.team5190.robot.util.TIMEOUT


class WinchCommand : Command() {

    init {
        requires(ClimbSubsystem)
    }

    // Initializes the command
    override fun initialize() {
        ClimbSubsystem.masterClimbMotor.setSelectedSensorPosition(0, 0, TIMEOUT)
    }

    // Called periodically
    override fun execute() = Controls.winchSubsystem()

    // Command never finishes because it will be interrupted
    override fun isFinished() = false
}

class IdleClimbCommand : Command() {

    init {
        requires(ClimbSubsystem)
    }

    // Initializes the command
    override fun initialize() {
        ClimbSubsystem.set(ControlMode.PercentOutput, 0.0)
    }

    // Command never finishes because it is the default command
    override fun isFinished() = false
}