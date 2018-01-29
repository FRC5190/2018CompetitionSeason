package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class AutoArmCommand(private val armPosition: ArmPosition) : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun initialize() {
        ArmSubsystem.set(ControlMode.Position, armPosition.ticks.toDouble())
    }

    override fun isFinished() = ArmSubsystem.closedLoopError < 50
}


enum class ArmPosition (val ticks: Int){
    BEHIND(0), // When placing scale backwards
    UP(0), // Arm is always up, basically where it starts in auto
    MIDDLE(1200), // Angled a little up to help placement on scale and switch
    DOWN(0); // Lowest position, used for intaking the cube
}
