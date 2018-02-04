package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.absoluteValue

class AutoArmCommand(private val armPosition: ArmPosition) : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun initialize() {
        ArmSubsystem.set(ControlMode.MotionMagic, armPosition.ticks.toDouble())
    }

<<<<<<< HEAD
    override fun isFinished() = ArmSubsystem.closedLoopError < 150
=======
    override fun isFinished() = (ArmSubsystem.currentPosition - armPosition.ticks).absoluteValue  < 50
>>>>>>> master
}


enum class ArmPosition (val ticks: Int){
    BEHIND(2800), // When placing scale backwards
    UP(2600), // Arm is always up, basically where it starts in auto
    MIDDLE(1900), // Angled a little up to help placement on scale and switch
    DOWN(1668); // Lowest position, used for intaking the cube
}
