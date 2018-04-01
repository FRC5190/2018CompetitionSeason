package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.Lidar
import frc.team5190.robot.util.CircularBuffer

class LidarElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    private val heightBuffer = CircularBuffer(20)

    override fun execute() {
        heightBuffer.add((if (Lidar.underScale) ElevatorSubsystem.inchesToNativeUnits(Lidar.scaleHeight)
        else ElevatorPosition.SCALE.ticks).toDouble())

        ElevatorSubsystem.set(ControlMode.MotionMagic,
                heightBuffer.average.coerceIn(ElevatorPosition.FIRST_STAGE.ticks.toDouble(), ElevatorPosition.SCALE_HIGH.ticks.toDouble()))
    }

    override fun isFinished() = !IntakeSubsystem.isCubeIn

}