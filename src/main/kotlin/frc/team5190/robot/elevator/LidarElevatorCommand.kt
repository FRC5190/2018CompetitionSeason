package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.Lidar
import frc.team5190.robot.util.CircularBuffer
import frc.team5190.robot.util.ElevatorConstants

class LidarElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
        requires(Lidar)
    }

    private val heightBuffer = CircularBuffer(2)

    override fun initialize() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.ACTIVE_PEAK_OUT
    }

    override fun execute() {
        if (Lidar.underScale) heightBuffer.add(ElevatorSubsystem.inchesToNativeUnits(Lidar.scaleHeight - 15).toDouble())

        ElevatorSubsystem.set(ControlMode.MotionMagic,
                if (Lidar.underScale) heightBuffer.average.coerceIn(ElevatorPosition.FIRST_STAGE.ticks.toDouble(), ElevatorPosition.SCALE_HIGH.ticks.toDouble())
                else ElevatorPosition.SCALE.ticks.toDouble())

        println(heightBuffer.average)
    }

    override fun end() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT
    }

    override fun isFinished() = !IntakeSubsystem.isCubeIn
            && ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks -  ElevatorSubsystem.inchesToNativeUnits(1.0)

}