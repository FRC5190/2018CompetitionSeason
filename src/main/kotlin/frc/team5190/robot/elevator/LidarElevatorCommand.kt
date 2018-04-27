package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.Lidar
import frc.team5190.robot.util.CircularBuffer
import frc.team5190.robot.util.DriveConstants
import frc.team5190.robot.util.ElevatorConstants

class LidarElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
        requires(Lidar)
    }

    // Circular buffer that holds Lidar values. This ensures we don't have "jumpy" movements
    private val heightBuffer = CircularBuffer(3)

    // Initializes command
    override fun initialize() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.ACTIVE_PEAK_OUT
    }

    // Excecuted periodically
    override fun execute() {
        // Setpoint is changed dynamically based on the motion of the scale
        if (Lidar.underScale) heightBuffer.add(ElevatorSubsystem.inchesToNativeUnits(Lidar.scaleHeight - 15.0).toDouble())

        // Motion magic setpoint
        ElevatorSubsystem.set(ControlMode.MotionMagic,
                if (Lidar.underScale) heightBuffer.average.coerceIn(ElevatorPosition.FIRST_STAGE.ticks.toDouble(), ElevatorPosition.SCALE_HIGH.ticks.toDouble())
                else ElevatorPosition.SCALE.ticks.toDouble())
    }

    // Called when the command ends
    override fun end() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT
    }

    // Checks command for completion
    override fun isFinished() = !IntakeSubsystem.isCubeIn
            && ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks - ElevatorSubsystem.inchesToNativeUnits(1.0)

}