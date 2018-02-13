package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX

/**
 * Command that turns the robot to a certain angle
 * @param angle Angle to turn to in degrees
 */
class TurnCommand(angle: Double) : PIDCommand(0.075, 0.00, 0.1) {

    init {
        requires(DriveSubsystem)

        // Only execute the command for a total of a max of 5 seconds (should be close enough to target by then)
        setTimeout(3.0)
        setName("DriveSystem", "RotateController")

        setpoint = angle
        setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-0.8, 0.8)
        pidController.setAbsoluteTolerance(5.0)
        pidController.setContinuous(true)
    }

    override fun initialize() {
    }

    override fun usePIDOutput(output: Double) = DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, output, -output)

    override fun returnPIDInput(): Double = NavX.pidGet()

    private var onTargetTime: Long = 0

    override fun isFinished(): Boolean {
        when {
            pidController.onTarget() -> onTargetTime++
            else -> onTargetTime = 0
        }
        // stop the command once it is at the target for at least 500ms and/or the command has elapsed its max time allowed
        return onTargetTime > 500 / 20 || isTimedOut
    }
}