package frc.team5190.robot.vision

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX

class FindCubeCommand : PIDCommand(0.05, 0.003,0.03) {

    init {
        requires(VisionSubsystem)
        requires(DriveSubsystem)
    }

    override fun initialize() {
        setTimeout(5.0)
        setName("Jevois", "FindCube")
        pidController.setAbsoluteTolerance(2.0)

        Timer.getFPGATimestamp()
        val visible = VisionSubsystem.isTgtVisible
        val angle = VisionSubsystem.tgtAngle_Deg
        val distance = VisionSubsystem.tgtRange_in
        System.out.println("FindCubeCommand: $visible, $angle, $distance")

        setpoint = VisionSubsystem.tgtAngle_Deg + NavX.pidGet()
        println(setpoint + 10)
        setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-0.9, 0.9)
        pidController.setAbsoluteTolerance(1.0)
        pidController.setContinuous(true)
    }

    override fun usePIDOutput(output: Double) {
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, output, -output)
    }

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