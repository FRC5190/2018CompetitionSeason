package frc.team5190.robot.vision

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.sensors.NavX

class FindCubeCommand() : PIDCommand(0.03, 0.003, 0.05) {

    init {
        requires(VisionSubsystem)

        // Only execute the command for a total of a max of 5 seconds (should be close enough to target by then)
        setTimeout(5.0)
        setName("Jevois", "FindCube")

        val startTime = Timer.getFPGATimestamp()
        while (Timer.getFPGATimestamp() - startTime < 3) {
            if (VisionSubsystem.isTgtVisible == 1) {
                setpoint = VisionSubsystem.tgtAngle_Deg + NavX.pidGet()
                setInputRange(-180.0, 180.0)
                pidController.setOutputRange(-0.9, 0.9)
                pidController.setAbsoluteTolerance(2.0)
                pidController.setContinuous(true)
                break
            }
            else
                Thread.sleep(100)
        }
    }

    override fun usePIDOutput(output: Double) {
        System.out.println("FindCubeCommand: " + output)
//        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, output, -output)
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