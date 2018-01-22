package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.PIDOutput
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX

class TurnCommand(private val angle: Double) : TimedCommand(5.0) {

    init {
        requires(DriveSubsystem)
    }

    private lateinit var controller: TurnController

    override fun initialize() {
        controller = TurnController(angle)
        controller.enable()
    }

    private var onTargetTime: Long = 0

    override fun end() {
        controller.disable()
    }

    override fun isFinished(): Boolean {
        when {
            controller.onTarget() -> onTargetTime++
            else -> onTargetTime = 0
        }
        return onTargetTime > 500 / 20
    }
}

class TurnController(angle: Double) : PIDController(0.03, 0.003, 0.05, NavX, PIDOutput { output ->
    DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, output, -output)
}) {
    init {
//        SmartDashboard.putData("PID Controller", this)
        setpoint = angle
        setInputRange(-180.0, 180.0)
        val output = 0.9
        setOutputRange(-output, output)
        setAbsoluteTolerance(2.0)
        setContinuous(true)
        setName("DriveSystem", "RotateController")
    }
}