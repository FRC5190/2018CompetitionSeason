package frc.team5190.robot.navigation

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.PIDOutput
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import frc.team5190.robot.drive.DriveTrain

class TRNController(targetAngleDegrees: Double) : PIDOutput {

    private var rotateToAngleRate = 0.0

    private val p = 0.03
    private val i = 0.0
    private val d = 0.0
    private val f = 0.0

    private val target = targetAngleDegrees

    private val pidController = PIDController(p, i, d, f, DriveTrain.navX, this)

    init {
        pidController.setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-0.2, 0.2)
        pidController.setAbsoluteTolerance(2.0)
        pidController.setContinuous(true)
        pidController.disable()

        LiveWindow.add(pidController)
    }

    fun enable() {
        pidController.setpoint = target
        rotateToAngleRate = 0.0
        pidController.enable()
    }

    fun rotateToAngle() {
        val left = rotateToAngleRate
        val right = rotateToAngleRate

        DriveTrain.tankDrive(left, -right, ControlMode.PercentOutput)
    }

    fun hasFinished() = Math.abs(target - DriveTrain.navX.angle) < 1

    override fun pidWrite(output: Double) {
        rotateToAngleRate = output
    }

}