package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.drive.DriveTrain
import frc.team5190.robot.navigation.NAVCommand
import frc.team5190.robot.navigation.NAVHelper

class Robot : IterativeRobot() {

    private val navCommand = NAVCommand(NAVHelper.CENTER)

    init {
        DriveTrain
    }

    override fun robotInit() {
        navCommand.cancel()
    }

    override fun autonomousInit() {
        navCommand.start()
    }

    override fun autonomousPeriodic() {
        this.printToShuffleboard()
        Scheduler.getInstance().run()
    }


    override fun teleopInit() {
        navCommand.cancel()
    }

    override fun teleopPeriodic() {
        this.printToShuffleboard()
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
        navCommand.cancel()
    }

    private fun printToShuffleboard() {
        SmartDashboard.putNumber("Left Motor RPM", DriveTrain.frontLeft.sensorCollection.quadratureVelocity * 600.0 / 1440.0)
        SmartDashboard.putNumber("Right Motor RPM", DriveTrain.frontRight.sensorCollection.quadratureVelocity * 600.0 / 1440.0)
    }
}

