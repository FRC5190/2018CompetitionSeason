package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
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
        Scheduler.getInstance().run()
    }


    override fun teleopInit() {
        navCommand.cancel()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
        navCommand.cancel()
    }
}

