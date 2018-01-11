package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import frc.team5190.robot.drive.DriveTrain
import frc.team5190.robot.navigation.NAVHelper

class Robot : IterativeRobot() {

    lateinit var automode : NAVHelper.AutoMode

    override fun robotInit() {
        DriveTrain
        automode = NAVHelper.AutoMode.CENTER

        NAVHelper.getTrajectory(automode)
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }
}

