package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX

class Robot : IterativeRobot() {

    init {
        DriveSubsystem
        NavX
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

}
