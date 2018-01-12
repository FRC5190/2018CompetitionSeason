package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import frc.team5190.robot.drive.DriveTrain
import frc.team5190.robot.navigation.NAVHelper
import frc.team5190.robot.navigation.leftPoints
import frc.team5190.robot.navigation.rightPoints

class Robot : IterativeRobot() {

    init {
        DriveTrain
    }

    override fun robotInit() {
        val autoGenerator = NAVHelper.CENTER

        leftPoints  = autoGenerator.trajectoryLeft
        rightPoints = autoGenerator.trajectoryRight
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }
}

