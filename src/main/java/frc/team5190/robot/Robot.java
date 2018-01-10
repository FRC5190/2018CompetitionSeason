package frc.team5190.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import frc.team5190.robot.drive.DTRSubsystem;
import frc.team5190.robot.navigation.NAVHelper;

public class Robot extends IterativeRobot {

    public static DTRSubsystem driveTrain = new DTRSubsystem();
    public static OI oi = new OI();

    @Override
    public void robotInit() {
        NAVHelper.AutoMode autonomousMode = NAVHelper.AutoMode.CENTER;

        try {
            NAVHelper.configPointsFromCSV(autonomousMode);
        } catch (Exception e) {
            System.out.println("Auto init failed.");
            // TODO implement backup
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }
}
