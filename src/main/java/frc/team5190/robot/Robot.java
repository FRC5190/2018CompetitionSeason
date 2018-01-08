package frc.team5190.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team5190.robot.drive.DTRSubsystem;
import frc.team5190.robot.navigation.NAVSubsystem;

public class Robot extends IterativeRobot
{

    public static DTRSubsystem driveTrain;
    public static NAVSubsystem navigation;
    public static OI oi;

    @Override
    public void robotInit()
    {
        driveTrain = new DTRSubsystem();
        navigation = new NAVSubsystem();
        oi = new OI();
    }

    @Override
    public void disabledInit()
    {
    }

    @Override
    public void autonomousInit()
    {
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void testInit()
    {
    }

    @Override
    public void robotPeriodic()
    {
    }

    @Override
    public void disabledPeriodic()
    {
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic()
    {
    }
}
