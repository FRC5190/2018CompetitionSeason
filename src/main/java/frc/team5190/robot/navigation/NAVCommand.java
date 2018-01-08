package frc.team5190.robot.navigation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team5190.robot.Robot;

public class NAVCommand extends Command
{
    Arcade.Pos current = new Arcade.Pos(0, 0, 90);
    Arcade.Pos target  = new Arcade.Pos(10, 10, 0);


    public NAVCommand()
    {
        requires(Robot.driveTrain);
        requires(Robot.navigation);
    }

    @Override
    protected void initialize()
    {
        Robot.navigation.init(current, target);
    }

    @Override
    protected void execute()
    {
        Robot.navigation.drive();
    }

    @Override
    protected boolean isFinished()
    {
        return Robot.navigation.isFinished();
    }

    @Override
    protected void end()
    {
        Robot.driveTrain.tankDrive(0, 0, ControlMode.PercentOutput);
    }
}
