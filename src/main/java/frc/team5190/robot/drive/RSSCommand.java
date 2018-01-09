package frc.team5190.robot.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.team5190.robot.Robot;

public class RSSCommand extends Command
{

    public RSSCommand()
    {
        requires(Robot.driveTrain);
        requires(Robot.navigation);
    }

    @Override
    protected void execute()
    {
        Robot.driveTrain.reset();
    }

    @Override
    protected boolean isFinished()
    {
        return true;
    }
}
