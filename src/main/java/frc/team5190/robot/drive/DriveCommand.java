package frc.team5190.robot.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.team5190.robot.Robot;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void execute() {
        Robot.driveTrain.drive();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
