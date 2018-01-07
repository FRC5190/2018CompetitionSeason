package frc.team5190.robot.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.team5190.robot.Robot;

public class DTRCommand extends Command {

    public DTRCommand() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void execute() {
        Robot.driveTrain.testDrive();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
