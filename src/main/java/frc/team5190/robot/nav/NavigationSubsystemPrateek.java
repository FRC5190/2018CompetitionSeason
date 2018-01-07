package frc.team5190.robot.nav;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.team5190.robot.Robot;

public class NavigationSubsystemPrateek extends PIDSubsystem
{
    private int currentX;
    private int currentY;

    private double currentAngle;
    private double targetAngle;

    public boolean isAngleCalibrated = false;

    public NavigationSubsystemPrateek(double p, double i, double d) {
        super(p, i, d);
    }

    @Override
    protected double returnPIDInput() {
        currentAngle = Robot.driveTrain.navX.getAngle();
        return currentAngle;
    }

    @Override
    protected void usePIDOutput(double output) {
        if (Math.abs(currentAngle - targetAngle) > 0.2) {
            Robot.driveTrain.turn(output);
        }
        else {
            isAngleCalibrated = true;
        }
    }

    @Override
    protected void initDefaultCommand() {

    }

    private void goToCoordinates(int targetX, int targetY) {
        double hyp   = Math.sqrt(Math.pow((targetX - currentX), 2) + Math.pow((targetY - currentY), 2));
        double angle = Math.acos((targetX - currentX) / hyp);

        this.targetAngle = currentAngle + angle;
        this.setSetpoint(targetAngle);
        this.setAbsoluteTolerance(0.2);

    }
}
