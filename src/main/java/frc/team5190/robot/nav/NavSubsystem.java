package frc.team5190.robot.nav;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.team5190.robot.Robot;

@SuppressWarnings("ALL")
public class NavSubsystem extends PIDSubsystem
{
    private int currentX;
    private int currentY;

    private double currentAngle;
    private double targetAngle;

    private double distance;

    private STAGE current;

    private enum STAGE {
        ANGLE, STRAIGHT;
    }

    public NavSubsystem(double p, double i, double d) {
        super(p, i, d);
    }

    @Override
    protected double returnPIDInput() {
        currentAngle = Robot.driveTrain.navX.getAngle();
        return currentAngle;
    }

    @Override
    protected void usePIDOutput(double output) {
        if (this.current == STAGE.ANGLE) {
            if (Math.abs(targetAngle - currentAngle) > 0.2) {
                Robot.driveTrain.turn(output);
            }
            else {
                current = STAGE.STRAIGHT;
            }
        }
        else {

        }
    }

    @Override
    protected void initDefaultCommand() {

    }

    private void goToCoordinates(int targetX, int targetY) {
        distance     = Math.hypot((targetX - currentX), (targetY - currentY));
        double angle = Math.acos((targetX - currentX) / distance);

        this.targetAngle = currentAngle + angle;
        this.setSetpoint(targetAngle);
        this.setAbsoluteTolerance(0.2);
        this.current = STAGE.ANGLE;
    }
}
