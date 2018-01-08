package frc.team5190.robot.navigation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team5190.robot.Robot;
import frc.team5190.robot.util.Constants;
import frc.team5190.robot.util.Maths;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

@SuppressWarnings("WeakerAccess")
public class NAVSubsystem extends Subsystem
{
    private EncoderFollower left, right;

    public void init(Arcade.Pos currentPos, Arcade.Pos targetPos)
    {
        Waypoint[] waypoints = new Waypoint[2];

        waypoints[0] = new Waypoint(currentPos.getXMeters(), currentPos.getYMeters(), Pathfinder.r2d(currentPos.getAngle()));
        waypoints[1] = new Waypoint(targetPos.getXMeters(),  targetPos.getYMeters(),  Pathfinder.r2d(targetPos.getAngle()));

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
                Constants.TIME_STEP, Constants.VELOCITY, Constants.MAX_ACCEL, Constants.JERK_RATE);

        Trajectory trajectory = Pathfinder.generate(waypoints, config);


        TankModifier modifier = new TankModifier(trajectory);
        modifier.modify(Constants.WHEEL_BASE_WIDTH / 12);

        left  = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());

        left .configureEncoder(Robot.driveTrain.frontLeft .getSelectedSensorPosition(0), Constants.TICKS_PER_ROTATION, Maths.ftm(Constants.WHEEL_RADIUS / 6.0));
        right.configureEncoder(Robot.driveTrain.frontRight.getSelectedSensorPosition(0), Constants.TICKS_PER_ROTATION, Maths.ftm(Constants.WHEEL_RADIUS / 6.0));

        left .configurePIDVA(1.0, 0, 0, 1 / Constants.VELOCITY, 0);
        right.configurePIDVA(1.0, 0, 0, 1 / Constants.VELOCITY, 0);
    }

    public void drive()
    {
        double l = left .calculate(Robot.driveTrain.frontLeft .getSelectedSensorPosition(0));
        double r = right.calculate(Robot.driveTrain.frontRight.getSelectedSensorPosition(0));

        double gyro_heading = Robot.driveTrain.navX.getAngle();
        double desired_heading = Pathfinder.r2d(left.getHeading());

        double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

        Robot.driveTrain.tankDrive(l + turn, r - turn, ControlMode.PercentOutput);
    }

    public boolean isFinished()
    {
        return left.isFinished() && right.isFinished();
    }


    @Override
    protected void initDefaultCommand()
    {

    }
}
