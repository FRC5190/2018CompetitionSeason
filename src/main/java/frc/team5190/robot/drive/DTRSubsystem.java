package frc.team5190.robot.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team5190.robot.Robot;
import frc.team5190.robot.util.Constants;

@SuppressWarnings("WeakerAccess")
public class DTRSubsystem extends Subsystem {

    public TalonSRX frontLeft;
    public TalonSRX frontRight;
    public TalonSRX rearLeft;
    public TalonSRX rearRight;

    public AHRS navX;

    public DTRSubsystem() {
        frontLeft = new TalonSRX(Constants.FRONT_LEFT);
        frontRight = new TalonSRX(Constants.FRONT_RIGHT);
        rearLeft = new TalonSRX(Constants.REAR_LEFT);
        rearRight = new TalonSRX(Constants.REAR_RIGHT);

        DTRHelper.configurePIDF(frontLeft, 0.6, 0, 0, 1, Constants.HIGH_GEAR_MAX, Constants.WHEEL_RADIUS, Constants.TICKS_PER_ROTATION, FeedbackDevice.QuadEncoder);
        DTRHelper.configurePIDF(frontRight, 0.6, 0, 0, 1, Constants.HIGH_GEAR_MAX, Constants.WHEEL_RADIUS, Constants.TICKS_PER_ROTATION, FeedbackDevice.QuadEncoder);

        rearLeft.follow(frontLeft);
        rearRight.follow(frontRight);

        navX = new AHRS(SPI.Port.kMXP);
    }

    @Override
    protected void initDefaultCommand() {
        this.setDefaultCommand(new DTRCommand());
    }

    public void xboxDrive() {
        tankDrive(-Robot.oi.getXbox().getY(GenericHID.Hand.kLeft), Robot.oi.getXbox().getY(GenericHID.Hand.kRight), ControlMode.Velocity);
    }

    public void tankDrive(double leftValue, double rightValue, ControlMode mode) {
        if (leftValue >= 0.0) {
            leftValue = leftValue * leftValue;
        } else {
            leftValue = -(leftValue * leftValue);
        }
        if (rightValue >= 0.0) {
            rightValue = rightValue * rightValue;
        } else {
            rightValue = -(rightValue * rightValue);
        }

        if (mode == ControlMode.Velocity) {
            leftValue *= 1049;
            rightValue *= 1049;
        }

        frontLeft.set(mode, leftValue);
        frontRight.set(mode, rightValue);
    }

    public void reset() {
        navX.reset();
        navX.setAngleAdjustment(90);
        tankDrive(0, 0, ControlMode.PercentOutput);
    }
}
