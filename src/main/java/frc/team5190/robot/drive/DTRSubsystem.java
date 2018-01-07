package frc.team5190.robot.drive;

import com.ctre.phoenix.drive.DiffDrive;
import com.ctre.phoenix.drive.DriveMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team5190.robot.Robot;
import frc.team5190.robot.util.Constants;
import frc.team5190.robot.util.Maths;

@SuppressWarnings("WeakerAccess")
public class DTRSubsystem extends Subsystem {

    // Clockwork vals

    public TalonSRX frontLeft;
    public TalonSRX frontRight;
    public TalonSRX rearLeft;
    public TalonSRX rearRight;

    public AHRS navX;

    private DiffDrive driveBase;

    public DTRSubsystem() {
        frontLeft   = new TalonSRX(Constants.FRONT_LEFT);
        frontRight  = new TalonSRX(Constants.FRONT_RIGHT);
        rearLeft    = new TalonSRX(Constants.REAR_LEFT);
        rearRight   = new TalonSRX(Constants.REAR_RIGHT);

        frontLeft.set(ControlMode.Velocity, 0);
        frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        frontLeft.setSensorPhase(true);

        frontLeft.config_kP(0, 0.6, 0);
        frontLeft.config_kF(0, Maths.calculateFGain(1, 18.3f, 2, 1440), 0);

        rearLeft.follow(frontLeft);

        frontRight.set(ControlMode.Velocity, 0);
        frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        frontRight.setSensorPhase(true);

        frontRight.config_kP(0, 0.6, 0);
        frontRight.config_kF(0, Maths.calculateFGain(1, 18.3f, 2, 1440), 0);

        rearRight.follow(frontRight);

        navX = new AHRS(SPI.Port.kMXP);

        driveBase = new DiffDrive(frontLeft, frontRight);
    }

    @Override
    protected void initDefaultCommand() {
        this.setDefaultCommand(new DTRCommand());
    }

    public void testDrive() {
        falconTankDrive(-Robot.oi.getXbox().getY(GenericHID.Hand.kLeft), Robot.oi.getXbox().getY(GenericHID.Hand.kRight));
    }

    public void turn(double curve) {
        driveBase.set(DriveMode.PercentOutput, 0, curve);
    }


    public void falconTankDrive(double leftValue, double rightValue)
    {
        if (leftValue >= 0.0) {
            leftValue = leftValue * leftValue;
        }
        else {
            leftValue = -(leftValue * leftValue);
        }
        if (rightValue >= 0.0) {
            rightValue = rightValue * rightValue;
        }
        else {
            rightValue = -(rightValue * rightValue);
        }

        leftValue *= 1049;
        rightValue *= 1049;

        frontLeft.set(ControlMode.Velocity, leftValue);
        frontRight.set(ControlMode.Velocity, rightValue);

    }
}
