package frc.team5190.robot.drive;

import com.ctre.phoenix.drive.DiffDrive;
import com.ctre.phoenix.drive.DriveMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team5190.robot.Constants;
import frc.team5190.robot.OI;

@SuppressWarnings("WeakerAccess")
public class DriveTrainSubsystem extends Subsystem {

    public TalonSRX frontLeft;
    public TalonSRX frontRight;
    public TalonSRX rearLeft;
    public TalonSRX rearRight;

    private DiffDrive driveBase;

    public DriveTrainSubsystem() {
        frontLeft   = new TalonSRX(Constants.INSTANCE.getFrontLeftMotorVal());
        frontRight  = new TalonSRX(Constants.INSTANCE.getFrontRightMotorVal());
        rearLeft    = new TalonSRX(Constants.INSTANCE.getRearLeftMotorVal());
        rearRight   = new TalonSRX(Constants.INSTANCE.getRearRightMotorVal());

        frontLeft.setInverted(true);
        rearLeft.setInverted(true);

        frontLeft.set(ControlMode.PercentOutput, 0);
        rearLeft.follow(frontLeft);

        frontRight.set(ControlMode.PercentOutput, 0);
        rearRight.follow(frontRight);

        driveBase = new DiffDrive(frontLeft, frontRight);
    }

    @Override
    protected void initDefaultCommand() {
        this.setDefaultCommand(new DriveCommand());
    }

    public void testDrive() {
        driveBase.set(DriveMode.PercentOutput, OI.INSTANCE.getXbox().getY(GenericHID.Hand.kLeft), -OI.INSTANCE.getXbox().getX(GenericHID.Hand.kLeft));
    }
}
