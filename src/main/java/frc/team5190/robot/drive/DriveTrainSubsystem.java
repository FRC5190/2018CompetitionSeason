package frc.team5190.robot.drive;

import com.ctre.phoenix.drive.DiffDrive;
import com.ctre.phoenix.drive.DriveMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team5190.robot.Constants;
import frc.team5190.robot.OI;

@SuppressWarnings("WeakerAccess")
public class DriveTrainSubsystem extends Subsystem {

    public TalonSRX frontLeft;
    public TalonSRX frontRight;
    public TalonSRX rearLeft;
    public TalonSRX rearRight;

    public AHRS navX;

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

        navX = new AHRS(SPI.Port.kMXP);

        driveBase = new DiffDrive(frontLeft, frontRight);
    }

    @Override
    protected void initDefaultCommand() {
        this.setDefaultCommand(new DriveCommand());
    }

    public void testDrive() {
        driveBase.set(DriveMode.PercentOutput, OI.INSTANCE.getYLeft(), -OI.INSTANCE.getXLeft());
    }

    public void turn(double curve) {
        driveBase.set(DriveMode.PercentOutput, 0, curve);
    }
}
