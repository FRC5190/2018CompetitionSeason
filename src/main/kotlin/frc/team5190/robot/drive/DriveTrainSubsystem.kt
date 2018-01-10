package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.team5190.robot.MotorIds
import frc.team5190.robot.OI
import kotlin.math.absoluteValue

enum class DriveMode {
    TANK,
    CURVE
}

class DriveTrainSubsystem : Subsystem() {

    var controlMode = DriveMode.TANK
        set(value) {
            println("Changing DriveMode from $field to $value")
            field = value
        }

    private val frontLeft = WPI_TalonSRX(MotorIds.FRONT_LEFT_VAL)
    private val frontRight = WPI_TalonSRX(MotorIds.FRONT_RIGHT_VAL)

    init {
        println("Init Drive")

        WPI_TalonSRX(MotorIds.REAR_LEFT_VAL).follow(frontLeft)
        WPI_TalonSRX(MotorIds.REAR_RIGHT_VAL).follow(frontRight)

        listOf(frontLeft, frontRight).forEach { it.setNeutralMode(NeutralMode.Brake) }
        println("Drive Done")
    }

    override fun initDefaultCommand() {
        this.defaultCommand = DriveCommand()
    }

    fun drive() {
        OI.xbox.getY(GenericHID.Hand.kLeft).takeIf { it.absoluteValue > 0.2 }?.let { frontRight.set(it) }
        OI.xbox.getY(GenericHID.Hand.kRight).takeIf { it.absoluteValue > 0.2 }?.let { frontLeft.set(-it) }
        if (OI.xbox.backButtonPressed) controlMode = DriveMode.TANK
        if (OI.xbox.startButtonPressed) controlMode = DriveMode.CURVE
    }
}

