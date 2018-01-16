package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.drive.DifferentialDrive

class FalconDrive(val leftMotors: List<WPI_TalonSRX>,
                  val rightMotors: List<WPI_TalonSRX>) : DifferentialDrive(leftMotors[0], rightMotors[0]) {

    val leftMaster = leftMotors[0]
    val leftSlaves = leftMotors.subList(1, leftMotors.size)

    val rightMaster = rightMotors[0]
    val rightSlaves = rightMotors.subList(1, rightMotors.size)

    val allMotors = listOf(*leftMotors.toTypedArray(), *rightMotors.toTypedArray())

    init {
        leftSlaves.forEach { it.follow(leftMaster) }
        rightSlaves.forEach { it.follow(rightMaster) }

        allMotors.forEach { it.setNeutralMode(NeutralMode.Brake) }
    }

}