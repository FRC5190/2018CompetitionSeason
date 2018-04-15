/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.Robot
import frc.team5190.robot.auto.MotionProfileCommand
import frc.team5190.robot.util.ChannelIDs
import frc.team5190.robot.util.DriveConstants
import jaci.pathfinder.Pathfinder
import openrio.powerup.MatchData
import org.apache.commons.math3.stat.regression.SimpleRegression

object Lidar : Subsystem() {

    private val pwmData = DoubleArray(2)

    private var lidarServo = Servo(ChannelIDs.LIDAR_SERVO)

    private val regressionFunction = SimpleRegression()

    // All in inches
    private const val minScaleHeight = 48.0
    private const val maxScaleHeight = 72.0
    private const val allowedTolerance = 3.0

    var underScale = false
        private set

    var scaleHeight = 0.0
        private set

    private var rawDistance = 0.0

    init {
        // X - Raw Sensor Units
        // Y - Height in Inches
        val data = if (DriveConstants.IS_RACE_ROBOT) arrayOf(
                1050.0 to 45.0,
                1500.0 to 55.0,
                1900.0 to 70.0
        )
        else arrayOf(
                1200.0 to 48.0,
                1490.0 to 62.0,
                1700.0 to 70.0
        )

        data.forEach { regressionFunction.addData(it.first, it.second) }
    }

    override fun periodic() {
        Canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, pwmData)

        rawDistance = pwmData[0]
        scaleHeight = regressionFunction.predict(rawDistance)
        underScale = minScaleHeight - allowedTolerance < scaleHeight && scaleHeight < maxScaleHeight + allowedTolerance

        val scaleSide = Robot.INSTANCE!!.scaleSide

        var servoAngle = Pathfinder.boundHalfDegrees(MotionProfileCommand.robotPosition?.let {
            val scalePosition = 27.0 to 13.5 + (if(scaleSide == MatchData.OwnedSide.LEFT) 1.0 else -1.0) * 6.5
            return@let Math.toDegrees(Math.atan2(scalePosition.first - it.first, scalePosition.second - it.second)) + 180 + Pigeon.correctedAngle
        } ?: (if (scaleSide == MatchData.OwnedSide.LEFT) 1.0 else -1.0) * 25.0 + 90.0)

        servoAngle = ((servoAngle + 90) % 360) - 90.0

        lidarServo.angle = if (Robot.INSTANCE!!.isOperatorControl) 90.0 else servoAngle

        SmartDashboard.putNumber("Raw Scale Height", rawDistance)
        SmartDashboard.putNumber("Scale Height", scaleHeight)
        SmartDashboard.putNumber("Servo Angle", servoAngle)
        SmartDashboard.putBoolean("Under Scale", underScale)
    }

    override fun initDefaultCommand() {}
}