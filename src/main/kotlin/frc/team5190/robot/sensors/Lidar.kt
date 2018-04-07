/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.apache.commons.math3.stat.regression.SimpleRegression

object Lidar : Subsystem() {

    private val pwmData = DoubleArray(2)

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
        val data = arrayOf(
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

        SmartDashboard.putNumber("Raw Scale Height", rawDistance)
        SmartDashboard.putNumber("Scale Height", scaleHeight)
        SmartDashboard.putBoolean("Under Scale", underScale)
    }

    override fun initDefaultCommand() {}
}