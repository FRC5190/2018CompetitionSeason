/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction

object Lidar : Subsystem() {

    private val pwmData = DoubleArray(2)

    private val interpolateFunction: PolynomialSplineFunction

    // All in inches
    private val minScaleHeight = 48.0
    private val maxScaleHeight = 72.0
    private val allowedTolerance = 3.0

    var underScale = false
        private set

    var scaleHeight = 0.0
        private set

    private var rawDistance = 0.0

    init {
        val interpolator = LinearInterpolator()

        // X - Raw Sensor Units
        // Y - Height in Inches
        val data = arrayOf(
                1200.0 to 48.0,
                1490.0 to 59.0,
                1790.0 to 69.0

        )

        interpolateFunction = interpolator.interpolate(data.map { it.first }.toDoubleArray(), data.map { it.second }.toDoubleArray())
    }

    override fun periodic() {
        Canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, pwmData)

        rawDistance = pwmData[0]
        if (interpolateFunction.isValidPoint(rawDistance)) {
            // If the values are within our calibrated value range, then calculate the height of scale and if bot is under it
            scaleHeight = interpolateFunction.value(rawDistance)
            underScale = minScaleHeight - allowedTolerance < scaleHeight && scaleHeight < maxScaleHeight + allowedTolerance
        } else {
            underScale = false
        }

        SmartDashboard.putNumber("Raw Scale Height", rawDistance)
        SmartDashboard.putNumber("Scale Height", scaleHeight)
        SmartDashboard.putBoolean("Under Scale", underScale)
    }

    override fun initDefaultCommand() {}
}