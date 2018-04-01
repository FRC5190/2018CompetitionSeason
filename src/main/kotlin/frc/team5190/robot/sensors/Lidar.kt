/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.CANifier
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction

object Lidar {

    private val pwmData = DoubleArray(2)

    private val interpolateFunction: PolynomialSplineFunction

    private val minInputValue: Double
    private val maxInputValue: Double
    private val allowedTolerance = 100.0

    val underScale: Boolean
        get() = minInputValue - allowedTolerance < rawDistance && rawDistance < maxInputValue + allowedTolerance


    val scaleHeight: Double
        get() = interpolateFunction.value(rawDistance)

    val rawDistance: Double
        get() = pwmData[0]

    init {
        val interpolator = LinearInterpolator()

        // X - Raw Sensor Units
        // Y - Height in Inches
        val data = arrayOf(
                5.0 to 6.0,
                7.0 to 10.0
        )

        minInputValue = data.map { it.first }.min()!!
        maxInputValue = data.map { it.first }.max()!!

        interpolateFunction = interpolator.interpolate(data.map { it.first }.toDoubleArray(), data.map { it.second }.toDoubleArray())
    }

    fun periodic() {
        LEDs.canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, pwmData)
    }

}