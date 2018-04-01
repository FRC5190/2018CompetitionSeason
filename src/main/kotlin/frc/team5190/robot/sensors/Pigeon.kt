/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.sensors.PigeonIMU
import frc.team5190.robot.util.TIMEOUT
import jaci.pathfinder.Pathfinder

/**
 * Creates a Pigeon singleton object
 */
object Pigeon : PigeonIMU(17) {

    init {
        reset()
    }

    var angleOffset = 0.0

    val correctedAngle: Double
        get() {
            val ypr = DoubleArray(3)
            getYawPitchRoll(ypr)
            return Pathfinder.boundHalfDegrees(ypr[0] + angleOffset)
        }


    fun reset(): ErrorCode = setYaw(0.0, TIMEOUT)
}