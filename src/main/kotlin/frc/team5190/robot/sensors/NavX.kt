/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.I2C
import jaci.pathfinder.Pathfinder

/**
 * Creates a NavX singleton object
 */
object NavX : AHRS(I2C.Port.kMXP) {

    var angleOffset = 0.0

    val correctedAngle: Double
        get() = Pathfinder.boundHalfDegrees(yaw + angleOffset)

    init {
        zeroYaw()
    }

}