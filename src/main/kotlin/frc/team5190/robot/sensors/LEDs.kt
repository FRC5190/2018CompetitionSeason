/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.Robot
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.util.setLEDOutput

object LEDs : Subsystem() {

    private val leds = CANifier(16)

    private var blinkedFor = 0L

    override fun initDefaultCommand() {}

    override fun periodic() {
        if (!Robot.INSTANCE!!.dataRec || !Robot.INSTANCE!!.isEnabled) {
            leds.setLEDOutput(64, 0, 64)
        } else {
            if (IntakeSubsystem.isCubeIn) {
                if (blinkedFor == 0L) blinkedFor = System.currentTimeMillis()
                if (System.currentTimeMillis() % 400 > 200 && System.currentTimeMillis() - blinkedFor < 2000)
                    leds.setLEDOutput(0, 0, 0)
                else
                    leds.setLEDOutput(0, 255, 0)
            } else {
                blinkedFor = 0L
                leds.setLEDOutput(0, 0, 0)
            }
        }
    }
}