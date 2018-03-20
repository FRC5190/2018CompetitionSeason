/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.Robot
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.util.setLEDOutput

object LEDs : Subsystem() {

    private val leds = CANifier(0)

    private var intakeLEDSolid = false
    private var blinked = false
    private var iterator = 0

    override fun initDefaultCommand() {}

    override fun periodic() {
        if (!Robot.INSTANCE!!.dataRec) {
            leds.setLEDOutput(255, 255, 255)
        } else if (Robot.INSTANCE!!.isEnabled) {

            if (IntakeSubsystem.isCubeIn) {
                if (!intakeLEDSolid) {
                    if ((iterator % 5 == 0)) {
                        if (blinked) leds.setLEDOutput(0, 0, 0) else leds.setLEDOutput(0, 255, 0)
                        blinked = if (blinked) false else true
                    }
                    iterator++

                    // 4 seconds have passed
                    if (iterator > 200)  {
                       intakeLEDSolid = true
                    }
                }
                else {
                    leds.setLEDOutput(0, 255, 0)
                }
            }
            else if (Robot.INSTANCE!!.alliance == DriverStation.Alliance.Blue) {
                if (iterator != 0) iterator = 0
                leds.setLEDOutput(0, 0, 255)
            } else {
                if (iterator != 0) iterator = 0
                leds.setLEDOutput(255, 0, 0)
            }
        }
    }
}