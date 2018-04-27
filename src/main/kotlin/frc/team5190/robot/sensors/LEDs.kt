/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.Robot
import frc.team5190.robot.climb.ClimbSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.util.setLEDOutput
import java.awt.Color

object LEDs : Subsystem() {

    private val COLOR_CLEAR = Color(0, 0, 0)
    private val COLOR_RED = Color(255, 0, 0)
    private val COLOR_ORANGE = Color(255, 100, 0)
    private val COLOR_AUTO = Color(255, 17, 115)
    private val COLOR_GREEN = Color(0, 255, 0)
    private val COLOR_GREEN_LESS_INTENSE = Color(0, 100, 0)

    private var blinkedFor = 0L

    override fun initDefaultCommand() {}

    // Periodic 50hz loop
    override fun periodic() {
        Canifier.setLEDOutput(if (!Robot.INSTANCE!!.fmsDataReceived)
            COLOR_CLEAR
        else when {
            // Green Light when disabled
            Robot.INSTANCE!!.isDisabled -> COLOR_GREEN_LESS_INTENSE

            // Blinking orange when climbing
            ClimbSubsystem.climbState -> if (System.currentTimeMillis() % 800 > 400) COLOR_CLEAR else COLOR_ORANGE

            // Blinking red in sensorless mode
            !ElevatorSubsystem.closedLpControl -> if (System.currentTimeMillis() % 600 > 300) COLOR_CLEAR else COLOR_RED

            // Blink for 3 seconds then solid green when cube is in
            IntakeSubsystem.isCubeIn -> {
                if (blinkedFor == 0L) blinkedFor = System.currentTimeMillis()
                if (System.currentTimeMillis() % 400 > 200 && System.currentTimeMillis() - blinkedFor < 2000) COLOR_CLEAR
                else if (Robot.INSTANCE!!.isAutonomous) COLOR_AUTO else COLOR_GREEN
            }

            // Clear in all other cases
            else -> {
                blinkedFor = 0L
                COLOR_CLEAR
            }
        })
    }
}