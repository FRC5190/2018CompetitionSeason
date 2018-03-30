/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.sensors

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.Robot
import frc.team5190.robot.climb.ClimbSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.util.setLEDOutput
import java.awt.Color

object LEDs : Subsystem() {

    private val canifier = CANifier(16)

    private val COLOR_CLEAR = Color(0, 0, 0)
    private val COLOR_RED = Color(255, 0, 0)
    private val COLOR_ORANGE = Color(255, 100, 0)
    private val COLOR_PURPLE = Color(64, 0, 128)
    private val COLOR_GREEN = Color(0, 255, 0)

    private var blinkedFor = 0L

    override fun initDefaultCommand() {}

    override fun periodic() {
        canifier.setLEDOutput(if (!Robot.INSTANCE!!.dataRec || !Robot.INSTANCE!!.isEnabled)
            COLOR_CLEAR
        else when {
            !ElevatorSubsystem.closedLpControl -> if (System.currentTimeMillis() % 600 > 300) COLOR_CLEAR else COLOR_RED
            ClimbSubsystem.climbState -> if (System.currentTimeMillis() % 600 > 300) COLOR_CLEAR else COLOR_ORANGE
            IntakeSubsystem.isCubeIn -> {
                if (blinkedFor == 0L) blinkedFor = System.currentTimeMillis()
                if (System.currentTimeMillis() % 400 > 200 && System.currentTimeMillis() - blinkedFor < 2000) COLOR_CLEAR
                else if (Robot.INSTANCE!!.isAutonomous) COLOR_PURPLE else COLOR_GREEN
            }
            else -> {
                blinkedFor = 0L
                COLOR_CLEAR
            }
        })
    }
}