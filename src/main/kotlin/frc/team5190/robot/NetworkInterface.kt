/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import frc.team5190.lib.wrappers.FalconRobotBase
import frc.team5190.lib.wrappers.networktables.FalconNetworkTable
import frc.team5190.lib.wrappers.networktables.get

@Suppress("HasPlatformType")
object NetworkInterface {

    val INSTANCE = FalconNetworkTable.getTable("Live Dashboard")

    private val robotX = INSTANCE["Robot X"]
    private val robotY = INSTANCE["Robot Y"]

    private val robotHdg = INSTANCE["Robot Heading"]

    private val pathX = INSTANCE["Path X"]
    private val pathY = INSTANCE["Path Y"]
    private val pathHdg = INSTANCE["Path Heading"]

    private val lookaheadX = INSTANCE["Lookahead X"]
    private val lookaheadY = INSTANCE["Lookahead Y"]

    private val isEnabled = INSTANCE["Is Enabled"]
    private val gameData = INSTANCE["Game Data"]

    private val notifier: Notifier

    init {
        notifier = Notifier {
            robotX.setDouble(Localization.robotPosition.translation.x)
            robotY.setDouble(Localization.robotPosition.translation.y)

            robotHdg.setDouble(Localization.robotPosition.rotation.radians)

            pathX.setDouble(MotionProfileCommand2.pathX)
            pathY.setDouble(MotionProfileCommand2.pathY)
            pathHdg.setDouble(MotionProfileCommand2.pathHdg)

            lookaheadX.setDouble(MotionProfileCommand2.lookaheadX)
            lookaheadY.setDouble(MotionProfileCommand2.lookaheadY)

            isEnabled.setString(if (FalconRobotBase.INSTANCE.isEnabled) "Enabled" else "Disabled")
            gameData.setString(DriverStation.getInstance().gameSpecificMessage ?: "null")
        }

        notifier.startPeriodic(0.02)
    }
}