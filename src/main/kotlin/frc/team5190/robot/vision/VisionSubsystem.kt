package frc.team5190.robot.vision

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import kotlin.math.pow
import kotlin.math.sign

object VisionSubsystem : Subsystem() {

    private var visionPort: SerialPort? = null
    private val camDisplacement = 11.25

    /**
     * Returns true when the JeVois sees a target and is tracking it, false otherwise.
     */
    var isTgtVisible = 0L
        private set

    var tgtAngle: Double = 0.0
        get() = correctedAngle()

    var tgtDistance: Double = 0.0
        get() = correctedDistance()

    /**
     * Returns the most recently seen target's angle relative to the camera in degrees
     * Positive means to the Right of center, negative means to the left
     */
    var rawAngle = 0.0
    /**
     * Returns the most recently seen target's range from the camera in inches
     * Range means distance along the ground from camera mount point to observed target
     * Return values should only be positive
     */
    private var rawDistance = 0.0

    /**
     * Constructor (more complex). Opens a USB serial port to the camera, sends a few test commands checking for error,
     * then fires up the user's program and begins listening for target info packets in the background.
     * Pass TRUE to additionally enable a USB camera stream of what the vision camera is seeing.
     */
    init {
        reset()
    }

    private fun reset() {

        if (visionPort != null) {
            visionPort!!.reset()
            visionPort!!.free()
            visionPort = null
        }

        isTgtVisible = 0L
        rawAngle = 0.0
        rawDistance = 0.0

        var retryCounter = 0

        //Retry strategy to get this serial port open.
        //I have yet to see a single retry used assuming the camera is plugged in
        // but you never know.
        while (visionPort == null && retryCounter++ < 10) {
            try {
                print("Vision: Creating JeVois SerialPort...")
                visionPort = SerialPort(BAUD_RATE, SerialPort.Port.kUSB1)
                println("SUCCESS!!")
            } catch (e: Exception) {
                println("FAILED!!")
                sleep(500)
            }
        }

        //Report an error if we didn't get to open the serial port
        if (visionPort == null) {
            println("Vision: Cannot open serial port to JeVois. Not starting vision system.")
            return
        }

        //Test to make sure we are actually talking to the JeVois
        sendCmd("streamoff")

        sleep(100)
        retryCounter = 0
        while (visionPort!!.bytesReceived > 0 && retryCounter++ < 10) {
            visionPort!!.readString()
        }

        if (sendPing() != 0) {
            println("Vision: JeVois ping test failed. Not starting vision system.")
            return
        }
        sendCmd("streamon")
    }

    override fun initDefaultCommand() {}

    /**
     * Send the ping command to the JeVois to verify it is connected
     *
     * @return 0 on success, -1 on unexpected response, -2 on timeout
     */
    private fun sendPing(): Int {
        var retval = -1
        if (visionPort != null) {
            retval = sendCmdAndCheck("ping")
        }

        return retval
    }

    /**
     * Sends a command over serial to the JeVois, waits for a response, and checks that response
     * Automatically ends the line termination character.
     *
     * @param cmd String of the command to send (ex: "ping")
     * @return 0 if OK detected, -1 if ERR detected, -2 if timeout waiting for response
     */
    private fun sendCmdAndCheck(cmd: String): Int {
        var retval = 0
        sendCmd(cmd)
        retval = blockAndCheckForOK(1.0)
        when (retval) {
            0 -> println("Vision: $cmd OK")
            -1 -> println("Vision: $cmd produced an error")
            -2 -> println("Vision: $cmd timed out")
        }
        return retval
    }

    /**
     * Sends a command over serial to JeVois and returns immediately.
     *
     * @param cmd String of the command to send (ex: "ping")
     */
    private fun sendCmd(cmd: String) {
        val bytes: Int = visionPort!!.writeString(cmd + "\n")
        println("Vision: Wrote " + bytes + "/" + (cmd.length + 1) + " bytes, cmd: " + cmd)
    }


    /**
     * Blocks thread execution till we get a response from the serial line
     * or timeout.
     * Return values:
     * 0 = OK in response
     * -1 = ERR in response
     * -2 = No token found before timeout_s
     */
    private fun blockAndCheckForOK(timeout_s: Double): Int {
        var retval = -2
        val startTime = Timer.getFPGATimestamp()
        var testStr = ""
        if (visionPort != null) {
            while (Timer.getFPGATimestamp() - startTime < timeout_s) {
                if (visionPort!!.bytesReceived > 0) {
                    testStr += visionPort!!.readString()
                    if (testStr.contains("OK")) {
                        retval = 0
                        break
                    } else if (testStr.contains("ERR")) {
                        DriverStation.reportError("JeVois reported error:\n" + testStr, false)
                        retval = -1
                        break
                    }

                } else {
                    sleep(10)
                }
            }
        }
        return retval
    }

    override fun periodic() {
        if (visionPort == null)
            return

        try {
            if (visionPort!!.bytesReceived > 0) {
                val string = visionPort!!.readString()
//                println(string)
                val parser = JSONParser()
                val obj = parser.parse(string)
                val jsonObject = obj as JSONObject
                isTgtVisible = jsonObject["Track"] as Long
                if (isTgtVisible == 1L) {
                    rawAngle = jsonObject["Angle"] as Double
                    rawDistance = jsonObject["Range"] as Double

                    SmartDashboard.putNumber("Raw Angle", rawAngle)
                    SmartDashboard.putNumber("Raw Distance", rawDistance)

                    SmartDashboard.putNumber("Corrected Angle", correctedAngle())
                    SmartDashboard.putNumber("Corrected Distance", correctedDistance())

                } else {
                    rawAngle = 0.0
                    rawDistance = 0.0
                }
            }
        } catch (e: Exception) {
        }
    }

    /**
     * Private wrapper around the Thread.sleep method, to catch that interrupted error.
     *
     * @param time_ms
     */
    private fun sleep(time_ms: Int) {
        try {
            Thread.sleep(time_ms.toLong())
        } catch (e: InterruptedException) {
            println("DO NOT WAKE THE SLEEPY BEAST")
        }

    }

    private fun correctedAngle() : Double {
        val adjacentDistance = rawDistance * Math.cos(Math.toRadians(rawAngle))

        val oppositeDistance = adjacentDistance * Math.sin(Math.toRadians(rawAngle)) + camDisplacement


        return Math.toDegrees(Math.atan2(oppositeDistance, adjacentDistance))
    }

    private fun correctedDistance() : Double {
        val adjacentDistance = rawDistance * Math.cos(Math.toRadians(rawAngle))
        val oppositeDistance = adjacentDistance * Math.sin(Math.toRadians(rawAngle)) + camDisplacement
        return Math.sqrt(adjacentDistance.pow(2.0) + oppositeDistance.pow(2.0))
    }

    private val BAUD_RATE = 115200
}