package frc.team5190.robot.vision

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.lang.Thread.sleep
import kotlin.concurrent.thread

object Vision {

    var isTgtVisible = 0L
    var tgtAngle = 0L
    var tgtRange = 0L
    private var visionPort: SerialPort? = null
    private var lastDataUpdated: Long = 0
    private var stopped = false
    private const val BAUD_RATE = 115200

    init {
        thread(name = "Vision") {
            while (!stopped) {
                try {
                    processData()
                    sleep(10)
                } catch (e: Exception) {
                    sleep(50)
                }
            }
        }

//        try {
//            print("[Vision] Starting stream capture...")
//            CameraServer.getInstance().startAutomaticCapture(0).apply {
//                setPixelFormat(VideoMode.PixelFormat.kYUYV)
//            }
//            println(" success!")
//        } catch (e: Exception) {
//            println(" failed!")
//        }
    }

    fun stop() {
        stopped = true

        if (visionPort != null) {
            sendCmd("streamoff")
            visionPort!!.free()
        }
    }

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
        sendCmd(cmd)
        val retval = blockAndCheckForOK(1.0)
        when (retval) {
            0 -> println("[Vision] $cmd OK")
            -1 -> println("[Vision] $cmd produced an error")
            -2 -> println("[Vision] $cmd timed out")
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
        println("[Vision] Wrote $bytes/${cmd.length + 1} bytes, cmd: $cmd")
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
                    lastDataUpdated = System.currentTimeMillis()
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

    private fun connect() {
        try {
            visionPort = SerialPort(BAUD_RATE, SerialPort.Port.kUSB1)

            // In case we start getting data immediately, turn it off
            sendCmd("streamoff")

            // Wait for the incoming stream to stop and read up all the incoming data
            sleep(100)
            if (visionPort!!.bytesReceived > 0) {
                visionPort!!.readString()
            }

            // test with a ping
            val pingResult = sendPing()
            if (pingResult != 0) {
                println("[Vision] JeVois ping test failed!")
                visionPort = null
            } else {
                println("[Vision] JeVois ping test completed! :)")
            }

            // start the stream again
            sendCmd("streamon")
        } catch (e: Exception) {
            visionPort = null
            return
        }
    }

    private val gson = Gson()

    data class VisionTemplate(
            val Track: Long,
            val Angle: Long,
            val Range: Long)

    private fun processData() {
        if (visionPort == null) {
            connect()
        }

        // still not connected
        if (visionPort == null) {
            return
        }

        if (visionPort!!.bytesReceived > 0) {
            // got data
            val string = visionPort!!.readString()
            val obj = gson.fromJson<VisionTemplate>(string)
            // (1) newly acquired target or (2) not too different from previous target or (3) have been getting different values for over 200ms
            if (isTgtVisible == 0L
                    || ((Math.abs(tgtAngle - obj.Angle) < 5 && Math.abs(tgtRange - obj.Range) < 10))
                    || (System.currentTimeMillis() - lastDataUpdated > 200)) {
                lastDataUpdated = System.currentTimeMillis()
                isTgtVisible = obj.Track
                tgtAngle = obj.Angle
                tgtRange = obj.Range
            }
        }
        else if (System.currentTimeMillis() - lastDataUpdated > 200) {
            // no data received for more than 200ms => not tracking any object
            isTgtVisible = 0
            tgtAngle = 0
            tgtRange = 0
        }

        SmartDashboard.putNumber("Angle", tgtAngle.toDouble())
        SmartDashboard.putNumber("Distance", tgtRange.toDouble())
    }
}