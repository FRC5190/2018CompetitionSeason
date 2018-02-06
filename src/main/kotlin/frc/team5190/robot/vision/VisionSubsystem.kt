package frc.team5190.robot.vision

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.command.Subsystem
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser

object VisionSubsystem : Subsystem() {
    private var visionPort: SerialPort? = null

    private var tracking = false
    /**
     * Returns true when the JeVois sees a target and is tracking it, false otherwise.
     */
    var isTgtVisible = 0L
        private set
    /**
     * Returns the most recently seen target's angle relative to the camera in degrees
     * Positive means to the Right of center, negative means to the left
     */
    var tgtAngle_Deg = 0L
        private set
    /**
     * Returns the most recently seen target's range from the camera in inches
     * Range means distance along the ground from camera mount point to observed target
     * Return values should only be positive
     */
    var tgtRange_in = 0.0
        private set

    /**
     * Constructor (more complex). Opens a USB serial port to the JeVois camera, sends a few test commands checking for error,
     * then fires up the user's program and begins listening for target info packets in the background.
     * Pass TRUE to additionaly enable a USB camera stream of what the vision camera is seeing.
     */
    init {
        reset()
    }

    fun reset() {

        if (visionPort != null) {
            visionPort!!.reset()
            visionPort!!.free()
            visionPort = null
        }

        tracking = false
        isTgtVisible = 0L
        tgtAngle_Deg = 0L
        tgtRange_in = 0.0

        var retry_counter = 0

        //Retry strategy to get this serial port open.
        //I have yet to see a single retry used assuming the camera is plugged in
        // but you never know.
        while (visionPort == null && retry_counter++ < 10) {
            try {
                print("Creating JeVois SerialPort...")
                visionPort = SerialPort(BAUD_RATE, SerialPort.Port.kUSB1)
                println("SUCCESS!!")
            } catch (e: Exception) {
                println("FAILED!!")
                sleep(500)
                println("Retry " + Integer.toString(retry_counter))
            }

        }

        //Report an error if we didn't get to open the serial port
        if (visionPort == null) {
            DriverStation.reportError("Cannot open serial port to JeVois. Not starting vision system.", false)
            return
        }

        //Test to make sure we are actually talking to the JeVois
        if (sendPing() != 0) {
            DriverStation.reportError("JeVois ping test failed. Not starting vision system.", false)
            return
        }

        try {
            print("Starting JeVois Cam Stream...")
//            val visionCam = UsbCamera ("Frc5190Cam", 0)
//            visionCam.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 480, 30)
//            val camServer = MjpegServer ("Frc5190CamServer", 1180)
//            camServer.source = visionCam

//            CameraServer.getInstance().startAutomaticCapture(0)
            println("SUCCESS!!")
        } catch (e: Exception) {
            DriverStation.reportError("Cannot start camera stream from JeVois", false)
        }

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
        if (retval == 0) {
            println(cmd + " OK")
        } else if (retval == -1) {
            println(cmd + " Produced an error")
        } else if (retval == -2) {
            println(cmd + " timed out")
        }
        return retval
    }

    /**
     * Sends a command over serial to JeVois and returns immediately.
     *
     * @param cmd String of the command to send (ex: "ping")
     */
    private fun sendCmd(cmd: String) {
        val bytes: Int
        bytes = visionPort!!.writeString(cmd + "\n")
        println("wrote " + bytes + "/" + (cmd.length + 1) + " bytes, cmd: " + cmd)
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

        val parser = JSONParser()
        //System.out.println("Testing Parser: " + packet);
        try {
            val string = visionPort!!.readString()
//            println(string)
            val obj = parser.parse(string)

            val jsonObject = obj as JSONObject
            isTgtVisible = jsonObject["Track"] as Long
            tgtAngle_Deg = jsonObject["Angle"] as Long
            tgtRange_in = jsonObject["Range"] as Double
//            println("[$isTgtVisible, $tgtAngle_Deg, $tgtRange_in")
        } catch (e: Exception) {
//            println(e.message)
            isTgtVisible = 0
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

    fun startTracking() {
        tracking = true
    }

    fun stopTracking() {
        tracking = false
        isTgtVisible = 0
    }

    private val BAUD_RATE = 115200
}