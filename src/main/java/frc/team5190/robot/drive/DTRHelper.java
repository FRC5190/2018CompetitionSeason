package frc.team5190.robot.drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team5190.robot.util.Maths;

class DTRHelper {

    static void configurePIDF(TalonSRX motor, double p, double i, double d, double power, double velocity, double wheelSize, double sensorUnitsPerRotation, FeedbackDevice dev) {
        configurePIDF(motor, p, i, d, Maths.calculateFGain(power, velocity, wheelSize, sensorUnitsPerRotation));
        motor.configSelectedFeedbackSensor(dev, 0, 0);
    }

    @SuppressWarnings("unused")
    static void configurePIDF(TalonSRX motor, double p, double i, double d, double power, double rpm, double sensorUnitsPerRotation, FeedbackDevice dev) {
        configurePIDF(motor, p, i, d, Maths.calculateFGain(power, rpm, sensorUnitsPerRotation));
        motor.configSelectedFeedbackSensor(dev, 0, 0);
    }

    private static void configurePIDF(TalonSRX motor, double p, double i, double d, double f) {
        motor.config_kP(0, p, 0);
        motor.config_kI(0, i, 0);
        motor.config_kD(0, d, 0);
        motor.config_kF(0, f, 0);
    }
}
