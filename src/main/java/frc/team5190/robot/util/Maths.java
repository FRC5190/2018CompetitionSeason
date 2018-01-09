package frc.team5190.robot.util;

public class Maths {

    public static double calculateFGain(double power, double feetPerSecond, double wheelSize, double sensorUnitsPerRotation) {
        return (power * 1023) / ((feetPerSecond * 12.0) / (wheelSize * Math.PI * 2) / 10.0f * sensorUnitsPerRotation);
    }

    public static double calculateFGain(double power, double rpm, double sensorUnitsPerRotation) {
        return (power * 1023) / (rpm / 60 / 10.0 * sensorUnitsPerRotation);
    }

    public static double feetPerSecondToRPM(double feetPerSecond, float wheelRadius) {
        return (feetPerSecond * 12.0f) / (wheelRadius * Math.PI * 2) * 60f;
    }

    public static double ftm(double f) {
        return f * 0.3048;
    }
}
