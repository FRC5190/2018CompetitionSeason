package frc.team5190.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

@SuppressWarnings("unused")
public class OI {
    private XboxController xbox = new XboxController(0);

    OI() {
        JoystickButton buttonA = new JoystickButton(xbox, 1);
        JoystickButton buttonX = new JoystickButton(xbox, 3);
        JoystickButton buttonB = new JoystickButton(xbox, 2);
    }

    public XboxController getXbox() {
        return xbox;
    }
}
