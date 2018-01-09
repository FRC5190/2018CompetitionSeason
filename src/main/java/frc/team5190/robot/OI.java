package frc.team5190.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team5190.robot.navigation.NAVCommand;

@SuppressWarnings("unused")
public class OI
{
    private XboxController xbox = new XboxController(0);

    public OI()
    {
        JoystickButton buttonA = new JoystickButton(xbox, 1);
        JoystickButton buttonX = new JoystickButton(xbox, 3);
        JoystickButton buttonB = new JoystickButton(xbox, 2);

        buttonX.whenPressed(new NAVCommand());
    }

    public XboxController getXbox()
    {
        return xbox;
    }
}
