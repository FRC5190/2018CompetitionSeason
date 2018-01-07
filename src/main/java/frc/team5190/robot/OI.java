package frc.team5190.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

@SuppressWarnings("unused")
public class OI
{
    private XboxController xbox = new XboxController(0);

    public OI()
    {
        JoystickButton buttonA = new JoystickButton(xbox, 1);
    }

    public XboxController getXbox()
    {
        return xbox;
    }
}
