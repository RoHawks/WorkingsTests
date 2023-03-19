package robosystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;

public class Wrist
{
    private CANSparkMax mWristMotor;

    public Wrist(CANSparkMax pWristMotor)
    {
        mWristMotor = pWristMotor;
        RelativeEncoder x = pWristMotor.getEncoder();
    }

    /** Calls CANSparkMax set method */
    public void wristSet(double pAmount)
    {

    }

    
}