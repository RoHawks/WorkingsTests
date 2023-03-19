package robosystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; //import static class representing the value of the DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
// import com.revrobotics.CANSparkMax.SparkMaxLimitSwitch.Type;

public class Intake {   

    private CANSparkMax mIntakeMotorTop;
    private CANSparkMax mIntakeMotorBottom;
    private DoubleSolenoid mIntakeDeploy;

    private static PeriodicFrame frameType = PeriodicFrame.kStatus1;
    private static int rate = 50;

    public Intake(CANSparkMax pIntakeMotorTop, CANSparkMax pIntakeMotorBottom, DoubleSolenoid pIntakeDeploy) {
        mIntakeMotorTop = pIntakeMotorTop;
        mIntakeMotorBottom = pIntakeMotorBottom;
        mIntakeDeploy = pIntakeDeploy;

        configureSparkMax(mIntakeMotorTop);
        configureSparkMax(mIntakeMotorBottom);

        
  }
    public static void configureSparkMax(CANSparkMax pSparkMax){
        pSparkMax.restoreFactoryDefaults(true); // If true, burn the flash with the factory default parameters?
        pSparkMax.setSmartCurrentLimit(20);
        pSparkMax.setOpenLoopRampRate(10);
        pSparkMax.setPeriodicFramePeriod(frameType, rate); //not sure what to place as arguments
        pSparkMax.burnFlash();
    }

    public void turnOn(){ //need to actually supply power using the CANSparkMax before any of this can happen
        mIntakeMotorBottom.set(-0.5);
        mIntakeMotorTop.set(-1.0);
        
    }

    public void turnOff(){
        mIntakeMotorTop.set(0);
        mIntakeMotorBottom.set(0);
    }

    public void deploy() {
        mIntakeDeploy.set(kForward);
    }
    public void retract() {
        mIntakeDeploy.set(kReverse);
    }
}
