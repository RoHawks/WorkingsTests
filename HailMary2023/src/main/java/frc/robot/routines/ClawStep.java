package frc.robot.routines;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawStep extends AComponentStep{


    private DoubleSolenoid mLeftFinger;
    private DoubleSolenoid mRightFinger;
    private boolean mShouldClose;
    private long TIME_TO_COMPLETE_MS = 1000;

    public ClawStep(String pName, List<StepPrequisite> pPrerequisites, DoubleSolenoid pLeftFinger, DoubleSolenoid pRightFinger, boolean pShouldClose)
    {
        super(pName, pPrerequisites);
        mLeftFinger = pLeftFinger;
        mRightFinger = pRightFinger;
        mShouldClose = pShouldClose;
    }

    @Override
    public void Run() {
        //System.out.println("Running Claw Step:  " + mName);
        mLeftFinger.set(mShouldClose ? Value.kReverse : Value.kForward);
        mRightFinger.set(mShouldClose ? Value.kReverse : Value.kForward);
        
    }

    @Override
    public double DistanceFromCompletion() {        
        if(IsComplete())
        {
            return 0;
        }
        else
        {
            return 1000;
        }
    }

    @Override
    public boolean IsComplete() {
        
        if(!mHasBeenStarted)
        {
            return false;
        }
        if(mName.equals("Grab"))
        {
            SmartDashboard.putNumber("GrabTimerCheck", (System.currentTimeMillis() - this.mEntryTime));
        }
        if(mHasBeenCompleted)
        {
            return true;
        }        
        mHasBeenCompleted = ((System.currentTimeMillis() - this.mEntryTime) > TIME_TO_COMPLETE_MS);
        return mHasBeenCompleted;
    }
    
}
