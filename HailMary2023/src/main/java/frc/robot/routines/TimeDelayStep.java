package frc.robot.routines;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TimeDelayStep extends AComponentStep {

    private long mTimeMs;

    public TimeDelayStep(String pName, List<StepPrequisite> pPrerequisites, long pTimeMs)
    {
        super(pName, pPrerequisites);
        mTimeMs = pTimeMs;
    }

    @Override
    public void Run() {
        
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
        SmartDashboard.putNumber(mName + "_ElapsedTime", (System.currentTimeMillis() - mEntryTime));
        return (System.currentTimeMillis() - mEntryTime) > mTimeMs;
        
    }
    
}
