package frc.robot.routines;

public class StepPrequisite {
    private AComponentStep mPrequisite;
    private double mPassPositiveBound;
    private double mPassNegativeBound;

    public StepPrequisite(AComponentStep pPrerequisite, double pPassPositiveBound, double pPassNegativeBound)
    {
        mPrequisite = pPrerequisite;
        mPassPositiveBound = pPassPositiveBound;
        mPassNegativeBound = pPassNegativeBound;
    }

    public boolean IsCompleteEnough()
    {
        double distanceFromComplete = mPrequisite.DistanceFromCompletion();
        //System.out.println("distanceFromCompete " + distanceFromComplete + ", hasBeenEnded" + mPrequisite.HasBeenEnded());
        return mPrequisite.HasBeenStarted()
        && 
        (mPrequisite.HasBeenEnded() || 
        (distanceFromComplete <= mPassPositiveBound && distanceFromComplete >= mPassNegativeBound));
    }
    
    
}
