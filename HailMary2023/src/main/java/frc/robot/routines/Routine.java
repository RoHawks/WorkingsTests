package frc.robot.routines;

import java.util.List;

public class Routine 
{
    private List<StepSequence> mStepSequences;

    public Routine(List<StepSequence> pStepSequences)
    {
        mStepSequences = pStepSequences;
    }

    public void Run()
    {
        for(int i = 0; i < mStepSequences.size(); i++)
        {
            StepSequence stepSequence = mStepSequences.get(i);
            stepSequence.Run();
        }
    } 
    
    public void Reset()
    {
        for(int i = 0; i < mStepSequences.size(); i++)
        {
            StepSequence stepSequence = mStepSequences.get(i);
            stepSequence.Reset();
        }
    }
    
}
