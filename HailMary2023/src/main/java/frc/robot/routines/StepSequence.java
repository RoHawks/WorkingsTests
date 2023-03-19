package frc.robot.routines;


import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StepSequence {
    private List<AComponentStep> mComponentSteps;
    private int mCurrentStepNumber;
    private String mName;

    public StepSequence(String pName, List<AComponentStep> pComponentSteps)
    {
        mComponentSteps = pComponentSteps;
        mCurrentStepNumber = 0;
        mName = pName;

    }

    public void Run()
    {
        AComponentStep currentStep = mComponentSteps.get(mCurrentStepNumber);
        SmartDashboard.putString(mName + "_CurrentStep", currentStep.Name());
        SmartDashboard.putBoolean(mName + "_CurrentStepIsComplete", currentStep.IsComplete());
        SmartDashboard.putBoolean(mName + "_CurrentStepAllPrerequisitesComplete", currentStep.AllPrerequisitesComplete());
        SmartDashboard.putBoolean(mName + "_CurrentStepHasBeenStarted", currentStep.HasBeenStarted());
        
        
        
        //this should always be the case, except for initial steps.  So check here for initial steps (and all steps, might as well)
        if(currentStep.AllPrerequisitesComplete())
        {
            SmartDashboard.putString(mName + "_Status", "Running " + currentStep.Name());
            if(mCurrentStepNumber == 0 && !currentStep.HasBeenStarted())
            {
                currentStep.EnterStep();
            }
            currentStep.Run();
            
        }
        else
        {
            SmartDashboard.putString(mName + "_Status", "Waiting for prereqs for " + currentStep.Name());
        }
        if(currentStep.IsComplete()
            && mCurrentStepNumber < mComponentSteps.size() - 1 //there is another step
            && currentStep.AllPrerequisitesComplete()
            && mComponentSteps.get(mCurrentStepNumber + 1).AllPrerequisitesComplete()
            )
        {
            currentStep.EndStep();
            mCurrentStepNumber++;
            AComponentStep newStep = mComponentSteps.get(mCurrentStepNumber);
            newStep.EnterStep();
        }
        if(currentStep.IsComplete() && ! currentStep.AllPrerequisitesComplete())
        {
            //System.out.println(mName + " is waiting on a prereq before moving on.");
        }
    }

    public boolean IsCompete()
    {
        return mCurrentStepNumber == mComponentSteps.size() - 1
        && mComponentSteps.get(mCurrentStepNumber).IsComplete();
    }
    
    public void Reset()
    {
        for(int i = 0; i < mComponentSteps.size(); i++)
        {
            mComponentSteps.get(i).Reset();
        }

        mCurrentStepNumber = 0;
    }
}
