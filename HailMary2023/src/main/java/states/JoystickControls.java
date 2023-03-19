package states;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class JoystickControls implements Controls
{
    private XboxController mMainController;
    private XboxController mAlternateController;

    public JoystickControls( XboxController pMainController, XboxController pAlternateController)
    {
        mMainController = pMainController;
        mAlternateController = pAlternateController;
    }

    public boolean GetDeployIntake()
    {
        return mAlternateController.getLeftBumper();
    }

    public boolean GetRetractIntake()
    {
        return mAlternateController.getRightBumper();
    }

    public boolean GetGrabIntakenPiece()
    {
        return mAlternateController.getXButton();
    }

    public boolean GetDropHeldPiece()
    {
        return mAlternateController.getYButton();
    }

    public boolean GetPrepareForHighCone()
    {
        return true; // GIVE ACUTAL LATER
    }

    public boolean GetPrepareForMiddleCone()
    {
        return true; // GIVE ACUTAL LATER
    }

    public boolean GetPrepareForHighCube()
    {
        return true; // GIVE ACUTAL LATER
    }

    public boolean GetPrepareForMiddleCube()
    {
        return true; // GIVE ACUTAL LATER
    }
    
    public boolean GetPrepareForLowPiece()
    {
        return true; // GIVE ACUTAL LATER
    }

    public boolean GetGoToDesiredScoringArmPosition()
    {
        return mMainController.getLeftBumper(); // GIVE ACUTAL LATER
    }

    public boolean GetGoToIntaking()
    {
        return true;
    }

    public boolean GetGoToDrivingToScoring()
    {
        return true;
    }


    public double GetSwerveXComponent()
    {
        return mMainController.getLeftX();
    }

    public double GetSwerveYComponent()
    {
        return mMainController.getLeftY();
    }

    public double GetSwerveLinearSpeed()
    {
        //square to allow for easy speed control on low speeds
        double triggerValue = mMainController.getRightTriggerAxis();
        return triggerValue * triggerValue;
    }

    public double GetSwerveRotationalSpeed()
    {
        return mMainController.getRightX();
    }


    public boolean GetSwerveModeSwitch()
    {
        return mMainController.getStartButton();
    }

    public SwerveNudgingDirection GetSwerveNudgingDirection()
    {
        if(mMainController.getAButton())
        {
            return SwerveNudgingDirection.NORTH;
        }
        else if(mMainController.getXButton())
        {
            return SwerveNudgingDirection.EAST;
        }
        else if(mMainController.getYButton())
        {
            return SwerveNudgingDirection.SOUTH;
        }
        else if(mMainController.getBButton())
        {
            return SwerveNudgingDirection.WEST;
        }
        else
        {
            return SwerveNudgingDirection.NONE;
        }
    }

    
    


    public SwerveGoalTracking GetSwerveGoalTracking()
    {
        if(mAlternateController.getAButton())
        {
            return SwerveGoalTracking.NORTH_WEST;
        }
        else if(mAlternateController.getBButton())
        {
            return SwerveGoalTracking.NORTH_EAST;
        }
        else if(mAlternateController.getPOV() == 0)
        {
            return SwerveGoalTracking.SOUTH_EAST;
        }
        else if(mAlternateController.getLeftBumper())
        {
            return SwerveGoalTracking.SOUTH_WEST;
        }
        else
        {
            return SwerveGoalTracking.NONE;
        }
    }


}