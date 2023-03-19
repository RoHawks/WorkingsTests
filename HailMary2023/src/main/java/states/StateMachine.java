package states;

import java.util.HashMap;



import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robosystems.*;
import universalSwerve.SwerveDrive;
import universalSwerve.SwerveFactory;

public class StateMachine 
{
    private HashMap<States, AState> mStates;
    private States mCurrentState = null;
    private long mStartTime; //ms
    //private StringBuilder mLog = new StringBuilder();

    public StateMachine(XboxController pMainController, XboxController pAlternateController,
    SwerveDrive pSwerveDrive, Intake pIntake, Claw pClaw, Wrist pWrist)
    {     
        mStates = new HashMap<States, AState>();

        
        // GameStartState gameStartState = new GameStartState(pDriveTrain, pIntake, pShooter);
        // mStates.put(gameStartState.GetState(), gameStartState);

        // IntakingState intakingState = new IntakingState(pMainController, pAlternateController, pDriveTrain, pIntake, pShooter, pFeedback, pClimber);
        // mStates.put(intakingState.GetState(), intakingState);

        // PreppingToShootState preppingToShootState = new PreppingToShootState(pMainController, pAlternateController, pDriveTrain, pIntake, pShooter, pFeedback, pClimber);
        // mStates.put(preppingToShootState.GetState(), preppingToShootState);

        // ShootingState shootingState = new ShootingState(pMainController, pAlternateController, pDriveTrain, pIntake, pShooter, pFeedback, pClimber);
        // mStates.put(shootingState.GetState(), shootingState);

        // SeekingClimbState seekingClimbState = new SeekingClimbState(pMainController, pAlternateController, pDriveTrain, pIntake, pShooter, pFeedback, pClimber);
        // mStates.put(seekingClimbState.GetState(), seekingClimbState);

        // ClimbRoutineState climbingRoutineState = new ClimbRoutineState(pClimber, pMainController, pAlternateController);
        // mStates.put(climbingRoutineState.GetState(), climbingRoutineState);

        // mCurrentState = intakingState.GetState();
    }


    public void Reset()
    {
        mStartTime = System.currentTimeMillis();
        //mLog = new StringBuilder();

        //Call this at the beginning of teleop and auto
        //Moves us to GameStart state (which may do nothing if it's already been done)
        if(mCurrentState != null)
        {
            AState currentStateObject = mStates.get(mCurrentState);
            currentStateObject.ExitState();
            
            mCurrentState = States.GameStart;
            AState newCurrentStateObject = mStates.get(mCurrentState);
            newCurrentStateObject.EnterState(null);
        }
    }

    public void Run()
    {
        AState currentStateObject = mStates.get(mCurrentState);
        SmartDashboard.putString("CurrentState", currentStateObject.GetName());
        NextStateInfo nextStateInfo = currentStateObject.Run();
        if(nextStateInfo.GetNextState() != mCurrentState)
        {            
            currentStateObject.ExitState();
            AState newState = mStates.get(nextStateInfo.GetNextState());
            /*
            mLog.append((System.currentTimeMillis() - mStartTime) / 1000);
            mLog.append(": ");
            mLog.append(newState.GetName());
            SmartDashboard.putString("StateMachineLog", mLog.toString());
            */
            newState.EnterState(nextStateInfo.GetNextStateParameter());
            mCurrentState = nextStateInfo.GetNextState();
        }        
    }
    
}
