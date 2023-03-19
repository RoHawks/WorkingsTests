// package states;

// import robotcode.driving.DriveTrain;
// import robotcode.intake.Intake;
// import robotcode.shooting.Shooter;
// import robotcode.stateMachine.Code2023.AState2023;
// import robotcode.stateMachine.Code2023.NextStateInfo2023;
// import robotcode.stateMachine.Code2023.States2023;

// public class GameStartState2023 extends AState
// {
     
//     private DriveTrain mDriveTrain;
//     private Intake mIntake;
//     private Shooter mShooter;
//     private boolean mHasCompletedGameStart = false;

//     public GameStartState2023 (
//         DriveTrain pDriveTrain, Intake pIntake, Shooter pShooter)
//     {        
//         mDriveTrain = pDriveTrain;
//         mIntake = pIntake;
//         mShooter = pShooter;
//     }
    
//     @Override
//     protected String GetName() {
        
//         return "GameStart";
//     }

//     @Override 
//     public void EnterState(Object pEntryParameter)
//     {
//         super.EnterState(pEntryParameter);
        
//     }

//     private long INTAKE_DEPLOYMENT_TIME = 1000;//ms

//     @Override
//     protected NextStateInfo2023 Run() 
//     {
//         if(mHasCompletedGameStart)        
//         {
//             //this will be true when we transition from auto to teleop
//             //and we don't want to really do the game start routines.
//             return new NextStateInfo2023(States2023.Intaking);
//         }
//         else
//         {
//             boolean intakeFinishedGameStart;
//             if(this.GetTimeSinceEntry() < INTAKE_DEPLOYMENT_TIME)
//             {
//                 //mIntake.Deploy();
//                 mIntake.StopDeploymentMotors();
//                 intakeFinishedGameStart = false;
//             }
//             else
//             {
//                 mIntake.StopDeploymentMotors();
//                 intakeFinishedGameStart = true;
//             }

//             boolean shooterFinishedGameStart = mShooter.GameStart();

//             if(intakeFinishedGameStart && shooterFinishedGameStart)
//             {
//                 mHasCompletedGameStart = true;
//                 return new NextStateInfo2023(States2023.Intaking);
//             }
//             else
//             {
//                 return new NextStateInfo2023(States2023.GameStart);
//             }
//         }
//     }

//     @Override
//     protected States2023 GetState() {
//         return States2023.GameStart;
//     }
    
// }
