package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.ProgrammingBoard;

@Autonomous
public class TestAuto extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    enum State {
        START,
        MOVE,
        STOP,
        DONE
    }
    enum State2 {
        START,
        FIFTY,
        SEVENTY_FIVE,
        FULL,
        STOP,
        DONE
    }
    enum TurnState {
        START,
        SERVO,
        DONE
    }
    State pathState;
    State2 pathState2;
    TurnState turnState;
    double lastTime;

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void start() {
        pathState = State.START;
        pathState2 = State2.START;
        turnState = TurnState.START;
        resetRuntime();
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        telemetry.addData("State", pathState);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in State", getRuntime() - lastTime);
        //autoPath();
        //autoPath2();
        autoTurnPath();
        telemetry.update();
    }

    public void autoPath() {
        switch (pathState) {
            case START:
                board.setServoPosition(0.5);
                pathState = State.MOVE;
                lastTime = getRuntime();
                break;
            case MOVE:
                board.setMotorSpeed(0.5);
                if(getRuntime() > lastTime + 2.0) {
                    pathState = State.STOP;
                    lastTime = getRuntime();
                }
                break;
            case STOP:
                board.setMotorSpeed(0.0);
                board.setServoPosition(0.0);
                pathState = State.DONE;
                lastTime = getRuntime();
                break;
            default:
                telemetry.addData("Auto", "Complete");
        }
    }

    public void autoPath2() {
        switch(pathState2) {
            case START:
                board.setMotorSpeed(0.25);
                pathState2 = State2.FIFTY;
                lastTime = getRuntime();
                break;
            case FIFTY:
                if(getRuntime() >= lastTime + 0.25){
                    board.setMotorSpeed(0.5);
                    pathState2 = State2.SEVENTY_FIVE;
                    lastTime = getRuntime();
                }
                break;
            case SEVENTY_FIVE:
                if(getRuntime() >= lastTime + 0.25){
                    board.setMotorSpeed(0.75);
                    pathState2 = State2.FULL;
                    lastTime = getRuntime();
                }
                break;
            case FULL:
                if(getRuntime() >= lastTime + 0.25){
                    board.setMotorSpeed(1.0);
                    pathState2 = State2.STOP;
                }
                break;
            case STOP:
                if(board.isTouchSensorPressed()){
                    board.setMotorSpeed(0.0);
                    pathState2 = State2.DONE;
                }
                break;
            default:
                telemetry.addData("Auto", "Complete");
        }
    }

    public void autoTurnPath() {
        switch(turnState) {
            case START:
                board.setMotorSpeed(0.5);
                board.setServoPosition(0.0);
                turnState = TurnState.SERVO;
                lastTime = getRuntime();
                break;
            case SERVO:
                if(board.getDistance(DistanceUnit.CM) < 10 || getRuntime() > lastTime + 5.0) {
                    board.setMotorSpeed(0.0);
                    board.setServoPosition(0.5);
                    turnState = TurnState.DONE;
                }
                break;
            default:
                telemetry.addData("Auto", "Complete");
        }
    }
}
