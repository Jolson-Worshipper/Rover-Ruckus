package org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.GamepadClasses;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadClass {
    //creates new gamepad, buttons for each one you need to debounce/toggle
    private Gamepad gamepad;
    public Button Y,RB,LB,X,A,B;
    //Creates new timer
    private double  LastTime = 0;
    private ElapsedTime timer;


    public GamepadClass(Gamepad gamepad) {
        //creates each button needed currently, sets up timer to be at 0
        this.gamepad = gamepad;
        Y = new Button(gamepad.y);
        X = new Button(gamepad.x);
        RB = new Button(gamepad.right_bumper);
        LB = new Button(gamepad.left_bumper);
        B = new Button(gamepad.b);
        A = new Button(gamepad.a);
        timer = new ElapsedTime();
        timer.reset();
    }

    public void update() {
        //if enough time has passed since the last loop, update the buttons
        if (timer.milliseconds() - LastTime > 10){
            Y.updateVar(gamepad.y);
            X.checkToggle(gamepad.x);
            A.updateVar(gamepad.a);
            B.updateVar(gamepad.b);
            RB.checkToggle(gamepad.right_bumper);
            LB.checkToggle(gamepad.left_bumper);
            LastTime = timer.milliseconds();
        }
    }
}