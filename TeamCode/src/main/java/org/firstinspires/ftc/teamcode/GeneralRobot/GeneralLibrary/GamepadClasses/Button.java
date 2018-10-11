package org.firstinspires.ftc.teamcode.GeneralRobot.GeneralLibrary.GamepadClasses;

public class Button {
    //sets up booleans for the gamepad
    private boolean gamepadValue;
    //toggleBool is for the on/off switch, toggle is to see if the gamepad was pressed down last loop
    private boolean toggle,toggleBool = true;

    //Takes in starting gamepad value
    public Button(boolean gamepadValue) {
        this.gamepadValue = gamepadValue;
    }

    //updates the current gamepad value (so that you can use debouncing)
    public void updateVar(boolean gamepadValue){
        this.gamepadValue = gamepadValue;
    }

    //checks if the toggle should be switched and updates the toggleBool accordingly
    public void checkToggle(boolean gamepadValue){
        //gets current value
        this.gamepadValue = gamepadValue;
        //if it wasn't pressed down last loop and it is pressed now, set the toggle to the opposite and set toggle to false
        if(toggle &&  gamepadValue){
            toggle = false;
            toggleBool = !toggleBool;
        }
        //if it was pressed down last loop and no longer is, set toggle to true
        else if(!toggle && !gamepadValue)
            toggle = true;
    }
    //return whether the toggle is on/off
    public boolean getToggle(){
        return toggleBool;
    }

    //returns the current value accounting for debouncing
    public boolean getVal(){
        return gamepadValue;
    }
}