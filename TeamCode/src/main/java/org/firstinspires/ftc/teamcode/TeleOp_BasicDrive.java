/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="NerdyBirds: Basic Op Mode", group="Iterative Opmode")
//@Disabled
public class TeleOp_BasicDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rearLift = null;
    private Servo dropperServo = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Establish hardware map using configured device names
        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        rearLift = hardwareMap.get(DcMotor.class, "rearLift");
        dropperServo = hardwareMap.get(Servo.class, "dropperServo");

        // Set DC motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLift.setDirection(DcMotor.Direction.FORWARD);

        // Set DC motor zero-power behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Clear encoder data
        rearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Configurable values
        double DROPPER_CLOSED_POSITION = 0.2;
        double DROPPER_OPENED_POSITION = 0.8;
        double LIFT_UP_POWER = 1;
        double LIFT_DOWN_POWER = -1;
        double SPEED_BOOSTER = 1;
        double SPEED_DEFAULT = 2;
        double SPEED_LIMITER = 4;
        int ERECT_LIMIT = 10000;

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Adds the boost button
        double speedModifier;
        if (gamepad1.right_bumper){
            speedModifier = SPEED_BOOSTER;
        } else if (gamepad1.left_bumper) {
            speedModifier = SPEED_LIMITER;
        } else {
            speedModifier = SPEED_DEFAULT;
        }

        // Calculate power for drive wheels
        leftPower  = -gamepad1.left_stick_y / speedModifier;
        rightPower = -gamepad1.right_stick_y / speedModifier;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Moves the lift up and down
        if (gamepad2.dpad_up){
            if (rearLift.getCurrentPosition() < ERECT_LIMIT){        // Makes sure the lift doesn't go too high
                rearLift.setPower(LIFT_UP_POWER);
            } else {
                rearLift.setPower(0);
            }
        } else if (gamepad2.dpad_down) {
            if (rearLift.getCurrentPosition() > 10){                 // Makes sure the lift doesn't go too low
                rearLift.setPower(LIFT_DOWN_POWER);
            } else {
                rearLift.setPower(0);
            }
        } else {
            rearLift.setPower(0);
        }

        // Control the dropper servo
        double dropperServoPos = dropperServo.getPosition();
        if (gamepad2.x){
            dropperServo.setPosition(DROPPER_CLOSED_POSITION);
        } else if (gamepad2.b){
            dropperServo.setPosition(DROPPER_OPENED_POSITION);
        }

          // Clears encoder values (for testing/debugging purposes ONLY)
        if (gamepad2.back) {
            rearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Pull position/encoder values for telemetry
        int leftWheelPos = leftDrive.getCurrentPosition();
        int rightWheelPos = rightDrive.getCurrentPosition();
        int rearLiftPos = rearLift.getCurrentPosition();
        dropperServoPos = dropperServo.getPosition();

        // Send data back to the Driver Station
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Servo Position", dropperServoPos);
        telemetry.addData("Rear Lift Position", rearLiftPos);
        telemetry.addData("Left Wheel Position", leftWheelPos);
        telemetry.addData("Right Wheel Position", rightWheelPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
