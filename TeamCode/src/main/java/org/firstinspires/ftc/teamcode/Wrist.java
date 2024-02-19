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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {


    // Define class members
    double wristPosition = 0.0;


    private OpMode myOpMode;   // gain access to methods in the calling OpMode.

    Servo servo = null;
    public Wrist(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        servo = myOpMode.hardwareMap.get(Servo.class, "left_hand");

        // Initialize to position 0
        runToPosition();
    }

    public boolean runToPosition() {
        boolean anyChange = false;

        if (servo.getPosition() != wristPosition) {
            servo.setPosition(wristPosition);
            anyChange = true;
        }
        myOpMode.telemetry.addData("Wrist position: ", "%.1f", wristPosition);
        return anyChange;
    }

    public void wristDown() {
        wristPosition = 1.0;
        runToPosition();
    }
    public void wristUp() {
        wristPosition = 0.0;
        runToPosition();
    }

    public void go_to_position(double wristPosition) {
        this.wristPosition = wristPosition;
        runToPosition();
    }

    public boolean listen() {

        // move arm according to the right stick y
        if (myOpMode.gamepad2.dpad_up) {
            if (wristPosition > 0.0) {
                if (myOpMode.gamepad2.right_bumper) {
                    wristPosition -= 0.1;
                    ;
                } else {
                    wristPosition = 0.0;
                }
            }

        } else if (myOpMode.gamepad2.dpad_down) {
            if (wristPosition < 1.0) {
                if (myOpMode.gamepad2.right_bumper) {
                    wristPosition += 0.1;
                } else {
                    wristPosition = 1.0;
                }
            }
        }

        if(myOpMode.gamepad2.left_bumper) {
            wristUp();
        }

        return runToPosition();

    }
}
