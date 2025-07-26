package org.firstinspires.ftc.teamcode.SamplesDetection;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp(name = "HuskyLens Telemetry")
public class HuskyLensGJ extends LinearOpMode{

    private HuskyLens huskyLens;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode(){
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");



        //checking for problems with the huskylens
        if(!huskyLens.knock()){
            telemetry.addData(">>","problem with husky");
        }else{
            //can we please keep this starting message
            telemetry.addData(">>","press the holy button of starting");
        }
        telemetry.update();
        waitForStart();

        //setting the detection algorithm to use
        //using this specific algorithms because you can train the lens
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);

        while(opModeIsActive()){

            int BlueCounter = 0;
            int YellowCounter = 0;
            int RedCounter = 0;

            //this is an array of block objects with each block
            //representing something the huskylens detected
            //huskyLens.blocks() returns a list of all objects it detects in this moment
            HuskyLens.Block[] blocks = huskyLens.blocks();

            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("distance: ",distance);

            //given that the array is made up of all the blocks in the pov the length
            //of the length of the array will give the number of blocks
            telemetry.addData("Block count", blocks.length);


            //this for loop processes each block in the array
            for(int a = 0; a<blocks.length; a++){
                //grabs the a'th block in the array
                HuskyLens.Block currentBlock = blocks[a];

                String color;
                //I love switches. legit just started using them more and now
                //if statements are so inferior
                switch(currentBlock.id){
                    case 1: color = "Red";
                        RedCounter++;
                        break;
                    case 2: color = "Blue";
                        BlueCounter++;
                        break;
                    case 3: color = "Yellow";
                        YellowCounter++;
                        break;
                    default: color = "idk what this dohickory is";
                        break;
                }



                //Have to add some more telemetry statements
                //the statements would output the percentage of each type of
                //game piece there is
                //there should also be more statements determining the closes block
                //maybe there shoud be multiple statements for determining the distance
                //between every block




            }





        }



    }




}

