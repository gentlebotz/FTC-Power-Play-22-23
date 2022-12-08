package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
//import org.firstinspires.ftc.robotcore.external.Telemetry;

public class powerplayPipeline extends OpenCvPipeline {
    // Only needeed for EOCV Sim telemetry
    // Telemetry telemetry;
    //
    //  public powerplayPipeline(Telemetry telemetry) {
    //      this.telemetry = telemetry;
    //  }

    //Color definitions
    private final Scalar WHITE = new Scalar(255, 255, 255);
    private final Scalar BLACK = new Scalar(0,0,0);
    private final Scalar YELLOW = new Scalar(214, 157, 8);
    private Scalar sumColors;


    private Mat ycrcbMat        = new Mat();
    private Mat areaMat         = new Mat();
    private Mat maskedInputMat  = new Mat();

    private double minColor;

    // Setup detection area bounds
    static Point borderLeft = new Point(420,130);
    static Point borderRight = new Point(680,470);

    /*
    WHITE   = LEFT  (1)
    YELLOW  = MID   (2)
    BLACK   = RIGHT (3)
    */
    public enum ParkLocation{
        LEFT,
        MID,
        RIGHT
    }

    private volatile ParkLocation location = ParkLocation.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched

        // Convert image to brightness independant YCrCb colorspace
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Get the material
        areaMat = ycrcbMat.submat(new Rect(borderLeft, borderRight));
        ycrcbMat.release();

        // Get the sum of all color values
        sumColors = Core.sumElems(areaMat);

        // Get the minimum value of each RGB channel
        minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        // Decide the parking location and change border color accordingly
        if(sumColors.val[0] == minColor) {
            location = ParkLocation.RIGHT;
            Imgproc.rectangle(
                    input,
                    borderLeft,
                    borderRight,
                    BLACK,
                    2
            );
        } else if(sumColors.val[2] == minColor){
            location = ParkLocation.MID;
            Imgproc.rectangle(
                    input,
                    borderLeft,
                    borderRight,
                    YELLOW,
                    2
            );
        } else{
            location = ParkLocation.LEFT;
            Imgproc.rectangle(
                    input,
                    borderLeft,
                    borderRight,
                    WHITE,
                    4
            );
        }

        // Only needed for EOCV Sim telemetry
        // telemetry.addData("sumColors[0]: ", sumColors.val[0]);
        // telemetry.addData("sumColors[1]: ", sumColors.val[1]);
        // telemetry.addData("sumColors[2]: ", sumColors.val[2]);
        // telemetry.addData("minColor: ", minColor);
        // telemetry.addData("Location: ", location);
        // telemetry.update();

        areaMat.release();
        return input; // Return the image that will be displayed in the viewport
    }

    // Return enum for parking location
    public ParkLocation getLocation(){
        return location;
    }

}