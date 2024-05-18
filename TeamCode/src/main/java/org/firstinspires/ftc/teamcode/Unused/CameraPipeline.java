package org.firstinspires.ftc.teamcode.Unused;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

class CameraPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
        return input;
    }
}
