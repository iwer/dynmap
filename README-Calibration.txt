To get an optimal calibration, grabbed images should ensure the following:

    Cover as most image area as possible. Especially check for coverage of the image corners.
    Try to get the chessboard as close as possible to the camera to get better precision.
    For depth calibration, you will need some images with IR and depth. But for stereo calibration, the depth information is not required, so feel free to cover the IR projector and get very close to the camera to better estimate IR intrinsics and stereo parameters. The calibration algorithm will automatically determine which grabbed images can be used for depth calibration.
    Move the chessboard with various angles.
    I usually grab a set of 30 images to average the errors.
    Typical reprojection error is < 1 pixel. If you get significantly higher values, it means the calibration probably failed. 

from: http://labs.manctl.com/rgbdemo/index.php/Documentation/Calibration
