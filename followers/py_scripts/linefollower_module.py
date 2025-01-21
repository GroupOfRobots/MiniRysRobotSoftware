import cv2 as cv
import numpy as np
from pid import PID
from rclpy.impl.rcutils_logger import RcutilsLogger
from enum import Enum
from typing import Optional

class LineFollowerModuleError(RuntimeError):
    pass

class LineFollowerModule:
    class InputMode(Enum):
        NONE = 0
        RAW = 1
        PROCESSED = 2

    _image = None
    _input_mode = InputMode.NONE
    _image_binary = None
    _pid = None
    #_control_points = [(220, 270),(220, 230),(220, 200),(220, 170)]
    _control_points = [(220, 200)]
    _leftT = (435, 270)
    _rightT = (5, 270)

    def __init__(
            self,
            step_time: float,
            pid_k: float,
            pid_ti: float,
            pid_td: float,
            min_u: float,
            max_u: float,
            turn_offset: float,
            logger: Optional[RcutilsLogger]):
        self._pid = PID(step_time, pid_k, pid_ti, pid_td)
        self._min_u = min_u
        self._max_u = max_u
        self._turn_offset = turn_offset
        self._logger = logger

    def set_image(self, img: np.ndarray, mode: InputMode) -> None:
        """
        Image should be either binary or encoded in RGB
        """
        self._image = img
        self._input_mode = mode

    def compute_angular_velocity(self) -> Optional[tuple[float, np.ndarray]]:
        try:
            self.__process_image()
            contour = self.__compute_max_contour()

        except LineFollowerModuleError as e:
            self.__warning(f"Encountered error \"{e}\"."
                           + " Errors at this stage can be raised due to the input image."
                           + " Skipping this update!")
            return None

        debug_image = self._image
        cv.drawContours(image=debug_image, contours=[contour], contourIdx=-1, color=(0, 255, 0), thickness=3, lineType=cv.LINE_AA)

        turn_offset = 0
        sum_dist = 0
        for control_point in self._control_points:
            x_value = 0
            y_value = control_point[1]
            i = 0
            for contour_point in contour:
                contour_point = contour_point[0]

                if contour_point[1] == y_value:
                    x_value += contour_point[0]
                    i += 1

            if i != 0:
                sum_dist += x_value/i - control_point[0]
                cv.circle(debug_image, control_point,             radius=5, color=(0, 0, 255), thickness=-1)
                cv.circle(debug_image, (int(x_value/i), y_value), radius=5, color=(255, 0, 0), thickness=-1)

        y = sum_dist / len(self._control_points)

        # recognizing 90 degree turn
        if cv.pointPolygonTest(contour, self._leftT, False) >= 0 and cv.pointPolygonTest(contour, self._rightT, False) < 0 :
            turn_offset = -self._turn_offset#-1.0 #-1

        elif cv.pointPolygonTest(contour, self._leftT, False) < 0 and cv.pointPolygonTest(contour, self._rightT, False) >= 0 :
            turn_offset = self._turn_offset#1.0 #1

        u = self._pid.pid(y, 0) + turn_offset
        u = np.clip(u, self._min_u, self._max_u)


        return u, debug_image

    def __process_image(self) -> None:
        if self._image is None:
            raise LineFollowerModuleError('Cannot process image when the image is not set')

        if self.InputMode.PROCESSED == self._input_mode:
            self._image_binary = self._image
            return

        elif self.InputMode.RAW == self._input_mode:
            gray = cv.cvtColor(self._image, cv.COLOR_BGR2GRAY)
            kernel = np.ones((5, 5), np.uint8)
            img_erosion = cv.erode(gray, kernel, iterations=1)

            _, thresh = cv.threshold(img_erosion, 127, 255, cv.THRESH_BINARY_INV)
            self._image_binary = thresh
            return

        raise LineFollowerModuleError('Cannot process image when input mode is not set')

    def __compute_max_contour(self) -> np.ndarray:
        if self._image_binary is None:
            raise LineFollowerModuleError('Cannot compute max contour when the image is not set')

        contours, _ = cv.findContours(image=self._image_binary, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_NONE)
        areas = [cv.contourArea(c) for c in contours]

        if 0 == np.size(areas):
            raise LineFollowerModuleError('No contour areas were found. Unable to compute the max contour')

        max_idx = np.argmax(areas)
        return contours[max_idx]

    def __info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)
        else:
            print("INFO:", message)

    def __warning(self, message: str) -> None:
        if self._logger is not None:
            self._logger.warning(message)
        else:
            print("WARNING:", message)
