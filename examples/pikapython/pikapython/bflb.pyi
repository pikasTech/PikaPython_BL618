class Camera:
    """Camera class for controlling and accessing the BLMCU camera module."""

    def __init__(self):
        """
        Initialize the Camera object, set up the necessary configurations,
        and prepare the camera module for operation.
        """

    def start(self):
        """
        Start the camera module for capturing frames.
        """

    def stop(self):
        """
        Stop the camera module from capturing frames.
        """

    def get_frame_count(self) -> int:
        """
        Get the current frame count of the camera module.

        Returns:
            int: The number of frames currently available.
        """

    def get_frame_info(self) -> tuple:
        """
        Get information about the next available frame.

        Returns:
            tuple: A tuple containing the frame's address (int) and size (int) in bytes.
        """

    def pop_one_frame(self):
        """
        Pop the next frame from the camera module's frame buffer.
        This should be called after retrieving frame information using get_frame_info().
        """

    def demo(self):
        """
        Run a demo of the camera module.
        """

    def set_callback(self, callback: any):
        """
        Set a callback function to be called when a frame is available.

        Args:
            callback (function): The callback function to be called.
        """

class Microphone:
    def __init__(self):...

    def start(self):...

    def stop(self):...

    def set_callback(self, callback: any):...

    def demo(self):...
