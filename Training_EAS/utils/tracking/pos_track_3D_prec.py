from pypylon import pylon
from MTS_3DPOS_env.utils.tracking.process_frame_prec import get_2D_pos_prec
import cv2

class pos_track_3D_prec():
    def __init__(self, maxCamerastoUse=2, normalize = False):

        self.maxCamerastoUse = maxCamerastoUse
        self.cameras = None
        self.converter = None
        self.cam1Feed = None
        self.cam2Feed = None
        self.normalize = normalize
        self.posXY = [0,0]
        self.posZ = [-1,0] # Using only 2nd Index
        self.pos_3D = [0,0,0] # x,y,z
        self.get_cameras()

    def get_cameras(self):
        try:
            tlFactory = pylon.TlFactory.GetInstance()
            # Get all attached devices and exit application if no device is found.
            devices = tlFactory.EnumerateDevices()
            if len(devices) == 0:
                raise pylon.RuntimeException("No camera present.")

            # Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
            self.cameras = pylon.InstantCameraArray(min(len(devices), self.maxCamerastoUse))

            l = self.cameras.GetSize()

            # Create and attach all Pylon Devices.
            for i, cam in enumerate(self.cameras):
                cam.Attach(tlFactory.CreateDevice(devices[i]))
                # Print the model name of the camera.
                print("Using device ", cam.GetDeviceInfo().GetModelName())

            # Grabing Continusely (video) with minimal delay
            self.cameras.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.converter = pylon.ImageFormatConverter()

            # converting to opencv bgr format
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        except:
            print("error opening cameras")

    def get_posZ(self, custom_roi = False, roi = None, visualize = False,
                  debug_ip = False, figure_name = "Z", normalize_output = False ):# roi [x, y, w, h]
        try:
            if self.cameras.IsGrabbing():
                grabResult = self.cameras[0].RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    # Access the image data
                    image = self.converter.Convert(grabResult)
                    img = image.GetArray()
                    if custom_roi == True:
                        y = roi[1] #0
                        h = roi[3] #1000
                        w = roi[2] #900
                        x = roi[0] #98
                        img = img[y:y + h, x:x + w]

                    self.cam1Feed = img
                    self.cam1Feed = cv2.rotate(self.cam1Feed, cv2.ROTATE_90_CLOCKWISE)
                    self.cam1Feed = cv2.flip(self.cam1Feed, 1)

                    self.posZ = get_2D_pos_prec(self.cam1Feed, debug=debug_ip, fig_name=figure_name+"_debug",
                                        normalize=normalize_output, w_pixels=roi[2], h_pixels=roi[3])

                    self.cam1Feed = cv2.putText(self.cam1Feed,
                                                "Pos Z: " + str(self.posZ[1]),
                                                (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

                    if visualize == True:
                        cv2.namedWindow(figure_name, cv2.WINDOW_NORMAL)
                        cv2.imshow(figure_name, self.cam1Feed)
                        k = cv2.waitKey(1)
                self.pos_3D[2] = self.posZ[1]
                grabResult.Release()
        except ValueError:
            print("Problem getting Z Position : " + ValueError)
        return self.cam1Feed

    def get_posXY(self, custom_roi = False, roi = None, visualize = False,
                  debug_ip = False, figure_name = "XY", normalize_output = False):# roi [x, y, w, h]
        try:
            if self.cameras.IsGrabbing():
                grabResult = self.cameras[1].RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    # Access the image data
                    image = self.converter.Convert(grabResult)
                    img = image.GetArray()
                    if custom_roi == True:
                        y = roi[1] #0
                        h = roi[3] #900
                        w = roi[2] #900
                        x = roi[0] #198
                        img = img[y:y + h, x:x + w]

                    self.cam2Feed = img
                    # self.cam2Feed = cv2.flip(self.cam2Feed, 1)
                    self.posXY = get_2D_pos_prec(self.cam2Feed, debug=debug_ip, fig_name=figure_name+"_debug",
                                        normalize=normalize_output, w_pixels=roi[2], h_pixels=roi[3])
                    self.cam2Feed = cv2.putText(self.cam2Feed,
                                                "Pos XY: " + str(self.posXY[0]) + " , " + str(self.posXY[1]),
                                                (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    if visualize == True:
                        cv2.namedWindow(figure_name, cv2.WINDOW_NORMAL)
                        cv2.imshow(figure_name, self.cam2Feed)
                        k = cv2.waitKey(1)

                    self.pos_3D[0] = self.posXY[0]
                    self.pos_3D[1] = self.posXY[1]
                grabResult.Release()
        except:
            print("Problem getting XY Position")
        return self.cam2Feed
