#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
object_detection_node.py

This module creates the object_detection_node which is responsible for collecting
sensor data (camera images) from sensor_fusion_pkg and running object detection,
on specified object, providing normalized delta from target for
ftl_navigation_pkg.

The node defines:
    image_subscriber: A subscriber to the /sensor_fusion_pkg/sensor_msg published
                      by the sensor_fusion_pkg with sensor data.
    display_image_publisher: A publisher to publish the Image message using
                             web_video_server.
    delta_publisher: A publisher to publish the normalized error (delta) of the
                     detected object from the target (reference) position
                     with respect to x and y axes.
"""
import time
import signal
import threading
from tracemalloc import start
import cv2
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile,
                       QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

### MODIFIED ###
from deepracer_interfaces_pkg.msg import (EvoSensorMsg,
                                          DetectionDeltaMsg,
                                          ObjVelocityMsg)
### -------- ###

from openvino.inference_engine import IECore
import ngraph as ng
from object_detection_pkg import (constants_obj,
                                  utils)

### ADDED ###
from std_msgs.msg import String
### ----- ###

class ObjectDetectionNode(Node):
    """Node responsible for collecting sensor data (camera images) from sensor_fusion_pkg
       and running object detection on specified object, providing normalized delta from target for
       ftl_navigation_pkg.
    """

    def __init__(self, qos_profile):
        
        """ TRYING TO INSTANTIATE LISTS """


        """Create a ObjectDetectionNode.
        """
        super().__init__('object_detection_node')
        self.get_logger().info("object_detection_node started.")

        # Double buffer to hold the input images for inference.
        self.input_buffer = utils.DoubleBuffer(clear_data_on_get=True)
        # Get DEVICE parameter (CPU/MYRIAD) from launch file.
        self.declare_parameter("DEVICE")
        self.device = self.get_parameter("DEVICE").get_parameter_value().string_value
        if not self.device:
            self.device = constants_obj.DEVICE
        # Check if the inference output needs to be published to localhost using web_video_server
        self.declare_parameter("PUBLISH_DISPLAY_OUTPUT")
        self.publish_display_output = \
            self.get_parameter("PUBLISH_DISPLAY_OUTPUT").get_parameter_value().bool_value
        self.get_logger().info(f"Publish output set to {self.publish_display_output}")
        # Initialize Intel Inference Engine
        self.init_network()
        # Calculate target position for bounding box center.
        self.target_x, self.target_y = self.calculate_target_center(self.w, self.h)

        # Create subscription to sensor messages from camera.
        self.image_subscriber = self.create_subscription(EvoSensorMsg,
                                                         constants_obj.SENSOR_FUSION_TOPIC,
                                                         self.on_image_received_cb,
                                                         qos_profile)

        # Creating publisher for display_image.
        self.display_image_publisher = \
            self.create_publisher(Image,
                                  constants_obj.DISPLAY_IMAGE_PUBLISHER_TOPIC,
                                  10)

        # Creating publisher for error (delta) from target bb position.
        self.delta_publisher = self.create_publisher(DetectionDeltaMsg,
                                                     constants_obj.DELTA_PUBLISHER_TOPIC,
                                                     qos_profile)
        self.bridge = CvBridge()


        """ ################################ """
        ### CREATING MUTEXES TO CATCH ERRORS ###
        """ ################################ """
        self.mutex_inference = threading.Lock()
        self.mutex_velocity = threading.Lock()
        

        """ ############################### """
        ### BUFFER FOR PREVIOUS DELTA CALCS ###
        """ ############################### """
        self.delta_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Launching a separate thread to run inference.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.run_inference)
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(f"Waiting for input images on {constants_obj.SENSOR_FUSION_TOPIC}")


        """ #################################### """
        ### CREATING A PUBLISHER FOR VELOCITY!!! ###
        """ #################################### """
        # Creating publishing to calculate velocity of target
        self.velocity_publisher = \
            self.create_publisher(ObjVelocityMsg,
                                  constants_obj.INTERPOLATION_VELOCITY_PUBLISHER_TOPIC,
                                  qos_profile)

        """ #################################### """
        ### CREATING A SUBCRIBER FOR VELOCITY!!! ###
        """ #################################### """
        # Creating publishing to calculate velocity of target
        self.velocity_subscriber = \
            self.create_subscription(ObjVelocityMsg,
                                  constants_obj.INTERPOLATION_VELOCITY_PUBLISHER_TOPIC,
                                  self.velocity_listener_callback,
                                  10)
        
        """ ################################# """
        ### CREATING THREAD VELOCITY ESTIMATE ###
        """ ################################# """
        self.stop_thread_velocity = False
        self.thread_velocity_init = False
        self.thread_velocity = threading.Thread(target=self.calculate_velocity)
        self.thread_velocity.start() #self.bottom_right_x,self.bottom_right_y,self.bb_center_x,self.bb_center_y  = 
        self.thread_velocity_init = True


    def init_network(self):
        """Function which initializes Intel Inference Engine.
        """
        # Load OpenVINO Inference Engine.
        self.get_logger().info(f"Loading Inference Engine on {self.device}")
        self.ie = IECore()

        # Read and load the network.
        self.net = self.ie.read_network(model=constants_obj.MODEL_XML, weights=constants_obj.MODEL_BIN)
        self.func = ng.function_from_cnn(self.net)
        self.ops = self.func.get_ordered_ops()
        self.exec_net = self.ie.load_network(network=self.net, device_name=self.device)

        # Read expected input image info from network and prepare input blobs.
        # n: batch size, c: no. of channels, h: input height, w: input width
        for self.input_key in self.net.input_info:
            self.input_name = self.input_key
            self.n, self.c, self.h, self.w = self.net.input_info[self.input_key].input_data.shape
        # Initializing to float for optimizing in later functions
        self.h = float(self.h)
        self.w = float(self.w)

        # Prepare output blobs
        self.out_blob = next(iter(self.net.outputs))

    def wait_for_thread(self):
        """Function which joins the created background thread.
        """
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")
        
        """ #################################### """
        ### WAITING FOR VELOCITY ESTIMATE THREAD ###
        """ #################################### """
        if self.thread_velocity_init:
            self.thread_velocity.join()
        
    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread.
        """
        self.stop_thread = True
    

    """ ##################################### """
    ### SHUTTING DOWN THREAD OF VELOCITY CALC ###
    """ ##################################### """
    def thread_shutdown_velocity(self):
        """Function which sets the flag to shutdown background thread.
        """
        self.stop_thread_velocity = True
    
    def on_image_received_cb(self, sensor_data):
        """Call back for adding to the input double buffer whenever
           new sensor image is received from sensor_fusion_node.

        Args:
            sensor_data (EvoSensorMsg): Message containing sensor images and lidar data.
        """
        self.input_buffer.put(sensor_data)

    def preprocess(self, sensor_data):
        """Method that preprocesses the input data to be provided for inference to network.

        Args:
            sensor_data (EvoSensorMsg): Contains sensor images and lidar data.

        Returns:
            image: Preprosessed image expected by the network.
        """
        image = self.bridge.imgmsg_to_cv2(sensor_data.images[0])
        ih, iw = image.shape[:-1]
        # Resize to required input size
        if (ih, iw) != (int(self.h), int(self.w)):
            image = cv2.resize(image, (int(self.w), int(self.h)))
        # Change data layout from HWC to CHW.
        image = image.transpose((2, 0, 1))
        return image

    def calculate_target_center(self, image_width, image_height):
        """Method that calculates the target center's x and y co-ordinates for
           bounding box to be used as reference.

        Args:
            image_width (int): Width of the preprocessed image.
            image_height (int): Height of the preprocessed image.

        Returns:
            target_x, target_y (float)
        """
        target_x = float(image_width) / 2.0
        target_y = float(image_height) / 3.0
        self.get_logger().info(f"Target Center: x={target_x} y={target_y}")
        return target_x, target_y

    def calculate_bb_center(self, top_left_x, top_left_y, bottom_right_x, bottom_right_y):
        """Method that calculates the bounding box center's x and y co-ordinates
           representing the detected object.

        Args:
            top_left_x (int)
            top_left_y (int)
            bottom_right_x (int)
            bottom_right_y (int)

        Returns:
            bb_center_x, bb_center_y (float): Containing the x and y coordinates of
                                              detected bounding box center.
        """
        bb_center_x = top_left_x + ((bottom_right_x - top_left_x) / 2.0)
        bb_center_y = top_left_y + ((bottom_right_y - top_left_y) / 2.0)
        return bb_center_x, bb_center_y

    def calculate_delta(self, target_x, target_y, bb_center_x, bb_center_y):
        """Method that calculates the normalized error (delta) of the
           detected object from the target (reference) position
           with respect to x and y axes.

        Args:
            target_x (float): Target x co-ordinate.
            target_y (float): Target y co-ordinate.
            bb_center_x (float): x co-ordinate of center of detected bounding box.
            bb_center_y (float): y co-ordinate of center of detected bounding box.

        Returns:
            delta (DetectionDeltaMsg): Normalized Error (delta) in x and y respectively
            returned as a list of floats and converted to ObjectDetectionErrorMsg.
        """
        delta_x = (bb_center_x - target_x) / self.w
        delta_y = (bb_center_y - target_y) / self.h
        start_time = time.time()
        delta = DetectionDeltaMsg()
        delta.delta = [delta_x, delta_y,start_time]
        self.get_logger().debug(f"Delta from target position: {delta_x} {delta_y} at t={start_time}")
        return delta, delta_x, delta_y
    
    def run_inference(self):
        """Method for running inference on received input image.
        """
        ### MODIFIED bb_... INTO self.bb_... so __init__ can access them and create a thread ###
        self.mutex_inference.acquire()
        self.bottom_right_x,self.bottom_right_y,self.bb_center_x,self.bb_center_y = 0, 0, 0, 0
        try:
            while not self.stop_thread:
                # Get an input image from double buffer.
                sensor_data = self.input_buffer.get()

                # Pre-process input.
                input_data = {}
                input_data[self.input_name] = self.preprocess(sensor_data)

                # Perform Inference.
                res = self.exec_net.infer(inputs=input_data)

                # Read and postprocess output.
                res = res[self.out_blob]
                boxes, classes = {}, {}
                output_data = res[0][0]
                detected = False
                for number, proposal in enumerate(output_data):
                    # confidence for the predicted class.
                    confidence = proposal[2]
                    if (confidence > constants_obj.CONFIDENCE_THRESHOLD and
                            constants_obj.COCO_LABELS[proposal[1]] == constants_obj.DETECT_CLASS):
                        # ID of the image in the batch.
                        imid = np.int(proposal[0])
                        # predicted class ID.
                        label = np.int(proposal[1])
                        # coordinates of the top left bounding box corner.
                        # (coordinates are in normalized format, in range [0, 1])
                        """ MODIFIED bb_... INTO self.bb_... so __init__ can access them and create a thread """
                        self.top_left_x = np.int(self.w * proposal[3])
                        self.top_left_y = np.int(self.h * proposal[4])
                        # coordinates of the bottom right bounding box corner.
                        # (coordinates are in normalized format, in range [0, 1])
                        """ MODIFIED bb_... INTO self.bb_... so __init__ can access them and create a thread """
                        self.bottom_right_x = np.int(self.w * proposal[5])
                        self.bottom_right_y = np.int(self.h * proposal[6])
                        # Calculate bounding box center
                        """ MODIFIED bb_... INTO self.bb_... so __init__ can access them and create a thread """
                        self.bb_center_x, self.bb_center_y = self.calculate_bb_center(self.top_left_x,
                                                                            self.top_left_y,
                                                                            self.bottom_right_x,
                                                                            self.bottom_right_y)
                        # Calculate detection delta.
                        """ MODIFIED bb_... INTO self.bb_... so __init__ can access them and create a thread """
                        detection_delta, _, _ = self.calculate_delta(self.target_x,
                                                                self.target_y,
                                                                self.bb_center_x,
                                                                self.bb_center_y)
                        # Publish to object_detection_delta topic.
                        self.delta_publisher.publish(detection_delta)
                        # Set the flag that there is a detected object.
                        detected = True

                        if imid not in boxes.keys():
                            boxes[imid] = []
                        """ MODIFIED bb_... INTO self.bb_... so __init__ can access them and create a thread """
                        boxes[imid].append([self.top_left_x, self.top_left_y, self.bottom_right_x, self.bottom_right_y])
                        if imid not in classes.keys():
                            classes[imid] = []
                        classes[imid].append(label)
                        # Break as soon as specified class is detected.
                        break

                if not detected:
                    # Assume being at target position.
                    detection_delta = self.calculate_delta(self.target_x,
                                                            self.target_y,
                                                            self.target_x,
                                                            self.target_y)
                    self.delta_publisher.publish(detection_delta)

                if self.publish_display_output:
                    # Change data layout from CHW to HWC.
                    display_image = input_data[self.input_name].transpose((1, 2, 0))
                    for imid in classes:
                        for box in boxes[imid]:
                            # Drawing bounding boxes on the image.
                            cv2.rectangle(display_image,
                                            (box[0], box[1]),
                                            (box[2], box[3]),
                                            (232, 35, 244),
                                            2)
                    # Printing target center on the image.
                    cv2.circle(display_image,
                                (int(self.target_x),
                                int(self.target_y)),
                                5,
                                (0, 255, 0),
                                -1)
                    # Publish to display topic (Can be viewed on localhost:8080).
                    display_image = self.bridge.cv2_to_imgmsg(np.array(display_image), "bgr8")
                    self.display_image_publisher.publish(display_image)
                """ MODIFIED commented to check if other thread effectively runs"""
                #self.get_logger().info(f"Total execution time = {time.time() - start_time}")
        except BaseException as ex:
            self.get_logger().error(f"Failed inference step: {ex}")
            # Destroy the ROS Node running in another thread as well.
            """
            self.destroy_node()
            rclpy.shutdown()
            """
        finally:
            self.mutex_inference.release()
            self.get_logger().info(f"Inference mutex released by inference process")

    """ ##################################################### """
    ### !!! IMPLEMENTING VELOCITY ESTIMATE WITH INTERPOLATION !!!
    """ ##################################################### """
    def calculate_velocity(self):
        """Method that calculates a velocity estimate of the object we are tracking

        Args:
            TO DO

        Returns:
            TO DO
        """ 
        self.mutex_inference.acquire()
        self.mutex_velocity.acquire()
        try:
            while not self.stop_thread_velocity:
                delta_t, delta_x_t, delta_y_t = self.calculate_delta(self.target_x, self.target_y, self.bb_center_x, self.bb_center_y)
                delta_t_1 = self.delta_buffer.get()
                vx = (delta_x_t-delta_t_1[0])/(delta_t[2]-delta_t_1[2])
                vy = (delta_y_t-delta_t_1[1])/(delta_t[2]-delta_t_1[2])
                reference_time = time.perf_counter()
                
                Velocity = ObjVelocityMsg()
                Velocity.velocity = [vx,vy,reference_time]
                self.get_logger().info(f"Relative positions: {delta_x_t},{delta_y_t}")
                self.get_logger().info(f"Vel from target position: {vx},{vy}")
                # self.get_logger().debug(f"Vel from target type: {type(vx)}")
                self.velocity_publisher.publish(Velocity)
                # return (vx**2+vy**2)**0.5/(delta_t)
        except Exception as ex:
            self.get_logger().error(f"Failed velocity calculation step: {ex}")
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()
        finally:
            self.mutex_velocity.release()
            self.get_logger().info(f"Velocity mutex released by velocity process")
            self.mutex_inference.release()
            self.get_logger().info(f"Inference mutex released by velocity process")

    """ #################################### """
    ### CREATING A CALLBACK FUNC OF VELOCITY ###
    """ #################################### """
    def velocity_listener_callback(self, msg):
        msg = ObjVelocityMsg()
        self.get_logger().info(f'I heard: "{msg.velocity}')

def main(args=None):
    rclpy.init(args=args)
    """ #################################################### """
    ### MODIFIED RELIABILITY AND DEPTH TO PRODUCE TO OUTPUTS ###
    """ #################################################### """
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

    try:
        object_detection_node = ObjectDetectionNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number
                frame: the current stack frame (None or a frame object)
            """
            object_detection_node.get_logger().info("Signal Handler initiated")
            object_detection_node.thread_shutdown()
            object_detection_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)

        rclpy.spin(object_detection_node, executor)

    except Exception as ex:
        object_detection_node.get_logger().error(f"Exception in Object Detection Node: {ex}")
        object_detection_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    ### MODIFIED: COMMENTED TO ERASE HANDLE ERROR ###
    object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
