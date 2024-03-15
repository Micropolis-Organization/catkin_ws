#!/usr/bin/python3
# F74rr*CI}Z
import rospy, rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import copy
import time
import cv2 as cv
import numpy as np
import onnxruntime
from cv_bridge import CvBridge, CvBridgeError

# RosPack instance to get oackage info
rospack = rospkg.RosPack()


class LaneDetectionNode:
    def __init__(self, _verbose=False):
        # Initialize the ROS node
        rospy.init_node('lane_kepping', anonymous=True)

        # Set the input size and number of classes for the model
        self.input_size = (288, 480)
        self.image_size = (480, 640)

        # Set the path to the weights file
        self.weights_path = rospack.get_path('lane_kepping') + '/models/' +'yolopv2_post_288x480.onnx'

        # Set the threshold for postprocessing the predictions
        self.threshold = 0.5

        # Create a CvBridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_subscriber = rospy.Subscriber('/m2d/camera/image', Image, self.image_callback)

        # Create a publisher for the velocity commands
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Load Model
        self.onnx_session = onnxruntime.InferenceSession(
            self.weights_path,
            providers=[
            #  'TensorrtExecutionProvider', 
             'CUDAExecutionProvider', 
             'CPUExecutionProvider']
        )

        self.verbose = _verbose


    def image_callback(self, image_message):
        try:
            # Convert the ROS image to a OpenCV image
            image = self.bridge.imgmsg_to_cv2(image_message, 'bgr8')
        except CvBridgeError as e:
            print(e)
        self.image_size = image.shape[:2]
        # H, W = image.shape[:2]
        # image = image[int(H / 2) :, :,:]

        # Detect the lanes in the image
        lane_image = self.detect_lanes(image)

        # Estimate the position of the robot in the lane
        position = self.estimate_position(lane_image)

        # Plan the trajectory for the robot
        self.plan_trajectory(position)


    def detect_lanes(self, image):
        start_time = time.time()
        debug_image = copy.deepcopy(image)

        # Run the image through the model to get the lane predictions
        drivable_area, lane_line, bboxes, scores, class_ids = self.run_inference(
            self.onnx_session,
            self.input_size,
            image,
        )

        # lane_line = self.roi(lane_line)

        elapsed_time = time.time() - start_time

        if self.verbose:
            # Draw
            debug_image = self.draw_debug(
                debug_image,
                elapsed_time,
                drivable_area,
                lane_line,
                bboxes,
                scores,
                class_ids,
            )
            cv.imshow('Drivable Area', debug_image)
            cv.imshow('Lanes', lane_line)
            cv.waitKey(30)

        return lane_line

    def preprocess_input(self, image, input_size):
        # Pre process:Resize, BGR->RGB, float32 cast
        input_image = cv.resize(image, dsize=(input_size[1], input_size[0]))
        input_image = cv.cvtColor(input_image, cv.COLOR_BGR2RGB)
        input_image = input_image.transpose(2, 0, 1)
        input_image = np.expand_dims(input_image, axis=0)
        input_image = input_image.astype('float32')
        input_image = input_image / 255.0

        return input_image

    def postprocess_output(self, prediction, threshold):
        # Get the binary segmentation mask for the lanes
        binary_mask = prediction
        # Convert the binary mask to a grayscale image
        lane_image = np.array(binary_mask[0, :, :, 0] * 255, dtype=np.uint8)

        return lane_image

    def roi(self, image):
        h, w = image.shape
        cut = int(h / 3)
        ret = image.copy()
        # ret[-cut:,:] = 0
        ret[:cut,:] = 0
        return ret

    def calculate_position(self, lane_mask):
        # Calculate the histogram of the lane mask
        histogram = np.sum(lane_mask[lane_mask.shape[0]//2:,:], axis=0)

        # Calculate the midpoint of the histogram
        midpoint = int(histogram.shape[0]//2)

        # Calculate the position of the left and right lane lines
        left_lane = np.argmax(histogram[:midpoint])
        right_lane = np.argmax(histogram[midpoint:]) + midpoint

        # Calculate the position of the vehicle within the lane
        position = (left_lane + right_lane) / (2.0 * histogram.shape[0])

        return position


    def estimate_position(self, lane_image):
        # Get the moments of the binary mask
        moments = cv.moments(lane_image)

        # Get the centroid of the moments
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
        else:
            cx, cy = 0, 0

        return cx, cy

    def plan_trajectory(self, position):
        # Get the position of the robot in the lane
        cx, cy = position

        # Get the center of the image
        center = self.image_size[1] / 2

        # Calculate the error between the center of the lane and the center of the image
        error = center - cx
        
        # Create a Twist message to control the velocity of the robot
        velocity_command = Twist()
        
        # Set the linear velocity based on the error
        velocity_command.linear.x = 2.5
        velocity_command.angular.z = error * 0.01
        
        # Publish the velocity command
        self.velocity_publisher.publish(velocity_command)


    def run_inference(self, onnx_session, input_size, image):
        image_width, image_height = image.shape[1], image.shape[0]

        # Preprocess image
        input_image = self.preprocess_input(image, input_size)

        # Inference
        input_name = onnx_session.get_inputs()[0].name
        results = onnx_session.run(None, {input_name: input_image})

        # Post process
        drivable_area = np.squeeze(results[0])
        lane_line = np.squeeze(results[1])
        scores = results[2]
        batchno_classid_y1x1y2x2 = results[3]

        # Drivable Area Segmentation
        drivable_area = drivable_area.transpose(1, 2, 0)
        drivable_area = cv.resize(
            drivable_area,
            dsize=(image_width, image_height),
            interpolation=cv.INTER_LINEAR,
        )
        drivable_area = drivable_area.transpose(2, 0, 1)

        # Lane Line
        lane_line = cv.resize(
            lane_line,
            dsize=(image_width, image_height),
            interpolation=cv.INTER_LINEAR,
        )

        # Traffic Object Detection
        od_bboxes, od_scores, od_class_ids = [], [], []
        for score, batchno_classid_y1x1y2x2_ in zip(
                scores,
                batchno_classid_y1x1y2x2,
        ):
            class_id = int(batchno_classid_y1x1y2x2_[1])

            if self.threshold > score:
                continue

            y1 = batchno_classid_y1x1y2x2_[-4]
            x1 = batchno_classid_y1x1y2x2_[-3]
            y2 = batchno_classid_y1x1y2x2_[-2]
            x2 = batchno_classid_y1x1y2x2_[-1]
            y1 = int(y1 * (image_height / input_size[0]))
            x1 = int(x1 * (image_width / input_size[1]))
            y2 = int(y2 * (image_height / input_size[0]))
            x2 = int(x2 * (image_width / input_size[1]))

            od_bboxes.append([x1, y1, x2, y2])
            od_class_ids.append(class_id)
            od_scores.append(score)

        return drivable_area, lane_line, od_bboxes, od_scores, od_class_ids

    def draw_debug(self,
        debug_image,
        elapsed_time,
        drivable_area,
        lane_line,
        bboxes,
        scores,
        class_ids,
    ):
        # Draw:Drivable Area Segmentation
        # Not in Drivable Area
        bg_image = np.zeros(debug_image.shape, dtype=np.uint8)
        bg_image[:] = (255, 0, 0)

        mask = np.where(drivable_area[0] > 0.5, 0, 1)
        mask = np.stack((mask, ) * 3, axis=-1).astype('uint8')
        mask_image = np.where(mask, debug_image, bg_image)
        debug_image = cv.addWeighted(debug_image, 0.75, mask_image, 0.25, 1.0)

        # Drivable Area
        bg_image = np.zeros(debug_image.shape, dtype=np.uint8)
        bg_image[:] = (0, 255, 0)

        mask = np.where(drivable_area[1] > 0.5, 0, 1)
        mask = np.stack((mask, ) * 3, axis=-1).astype('uint8')
        mask_image = np.where(mask, debug_image, bg_image)
        debug_image = cv.addWeighted(debug_image, 0.5, mask_image, 0.5, 1.0)

        # Draw:Lane Line
        bg_image = np.zeros(debug_image.shape, dtype=np.uint8)
        bg_image[:] = (0, 0, 255)

        mask = np.where(lane_line > 0.5, 0, 1)
        mask = np.stack((mask, ) * 3, axis=-1).astype('uint8')
        mask_image = np.where(mask, debug_image, bg_image)
        debug_image = cv.addWeighted(debug_image, 0.5, mask_image, 0.5, 1.0)

        # Draw:Traffic Object Detection
        for bbox, score, class_id in zip(bboxes, scores, class_ids):
            x1, y1 = int(bbox[0]), int(bbox[1])
            x2, y2 = int(bbox[2]), int(bbox[3])

            # cv.rectangle(debug_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
            # cv.putText(debug_image, '%d:%.2f' % (class_id, score), (x1, y1 - 5), 0,
            #         0.7, (255, 255, 0), 2)

        # Inference elapsed time
        cv.putText(debug_image,
                "Elapsed Time : " + '{:.1f}'.format(elapsed_time * 1000) + "ms",
                (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1,
                cv.LINE_AA)

        return debug_image


if __name__ == '__main__':
    try:
        node = LaneDetectionNode(True)
        rospy.spin()
        cv.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
