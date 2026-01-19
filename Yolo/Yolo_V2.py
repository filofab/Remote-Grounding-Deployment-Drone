# 1. Import the InferencePipeline library
from inference import InferencePipeline
import cv2

def my_sink(result, video_frame):
    if result.get("output_image"): # Display an image from the workflow response
        cv2.imshow("Workflow Image", result["output_image"].numpy_image)
        cv2.waitKey(1)
    # Do something with the predictions of each frame
    print(result)


# 2. Initialize a pipeline object
pipeline = InferencePipeline.init_with_workflow(
    api_key="46yfJK5ofLbVsRy1aUMk",
    workspace_name="yolo-ytzsf",
    workflow_id="find-people",
    video_reference=0, # Path to video, device id (int, usually 0 for built in webcams), or RTSP stream url
    max_fps=30,
    on_prediction=my_sink
)

# 3. Start the pipeline and wait for it to finish
pipeline.start()
pipeline.join()
