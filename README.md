# image_frame_process_worker

## Use case:
There are many video chat room (e.g. >100) need do image recognize with a few (e.g. 4) GPU, but we hope all chat room could take advantage of GPU-accelerate.

Use the classic producer/consumer architecture to solve this problem.
![image](https://github.com/clydewu/image_frame_process_worker/blob/master/Video%20Frame%20process.png)

## Behavior
1. An external video streaming service will send video frame into the queue of RabbitMQ.
2. Workers will get the video frame which be packaged with protobuf.
3. Fetch the video frame and use OpenCV to do image recognize with GPU-accelerate and try to find a heart shape
4. Packge the processed frame and recognization result with protobuf, and send back to the specified queue of its chating room
