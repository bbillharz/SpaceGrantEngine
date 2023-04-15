from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge

def detectHeading(depthMap):

    # fill out the edges so we don't use that data
    borderSize = 50
    depthMap[:, 0 : borderSize] = 0
    depthMap[:, -borderSize : -1] = 0
    depthMap[0 : borderSize, :] = 0
    depthMap[-borderSize : -1, :] = 0

    minDepth = np.min(depthMap)
    maxDepth = np.max(depthMap)
    
    # Blur source
    radius = 7
    depthMap = cv2.filter2D(depthMap, -1, np.ones((radius,radius),np.float32))

    laplacian = cv2.Laplacian(depthMap, cv2.CV_32F)
    n = np.linalg.norm(laplacian)
    laplacian /= n
    laplacian *= 255.0
    radius = 7
    #laplacian = cv2.filter2D(laplacian, -1, np.ones((radius,radius),np.float32))
    ma = np.max(laplacian)
    mi = np.min(laplacian)
    #print(f"max={ma}, min={mi}")

    # Bias the laplacian
    depthScale = ((depthMap - minDepth) / (maxDepth - minDepth)).astype(np.float32)
    #print(f"depthType={depthScale.dtype}, laplacianType={laplacian.dtype}")
    laplacian = laplacian + (1 - depthScale)
    
    laplacian = np.where(laplacian < (mi + 0.3*(ma - mi)), 0, laplacian)

    # Find best region
    binary = np.where(laplacian < 0.8, 0, 1)
    nonZeros = np.argwhere(binary > 0.5)
    distances = np.zeros((binary.shape[1]))
    c = int(binary.shape[0] / 2)

    if len(nonZeros) == 0:
        # just make points on left and right side to give us default heading
        nonZeros = np.zeros((2,2), dtype=int)
        nonZeros[0] = (int(binary.shape[1]), c)
        nonZeros[1] = (0, c)

    for r in range(binary.shape[1]):
        parts = np.subtract(np.flip(nonZeros, axis=1), (r,c))
        parts = parts * parts
        euclidDist = np.sqrt(np.sum(parts, axis=1))
        minDist = np.min(euclidDist)
        distances[r] = minDist

    # Grayscale to bgr for debugging
    #laplacian = cv2.cvtColor(laplacian, cv2.COLOR_GRAY2BGR)

    # Debug non zero locations
    #for i,(x,y) in enumerate(nonZeros):
    #    laplacian = cv2.circle(laplacian, center=(y,x), radius=1, color=(0,1,0), thickness=1)

    # Debug distances vector
    #maxDist = np.max(distances)
    #minDist = np.min(distances)
    #print(f"MAXDIST={maxDist}, MINDIST={minDist}")
    #for i,d in enumerate(distances):
    #    scale = (d - minDist) / (maxDist - minDist)
    #    #print(f"i={i}, scale={scale}")
    #    laplacian = cv2.circle(laplacian, center=(int(i), int(c)), radius=1, color=(1.0-scale,scale,0), thickness=1)
    #    #laplacian = cv2.circle(laplacian, center=(int(i), int(c)), radius=1, color=(i / binary.shape[1],0,0), thickness=1)

    minIdx = np.argmax(distances)
    #print(f"minIdx={minIdx}")
    #laplacian = cv2.circle(laplacian, center=(int(minIdx), int(c)), radius=8, color=(0,0,0.5), thickness=3)

    # Rolling average of headings
    #headings = np.append(headings, minIdx)
    #historyLen = 8
    #print(f"h={headings}")
    #if len(headings) > historyLen:
    #    headings = headings[1 :]

    #avgHeading = np.average(headings)
    #laplacian = cv2.circle(laplacian, center=(int(avgHeading), int(c)), radius=14, color=(0,0,1), thickness=4)

    return minIdx

class PathfindingNode(Node):
    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

        self._historyLen = 8
        self._headings = np.zeros(shape=(0,1))
        self._bridge = CvBridge()
        Node.__init__(self, "pathfinding")
        
        def detect_callback(depthMap: Image) -> None:
            depthMapCv = self._bridge.imgmsg_to_cv2(depthMap, desired_encoding="passthrough")
            newHeading = detectHeading(depthMapCv)
            self._headings = np.append(self._headings, newHeading)
            if len(self._headings) > self._historyLen:
                self._headings = self._headings[1 :]

            avgHeading = np.average(self._headings)
            normHeading = float(avgHeading) / float(depthMapCv.shape[1])
            normHeading = normHeading * 2.0 - 1.0 # [-1, 1]

            # Angular quantization
            thresh = 0.5
            if normHeading < -thresh:
                normHeading = -1
            elif normHeading > thresh:
                normHeading = 1
            else:
                normHeading = 0

            moveCmd = TwoFloat()
            moveCmd.first = normHeading # angular
            moveCmd.second = 1 - abs(normHeading) # linear
            self._publisher.publish(moveCmd)

        self._subscription = self.create_subscription(
            Image, "/odom/depth", detect_callback, 10
        )
        self._publisher = self.create_publisher(TwoFloat, "pico/move_command", 10)

        print("Running Pathfinding")

def main(args=None):
    rclpy.init(args=args)
    pathfinding = PathfindingNode()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()