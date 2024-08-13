import math
import random
import time
import cvzone
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

detector = HandDetector(detectionCon=0.8, maxHands=1)

# Snake speed control
snake_speed = 0.1  # Adjust this value for desired speed (lower = slower)

class SnakeGameClass:
    # ... (keep the __init__ and other methods as they are)

    def update(self, currentHead):
        currentTime = time.time()

        # Speed control
        if currentTime - self.last_update_time > snake_speed:
            self.last_update_time = currentTime

            if self.gameOver:
                return

            px, py = self.previousHead
            cx, cy = currentHead

            self.points.append([cx, cy])
            distance = math.hypot(cx - px, cy - py)
            self.lengths.append(distance)
            self.currentLength += distance
            self.previousHead = cx, cy

            # Length Reduction
            while self.currentLength > self.allowedLength and self.lengths:
                self.currentLength -= self.lengths.pop(0)
                self.points.pop(0)

            # Check if snake ate the Food
            rx, ry = self.foodPoint
            if rx - self.wFood // 2 < cx < rx + self.wFood // 2 and \
                    ry - self.hFood // 2 < cy < ry + self.hFood // 2:
                self.randomFoodLocation()
                self.allowedLength += 50
                self.score += 1
                print(self.score)

            # Check for Collision
            if len(self.points) > 2:
                pts = np.array(self.points[:-2], np.int32)
                pts = pts.reshape((-1, 1, 2))
                minDist = cv2.pointPolygonTest(pts, (cx, cy), True)

                if -1 <= minDist <= 1:
                    print("Hit")
                    self.gameOver = True

    def draw(self, imgMain):
        # Draw Snake
        if self.points:
            for i in range(1, len(self.points)):
                cv2.line(imgMain, self.points[i - 1], self.points[i], (0, 0, 255), 20)
            cv2.circle(imgMain, self.points[-1], 20, (0, 255, 0), cv2.FILLED)

        # Draw Food
        rx, ry = self.foodPoint
        imgMain = cvzone.overlayPNG(imgMain, self.imgFood,
                                    (rx - self.wFood // 2, ry - self.hFood // 2))

        cvzone.putTextRect(imgMain, f'Score: {self.score}', [50, 80],
                        scale=3, thickness=3, offset=10)

        if self.gameOver:
            cvzone.putTextRect(imgMain, "Game Over", [300, 400],
                               scale=7, thickness=5, offset=20)
            cvzone.putTextRect(imgMain, f'Your Score: {self.score}', [300, 550],
                               scale=7, thickness=5, offset=20)

        return imgMain

game = SnakeGameClass("donut.png")

FPS = 30
frame_time = 1.0 / FPS

while True:
    start_time = time.time()

    success, img = cap.read()
    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)

    if hands:
        lmList = hands[0]['lmList']
        pointIndex = lmList[8][0:2]
        game.update(pointIndex)

    # Create a copy of the image for drawing
    display_img = img.copy()
    display_img = game.draw(display_img)

    cv2.imshow("Image", display_img)
    key = cv2.waitKey(1)

    if key == ord('r'):
        game.reset_game()
    elif key == 13:  # Enter key to exit
        break

    # Calculate the time taken for processing this frame
    process_time = time.time() - start_time
    
    # If processing time was less than the desired frame time, wait for the remaining time
    if process_time < frame_time:
        time.sleep(frame_time - process_time)

cap.release()
cv2.destroyAllWindows()