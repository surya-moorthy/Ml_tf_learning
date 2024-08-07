import math
import random
import cvzone
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector


cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)

detector = HandDetector(detectionCon = 0.8,maxHands=1)

class SnakeGame:
    def __init__(self , pathFood):
        self.points = [] #points of the snake
        self.lengths = [] #distence between each point
        self.currentlength = 0 #total length of the snake
        self.allowedLength = 500 #total allowed length 
        self.previoushead = 0,0 #previous head length
        self.score = 0
        self.imgFood = cv2.imread(pathFood , cv2.IMREAD_UNCHANGED)
        self.hFood , self.WFood , _ = self.imgFood.shape
        self.randomFoodLocation()
        self.gameover = False


    def randomFoodLocation(self):
        self.foodpoint = random.randint(100,1000),random.randint(100,600)
    def update(self , imgMain , headCurrent):

        if self.gameover:
             cvzone.putTextRect(imgMain,"Game Over",[300,400],scale=7,thickness=5,offset=50)
             cvzone.putTextRect(imgMain,f'Your score:{self.score}',[300,400],scale=7,thickness=5,offset=50)
        else:
            px ,py = self.previoushead
            cx , cy = headCurrent

            self.points.append([cx,cy])
            distance = math.hypot(cx-px,cy-py)
            self.lengths.append(distance)
            self.currentlength += distance
            self.previoushead = cx , cy
            
            #length reduction
            if self.currentlength > self.allowedLength:
                for i,length in enumerate(self.lengths):
                    self.currentlength -= length
                    self.lengths.pop(i)
                    self.points.pop(i)
                    if self.currentlength < self.allowedLength:
                        break
            #chaeck if snake ate food

            rx , ry = self.foodpoint
            if rx -self.WFood//2< cx< rx+self.WFood//2 and ry -self.hFood//2< cy < ry+self.hFood//2:
                self.randomFoodLocation()
                self.allowedLength += 50
                self.score += 1
                print(self.score)
        
        
            #Draw snake
            if self.points:
                for i,point in enumerate(self.points):
                    if i!=0 :
                        cv2.line(imgMain,self.points[i-1],self.points[i],(0,0,225),20)
                cv2.circle(imgMain,pointIndex,20,(200,0,200),cv2.FILLED)

            #draw food
            rx , ry = self.foodpoint

            imgMain = cvzone.overlayPNG(imgMain,self.imgFood,(rx-self.WFood//2,ry-self.hFood//2))

            cvzone.putTextRect(imgMain,f'Your score:{self.score}',[30,80],scale=3,thickness=3,offset=10)

            #check for collision
            pts = np.array(self.points[:-2],np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(imgMain,[pts],False,(0,200,0),3)
            minDist = cv2.pointPolygonTest(pts,(cx,cy),True)
            print(minDist)

            if -1<= minDist <=1:
                print("hit")
                self.gameover = True
                self.points = [] #points of the snake
                self.lengths = [] #distence between each point
                self.currentlength = 0 #total length of the snake
                self.allowedLength = 500 #total allowed length 
                self.previoushead = 0,0 #previous head length
                self.score = 0
                self.randomFoodLocation()



        return imgMain
        
game = SnakeGame("donut.png")

while True:
    success, img = cap.read()
    image = cv2.flip(img,1)
    hands , img = detector.findHands(img,flipType=False)
    
    if hands:
        lmlist = hands[0]['lmlist']
        pointIndex = lmlist[8][0:2]
        img = game.update(img , pointIndex)

    cv2.imshow("Image",img)
    key = cv2.waitKey(1)
    if key == ord('r'):
        game.gameover = False