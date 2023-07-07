import cv2    #importing the opencv module of python to access the image related functions (for the image processing part of the project)
import numpy as np      #importing the numpy module , gives access to mathematical functions and matrices etc.         #for the serial port communication  
import time             #gives access to time related functions

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)       #capturing the video from the droidcam and storing it in cap variable

def skeletonize(img):
    """ OpenCV function to return a skeletonized version of img, a Mat object"""

    #  hat tip to http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/

    img = img.copy() # don't clobber original
    skel = img.copy()

    skel[:,:] = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))

    while True:
        eroded = cv2.morphologyEx(img, cv2.MORPH_ERODE, kernel)
        temp = cv2.morphologyEx(eroded, cv2.MORPH_DILATE, kernel)
        temp  = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img[:,:] = eroded[:,:]
        if cv2.countNonZero(img) == 0:
            break

    return skel

angle = 0
while True:
    error = 0
    ret, frame = cap.read()             #this condition remains true as long as the read function is receiving frames from the source video
                                        #as soon as it stops receiving frames(at the end of the source video), this condition will turn false
                                        #and the loop exits
    if ret:
        #print(frame.shape[:2])              #used this to output my native resolution of the image received by the phone camera = (480, 640)
        blurKernel = 15

        frame = cv2.blur(frame, (blurKernel, blurKernel))
        contourFrame = np.copy(frame)

        low_b = np.uint8([84, 0, 223])
        high_b = np.uint8([136, 255, 255])

        low_y = np.uint8([0, 0, 239])
        high_y = np.uint8([43, 63, 255])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blueMask = cv2.inRange(hsv, low_b, high_b)
        yellowMask = cv2.inRange(hsv, low_y, high_y)

        # used masking as a method to preprocess the image 

        blueContours, hierarchy = cv2.findContours(blueMask, 1, cv2.CHAIN_APPROX_NONE) # then I used the contours method to introduce the contours in the masked image
        yellowContours, hierarchy = cv2.findContours(yellowMask, 1, cv2.CHAIN_APPROX_NONE) # then I used the contours method to introduce the contours in the masked image

        blueEndPoint = (frame.shape[1] / 8, frame.shape[0])
        yellowEndPoint = (frame.shape[1] * 7 / 8, frame.shape[0])

        blueAngle = None
        yellowAngle = None
        

        if len(blueContours) > 0:                                      # when there are more than two areas or two areas with black regions only consider the biggest area for the calculations
            c_b = max(blueContours, key=cv2.contourArea)
            #print(c_b.shape)
            M = cv2.moments(c_b)
            if M["m00"] !=0 :                                       #calculating the center of the countours using the moments method
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #blueEndPoint = max(np.reshape(c_b, (c_b.shape[0], c_b.shape[2])), key=lambda x: x[1])   
                blueAngle = 180 - round(np.arctan2(blueEndPoint[1] - cy, cx - blueEndPoint[0]) * 180 / np.pi)        
                print(f"Blue steering angle: {blueAngle} degrees")

                cv2.drawContours(contourFrame, [c_b], 0, (0, 0, 255), 3)
                cv2.circle(contourFrame, (cx,cy), 5, (255,255,255), -1)            #for the better view of the center of the contour we represent it by a circle to keep track of what the camera is viewing.
                cv2.line(contourFrame, (cx, cy), (round(blueEndPoint[0]), blueEndPoint[1]), (0, 255, 0), 5)

        if len(yellowContours) > 0:                                      # when there are more than two areas or two areas with black regions only consider the biggest area for the calculations
            c_y = max(yellowContours, key=cv2.contourArea)
            M = cv2.moments(c_y)
            if M["m00"] !=0 :                                       #calculating the center of the countours using the moments method
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00']) 
                #yellowEndPoint = max(c_y, key=lambda x: x[1])   
                yellowAngle = 180 - round(np.arctan2(yellowEndPoint[1] - cy, cx - yellowEndPoint[0]) * 180 / np.pi)        
                print(f"Yellow steering angle: {yellowAngle} degrees")

                cv2.drawContours(contourFrame, [c_y], 0, (0, 0, 255), 3)
                cv2.circle(contourFrame, (cx,cy), 5, (255,255,255), -1)            #for the better view of the center of the contour we represent it by a circle to keep track of what the camera is viewing.
                cv2.line(contourFrame, (cx, cy), (round(yellowEndPoint[0]), yellowEndPoint[1]), (0, 255, 0), 5)
        
        
        if blueAngle is None and yellowAngle is not None:
            angle = yellowAngle
        elif yellowAngle is None and blueAngle is not None:
            angle = blueAngle
        elif blueAngle is not None and yellowAngle is not None:
            angle = (blueAngle + yellowAngle) / 2
        else:
            print("Give up")

        cv2.imshow("Blue Mask",blueMask)                                             #display the mask for confirmation
        cv2.imshow("Yellow Mask", yellowMask)
        cv2.imshow("Frame",frame)                                           #display the original frame which is recieved
        cv2.imshow("Contours", contourFrame)
        print(f"Steering angle: {angle} degrees")
                            #The final step is to write this error on the Serial Port which
                                                                            #would be read by the Arduino and would help in implementing
                                                                            #the PID controller.
        time.sleep(0.005)

    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
                                            # it tells the loop to wait for the argument inside the waitKey function (in milliseconds)
                                        # and if it detects any keystroke inside that duration it will return true, 
                                        # also the 0xFF==ord('d') returns the ASCII value of key 'd' if it is pressed
                                        # so the break statement is only executed when the 0xFF==ord('d') returns True so the 
                                        # cv.waitKey(1) & 0xFF==ord('d') will also return true and the break statement will be executed
                                        # stopping the loop and the videocapture
        break

cap.release()
cv2.destroyAllWindows()