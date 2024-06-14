#!/usr/bin/python3
import cv2 
from threading import Thread
#from djitellopy import Tello
import os
from time import sleep


def videoRec():
    
    # Set text overaly parameters:
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (255,255,255)
    lineType               = 2
        
    FPS = 30
    codec = 'h264'
    fourcc = cv2.VideoWriter_fourcc(*codec)
    
    # Start the drone stream:
    tello.streamon()
    sleep(0.1)
    
    # Grab a frame to check size:
    img = tello.get_frame_read().frame
    shape = (img.shape[1], img.shape[0])
    

    while True:
        frame = tello.get_frame_read().frame
        low_hue = cv2.getTrackbarPos('low_hue','Trackbar')
        up_hue = cv2.getTrackbarPos('up_hue','Trackbar')
        low_sat= cv2.getTrackbarPos('low_sat','Trackbar')
        up_sat = cv2.getTrackbarPos('up_sat','Trackbar')
        low_val = cv2.getTrackbarPos('low_val','Trackbar')
        up_val = cv2.getTrackbarPos('up_val','Trackbar')

        lowe = np.array([low_hue,low_sat,low_val])
        upe = np.array([up_hue,up_sat,up_val])

        ## Convert to HSL (Cube)
        converto2HLS = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        convert2HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ## Detect color
        color_cube = cv2.inRange(converto2HLS,lowe, upe)
        parrot = cv2.inRange(convert2HSV,lowp,upp)
        exitRoute = cv2.inRange(convert2HSV,lowe,upe)
        ## Clean Image
        kernel = np.ones((5, 5), np.uint8) 
        color_cube = cv2.erode(color_cube, kernel, iterations=1) 
        color_cube = cv2.dilate(color_cube, kernel, iterations=1) 
        parrot = cv2.erode(parrot, kernel, iterations=1) 
        parrot = cv2.dilate(parrot, kernel, iterations=1) 
        exitRoute = cv2.erode(exitRoute, kernel, iterations=1) 
        exitRoute = cv2.dilate(exitRoute, kernel, iterations=1) 

        ## Cube
        # 0 - height, 1 width
        hRectangle = int(color_cube.shape[0]*.4)
        wRectangle = int(color_cube.shape[1]*.4)
        x = int((color_cube.shape[1] - wRectangle)/2)
        y = int((color_cube.shape[0] - hRectangle)/2)
        cubeROI = color_cube[y:y+hRectangle,x:x+wRectangle]
        contours,hierarchy = cv2.findContours(cubeROI, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if (area > 10000):
                forward = False
                print("White wall")
        ## Parrot
        # 0 - height, 1 width
        hRectangleP = int(parrot.shape[0]*.4)
        wRectangleP = int(parrot.shape[1]*.6)
        xP = int((parrot.shape[1] - wRectangleP)/2)
        yP = int((parrot.shape[0] - hRectangleP)/2)
        parrotROI = parrot[yP:yP+hRectangleP,xP:xP+wRectangleP]
        contours,hierarchy = cv2.findContours(parrotROI, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if (area > 4000):
                turn = True
                print("Parrot")
        ## Exit
        # 0 - height, 1 width
        hRectangleE = int(exitRoute.shape[0]*.4)
        wRectangleE = int(exitRoute.shape[1]*.6)
        xE = int((exitRoute.shape[1] - wRectangleE)/2)
        yE = int((exitRoute.shape[0] - hRectangleE)/2)
        exitRouteROI = exitRoute[yE:yE+hRectangleE,xE:xE+wRectangleE]
        contours,hierarchy = cv2.findContours(exitRouteROI, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if (area > 4000):
                exitState = True
                print("Exit")

        

        ## Display Frames
        # Cube ROI
        #color_cube_show = cv2.cvtColor(color_cube, cv2.COLOR_GRAY2RGB)
        #twoFrames = np.concatenate((frame,color_cube_show),axis=1)
        #cv2.rectangle(twoFrames,(x,y),(x+wRectangle,y+hRectangle),(255,0,0),2)
        # Parrot ROI
        #parrot = cv2.cvtColor(parrot, cv2.COLOR_GRAY2RGB)
        #twoFrames = np.concatenate((frame,parrot),axis=1)
        #cv2.rectangle(twoFrames,(x,y),(x+wRectangleP,y+hRectangleP),(255,0,0),2)
        # Exit ROI
        exitRoute = cv2.cvtColor(exitRoute, cv2.COLOR_GRAY2RGB)
        twoFrames = np.concatenate((frame,exitRoute),axis=1)
        cv2.rectangle(twoFrames,(x,y),(x+wRectangleE,y+hRectangleE),(255,0,0),2)


        cv2.imshow('ColorvsMask',twoFrames)
        cv2.imshow('cube',exitRoute)
        #print(twoFrames.shape)
        #out.write(twoFrames)
        
        cv2.imshow ('Drone Project', img)
        
        sleep(1/FPS)
        
        k = cv2.waitKey(1)
        if k == 27:         
        # wait for ESC key to exit and terminate feed.
            cv2.destroyAllWindows()
            tello.streamoff()
            tello.land()
            break
    return 0

# Rotate in a Circle
def mission_B():
    if(forward):
        tello.send_rc_control(0,15,0,0)
    else:
        tello.send_rc_control(0,0,0,0)
        tello.rotate_counter_clockwise(90)
    
    if(turn):
        tello.rotate_counter_clockwise(90)
        turn = False

    if(exitState):
        tello.move_forward(40)
        tello.land()

    return 0

def 

if __name__ == '__main__':
    ## State Variables
    forward = True
    turn = False
    exitState = False
    ## Capture video (pc camera)
    #vid = cv2.VideoCapture(0)
    #fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    #out = cv2.VideoWriter('output.avi',fourcc,30, (1280,480))
    ## Json file
    f = open('color.json')
    data = json.load(f)
    white = "white"
    # parrot
    parrot = "parrot"
    lowp = np.array([data[parrot]["low_hue"],data[parrot]["low_sat"],data[parrot]["low_val"]])
    upp = np.array([data[parrot]["up_hue"],data[parrot]["up_sat"],data[parrot]["up_val"]])
    # white
    white = "parrot"
    loww = np.array([data[white]["low_hue"],data[white]["low_sat"],data[white]["low_val"]])
    upw = np.array([data[white]["up_hue"],data[white]["up_sat"],data[white]["up_val"]])
    # exit 
    exitR = "exit"
    lowe = np.array([data[exitR]["low_hue"],data[exitR]["low_sat"],data[exitR]["low_val"]])
    upe = np.array([data[exitR]["up_hue"],data[exitR]["up_sat"],data[exitR]["up_val"]])
    ## Create Trackbars
    cv2.namedWindow('Trackbar')
    cv2.createTrackbar('low_hue','Trackbar',data[white]["low_hue"],180,empty)
    cv2.createTrackbar('up_hue','Trackbar',data[white]["up_hue"],180,empty)
    cv2.createTrackbar('low_sat','Trackbar',data[white]["low_sat"],255,empty)
    cv2.createTrackbar('up_sat','Trackbar',data[white]["up_sat"],255,empty)
    cv2.createTrackbar('low_val','Trackbar',data[white]["low_val"],255,empty)
    cv2.createTrackbar('up_val','Trackbar',data[white]["up_val"],255,empty)
    #Instantiating Drone
    tello = Tello()
    tello.connect()
    tello.query_battery()

    #Iniial sequence 
    tello.takeoff()
    tello.move_down(30)

    #This thread runs receiving video feed.
    receive_video_thread = Thread(target=videoRec)
    receive_video_thread.daemon = True
    receive_video_thread.start()

    #This thread starts the flight pattern.
    flight = mission_B
    mission_thread = Thread(target=flight)
    mission_thread.daemon = True
    mission_thread.start()
    receive_video_thread.join()
    

