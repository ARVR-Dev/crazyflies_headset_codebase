
#With dependencies below installed (and Uvicorn as well), run server with this command: python -m uvicorn ViconDroneServer:app --host 0.0.0.0 --port 8000 --reload
from fastapi import FastAPI, File, Form, UploadFile,Request,Body
import numpy as np
import json
from pydantic import BaseModel
from threading import Lock
from typing import List,Tuple, Annotated
from Bot import Bot
import sys


class Headset:
    def __init__(self):
        self.unityPositions = []
        self.unityOrientations = []
        self.viconPositions = []
        self.viconOrientations = []
        self.viconToUnityTransform = "None"
        self.currentUnityPosition = "None"
        self.currentUnityOrientation = "None"
        
    
    def computeViconToUnityTransform(self):
        self.viconToUnityTransform = self.computeTransformationMatrix(np.array(self.unityPositions),np.array(self.viconPositions))
    
    def transformViconPoint(self,point):
        homogeneous_point = np.array([point[0], point[1], point[2], 1.0])

        # Apply the transformation
        transformed_homogeneous = self.viconToUnityTransform @ homogeneous_point

        # Return the transformed x, y, z (ignore the homogeneous component)
        return transformed_homogeneous[:3]
        
    
    def computeTransformationMatrix(self,unityPositions,viconPositions):
        """
        Compute the rigid transformation matrix (rotation + translation)
        that aligns P1 to P2 using the Umeyama algorithm.
        
        :param P1: (N, 3) numpy array of points in the first coordinate system.
        :param P2: (N, 3) numpy array of points in the second coordinate system.
        :return: 4x4 transformation matrix.
        """
        if(len(viconPositions) > len(unityPositions)):
            sizeDiff = len(viconPositions) - len(unityPositions)
            viconPositions =  viconPositions[:-sizeDiff]
        elif(len(unityPositions) > len(viconPositions)):
            sizeDiff = len(unityPositions) - len(viconPositions)
            unityPositions =  unityPositions[:-sizeDiff]

        print("Vicon")
        print(viconPositions.shape)
        print("Unity")
        print(unityPositions.shape)
        

        # Compute centroids
        centroid_P1 = np.mean(viconPositions, axis=0)
        centroid_P2 = np.mean(unityPositions, axis=0)

        # Center the points
        P1_centered = viconPositions - centroid_P1
        P2_centered = unityPositions - centroid_P2

        # Compute covariance matrix
        H = P1_centered.T @ P2_centered

        # Compute SVD
        U, S, Vt = np.linalg.svd(H)

        # Compute rotation matrix
        R = Vt.T @ U.T

        # ✅ Step 2: Fix reflection if detected
        #if np.linalg.det(R) < 0:  
        #    print("⚠️ Reflection detected! Correcting rotation matrix...")
        #    Vt[-1, :] *= -1  # Flip the last singular vector
        #    R = Vt.T @ U.T  

        # Compute translation vector
        T = centroid_P2 - R @ centroid_P1

        # ✅ Step 3: Construct the transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = T


        return transformation_matrix


#Define JSON struct for receiving drone current state from 
#drone with a namespace
class DroneState(BaseModel):
    namespace: str
    position: List[float]
    orientation: List[float]
    trajectory: List[List[float]]
    batteryPercentage: float

class HeadsetPose(BaseModel):
    position: List[float]
    orientation: List[float]


class FastAPP(FastAPI):
    lock = Lock()
    def __init__(self):
        super().__init__()
        self.operatingDrones = {}
        self.viconToHeadsetTransform = -1
        self.headset = Headset()
        self.operatingDrones["/test"] = Bot("/test")
        self.operatingDrones["/test"].currentPosition = [0.012002854846663545,0.021227951688600817,0.033367674107008485]
        self.operatingDrones["/test"].currentOrientation = [0.0,0.0,0.0,1.0]
        self.operatingDrones["/test"].currentTrajectory = []
        self.operatingDrones["/test"].currentBatteryPercentage = 0.0
        self.NUM_LOCALIZATION_SAMPLES = 1000

#Create server application
app = FastAPP()

#Route for receiving pose POST requests from drones. Upon request,
#the drone's current coords are stored within its representation in the 
#operatingDrones dictionary if that drone has a representation. If not,
#one is created with that namespace and added to the dict
@app.post("/senddronecurrentstate")
async def sendDroneCurrentCoords(droneState: DroneState):
    with app.lock:
        if(droneState.namespace not in app.operatingDrones.keys()):
            app.operatingDrones[droneState.namespace] = Bot(droneState.namespace)
        app.operatingDrones[droneState.namespace].currentPosition = droneState.position
        app.operatingDrones[droneState.namespace].currentOrientation = droneState.orientation
        app.operatingDrones[droneState.namespace].currentTrajectory = droneState.trajectory
        app.operatingDrones[droneState.namespace].currentBatteryPercentage = droneState.batteryPercentage

@app.post("/sendheadsetcurrentposeunity")
async def sendHeadsetCurrentPoseUnity(headsetPoseUnity: HeadsetPose):
    with app.lock:
        #app.headset.unityPositions.append(headsetPoseUnity.position)
        #app.headset.unityOrientations.append(headsetPoseUnity.orientation)
        app.headset.currentUnityPosition = headsetPoseUnity.position
        app.headset.currentUnityOrientation = headsetPoseUnity.orientation
        #if(app.headset.viconToUnityTransform == "None" and len(app.headset.unityPositions) >= 500 and len(app.headset.viconPositions) >= 500):
        #    app.headset.computeViconToUnityTransform()


@app.post("/sendheadsetcurrentposevicon")
async def sendHeadsetCurrentPoseVicon(headsetPoseVicon: HeadsetPose):
    with app.lock:
        if(app.headset.currentUnityPosition != "None"):
            app.headset.viconPositions.append(headsetPoseVicon.position)
            app.headset.viconOrientations.append(headsetPoseVicon.orientation)
            app.headset.unityPositions.append(app.headset.currentUnityPosition)
            app.headset.unityOrientations.append(app.headset.currentUnityOrientation)
            if(app.headset.viconToUnityTransform == "None" and len(app.headset.unityPositions) >= app.NUM_LOCALIZATION_SAMPLES and len(app.headset.viconPositions) >= app.NUM_LOCALIZATION_SAMPLES):
                app.headset.computeViconToUnityTransform()
        else:
            pass


#Route used by headset to fetch most recent stored coords
#of drone with specified namespace, relative to Vicon frame
#Meaning, coords returned to headset from this route will need
#transformed into coordinate frame of headset
@app.get("/getdronecurrentcoords")
async def getDroneCurrentCoords(droneNamespace: str = "/default"):
    with app.lock:
        print(app.headset.viconToUnityTransform)
        print(droneNamespace)
        print("Unity")
        print(len(app.headset.unityPositions))
        print("Vicon")
        print(len(app.headset.viconPositions))
        #Make sure drone is active and there is a valid vicon to unity transformation
        if(app.headset.viconToUnityTransform != "None" and droneNamespace in app.operatingDrones.keys()):
            print("Original point")
            print(app.operatingDrones[droneNamespace].currentPosition)
            transformedPoint = app.headset.transformViconPoint(app.operatingDrones[droneNamespace].currentPosition)
            print("Transformed point")
            print(transformedPoint)
            x = transformedPoint[0]
            y = transformedPoint[1]
            z = transformedPoint[2]
            return {"X":x,"Y":y,"Z":z}
        else:
            return {"X":0.0,"Y":0.0,"Z":0.0}

@app.get("/getdronecurrentbatterypercentage")
async def getDroneCurrentBatteryPercentage(droneNamespace: str = "/default"):
    with app.lock:
        if(droneNamespace in app.operatingDrones.keys()):
            return {"batteryPercentage": app.operatingDrones[droneNamespace].currentBatteryPercentage}
        else:
            return {"batteryPercentage": -1.0}

@app.get("/getdronecurrenttrajectory")
async def getDroneCurrentTrajectory(droneNamespace: str = "/default"):
    with app.lock:
        if(droneNamespace in app.operatingDrones.keys() and app.headset.viconToUnityTransform != "None"):
            if(len(app.operatingDrones[droneNamespace].currentTrajectory)>0):
                return {"trajectoryPoints": [app.headset.transformViconPoint(point).tolist() for point in app.operatingDrones[droneNamespace].currentTrajectory]}
            else:
                return {"trajectoryPoints": [[-1000.0,-1000.0,-1000.0]]}
        else:
            return {"trajectoryPoints": [[-1000.0,-1000.0,-1000.0]]}





