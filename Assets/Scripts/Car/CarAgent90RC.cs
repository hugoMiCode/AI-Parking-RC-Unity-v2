using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.Barracuda;
using System;
using UnityEditor;


#pragma warning disable IDE0130 // Namespace does not match folder structure
namespace UnityStandardAssets.Vehicles.Car
#pragma warning restore IDE0130 // Namespace does not match folder structure
{
    public class CarAgent90RC : Agent
    {
        // Public variables
        public Vector2 spawnRangeX = new Vector2(1, 2.3f);
        public Vector2 spawnRangeZ = new Vector2(-9f, 2f);
        public GameObject targetFinal;

        // Private variables
        private int RayAmount;
        private float[] RayDistancesLeftTarget;
        private float[] RayDistancesRightTarget;
        private float RayDistanceFrontTarget;
        private float RayDistanceBackTarget;
        private float[] RayDistancesLeftPrevious;
        private float[] RayDistancesRightPrevious;
        private float RayDistanceFrontPrevious;
        private float RayDistanceBackPrevious;
        private RayPerceptionOutput.RayOutput[] RawRayOutputs;
        private float PreviousDistance = 0f;
        private float[] LidarM1;
        private float[] LidarM2;

        // Components
        private RayPerceptionSensorComponent3D RayPerceptionSensorComponent3D;
        private Rigidbody rb;
        private CarControllerRC carControllerRC;
        private StatsRecorder statsRecorder;
        private Vector3 startPosition;

        // Queues for benchmarking the agent
        private readonly Queue<bool> successQueue = new Queue<bool>();
        private readonly Queue<float> distanceToTargetLidarQueue = new Queue<float>();
        private const int queueSize = 100;




        public override void Initialize()
        {
            GameObject Sensor = transform.Find("Sensor").gameObject;
            RayPerceptionSensorComponent3D = Sensor.GetComponent<RayPerceptionSensorComponent3D>();
            carControllerRC = GetComponent<CarControllerRC>();
            rb = GetComponent<Rigidbody>();
            statsRecorder = Academy.Instance.StatsRecorder;
            startPosition = transform.localPosition;

            // Initialize the RayDistances arrays
            RayPerceptionInput RayPerceptionIn = RayPerceptionSensorComponent3D.GetRayPerceptionInput();
            RayPerceptionOutput RayPerceptionOut = RayPerceptionSensor.Perceive(RayPerceptionIn);
            RayPerceptionOutput.RayOutput[] RayOutputs = RayPerceptionOut.RayOutputs;
    
            // 'Length -1' because 2 Rays overlap in the end of the array.
            RayAmount = RayOutputs.Length - 1;
            RayDistancesLeftTarget = new float[(RayAmount - 2) / 2];
            RayDistancesRightTarget = new float[(RayAmount - 2) / 2];
            RayDistancesLeftPrevious = new float[(RayAmount - 2) / 2];
            RayDistancesRightPrevious = new float[(RayAmount - 2) / 2];

            LidarM1 = new float[RayAmount];
            LidarM2 = new float[RayAmount];

            Reset();
        }

        public override void OnEpisodeBegin()
        {
            Reset();
        }

        private void Reset()
        {
            // Reset the car's velocity
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            // Put the car in the slot space and read the lidar
            Vector3 ReadLidarPosition = new(targetFinal.transform.localPosition.x, startPosition.y, targetFinal.transform.localPosition.z);
            rb.transform.SetLocalPositionAndRotation(ReadLidarPosition, Quaternion.Euler(0, 0, 0));
            (RayDistancesLeftTarget, RayDistancesRightTarget, RayDistanceFrontTarget, RayDistanceBackTarget) = ReadRayCast();
        
            // Randomize car's spawn position and rotation
            float spawnX = UnityEngine.Random.Range(spawnRangeX.x, spawnRangeX.y);
            float spawnZ = UnityEngine.Random.Range(spawnRangeZ.x, spawnRangeZ.y);
            Vector3 spawnPosition = new(spawnX, startPosition.y, spawnZ);

            float baseAngle = UnityEngine.Random.Range(-80f, 80f);
            if (spawnX < -6)
                baseAngle = 90f + UnityEngine.Random.Range(-30f, 30f);
            else if (spawnX > 6)
                baseAngle = -90f + UnityEngine.Random.Range(-30f, 30f);

            // Set the car's position and rotation
            Quaternion spawnRotation = Quaternion.Euler(0, baseAngle, 0);
            rb.transform.SetLocalPositionAndRotation(spawnPosition, spawnRotation);
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Collect observations from the environment
            float[] RayDistancesLeftCurrent;
            float[] RayDistancesRightCurrent;
            float RayDistanceFrontCurrent;
            float RayDistanceBackCurrent;
            (RayDistancesLeftCurrent, RayDistancesRightCurrent, RayDistanceFrontCurrent, RayDistanceBackCurrent) = ReadRayCast();

            // Create the lidar array
            int totalRayCount = RayDistancesLeftCurrent.Length + RayDistancesRightCurrent.Length + 2;
            float[] CurrentLidar = new float[totalRayCount];
            int index = 0;

            CurrentLidar[index++] = RayDistanceFrontCurrent;
            for (int i = 0; i < RayDistancesRightCurrent.Length; i++)
                CurrentLidar[index++] = RayDistancesRightCurrent[i];

            CurrentLidar[index++] = RayDistanceBackCurrent;
            for (int i = 0; i < RayDistancesLeftCurrent.Length; i++)
                CurrentLidar[index++] = RayDistancesLeftCurrent[i];


            // Add the observations to the sensor
            for (int i = 0; i < CurrentLidar.Length; i++) {
                sensor.AddObservation(CurrentLidar[i]);
                sensor.AddObservation(LidarM1[i]);
                sensor.AddObservation(LidarM2[i]);
            }

            sensor.AddObservation(carControllerRC.CurrentSpeed / carControllerRC.MaxSpeed);


            LidarM2 = LidarM1;
            LidarM1 = CurrentLidar;
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            float steering = actions.ContinuousActions[0];
            float speed = actions.ContinuousActions[1];

            carControllerRC.SetSpeed(speed);
            carControllerRC.SetSteering(steering);

            float reward = CalculateReward();
            AddReward(reward);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

            float steering = Input.GetAxis("Horizontal");
            float speed = Input.GetAxis("Vertical");

            continuousActionsOut[0] = steering;
            continuousActionsOut[1] = speed;
        }

        void FixedUpdate()
        {
            RequestDecision();
        }

        private float CalculateReward()
        {
            float reward = 0f;

            // Negative reward for each step
            reward -= 1000f / MaxStep;

            float distance = CalculateDistanceDotProduct();

            if (distance < 5f && Mathf.Abs(carControllerRC.CurrentSpeed) < 0.15) {
                reward += 10000f;
                Debug.Log("Success");
                EndEpisodeWithSuccess(true, distance);
            }

            reward += (PreviousDistance - distance) * 100;
            PreviousDistance = distance;

            // if (Mathf.Abs(reward) > 3f)
            //     Debug.Log("Reward: " + reward + " Distance: " + distance);

            return reward;
        }

        private void EndEpisodeWithSuccess(bool wasSuccessful, float distanceToTargetLidar = -1f)
        {
            if (successQueue.Count >= queueSize) {
                successQueue.Dequeue();
                distanceToTargetLidarQueue.Dequeue();
            }

            successQueue.Enqueue(wasSuccessful);
            distanceToTargetLidarQueue.Enqueue(distanceToTargetLidar);

            float successRate = CalculateSuccessRate();
            float averageDistance = CalculateAverageDistance();

            statsRecorder.Add("ParkingSuccessRate", successRate);
            if (distanceToTargetLidar != -1f)
                statsRecorder.Add("ParkingMultiplier", averageDistance);

            EndEpisode();
        }

        private float CalculateSuccessRate()
        {
            int successCount = 0;
            foreach (bool success in successQueue) {
                if (success)
                    successCount++;
            }

            return (float)successCount / successQueue.Count * 100;
        }

        private float CalculateAverageDistance()
        {
            float totalMultiplier = 0f;
            foreach (float multiplier in distanceToTargetLidarQueue) {
                totalMultiplier += multiplier;
            }

            return totalMultiplier / distanceToTargetLidarQueue.Count;
        }

        private (float[] RDistL, float[] RDistR, float RDistF, float RDistB) ReadRayCast()
        {
            RayPerceptionInput RayPerceptionIn = RayPerceptionSensorComponent3D.GetRayPerceptionInput();
            RayPerceptionOutput RayPerceptionOut = RayPerceptionSensor.Perceive(RayPerceptionIn);
            RayPerceptionOutput.RayOutput[] RayOutputs = RayPerceptionOut.RayOutputs;

            // Indexing is quite retarded
            // Index ordering: [0] = front, [1] = right, [2] = left, [3] = right, [4] = left,...
            // Even numbers are left, odd numbers are right
            float[] RayDistancesLeft  = new float[(RayAmount - 2) / 2];
            float[] RayDistancesRight = new float[(RayAmount - 2) / 2];
            float RayDistanceFront = RayOutputs[0].HitFraction;
            float RayDistanceBack = RayOutputs[RayAmount - 1].HitFraction;

            for (int i = 1; i < RayAmount - 1; i++)
                if (i % 2 == 0)
                    RayDistancesLeft[(i - 2) / 2] = RayOutputs[i].HitFraction;
                else
                    RayDistancesRight[(i - 1) / 2] = RayOutputs[i].HitFraction;

            return (RayDistancesLeft, RayDistancesRight, RayDistanceFront, RayDistanceBack);
        }

        float CalculateDistanceDotProduct()
        {
            float[] RayDistancesLeftCurrent;
            float[] RayDistancesRightCurrent;
            float RayDistanceFrontCurrent;
            float RayDistanceBackCurrent;
            (RayDistancesLeftCurrent, RayDistancesRightCurrent, RayDistanceFrontCurrent, RayDistanceBackCurrent) = ReadRayCast();
            
            // calculate the dot product between the (target - previous) and (target - current) lidar
            float distFrontCurrent = RayDistanceFrontTarget - RayDistanceFrontCurrent;
            float distBackCurrent = RayDistanceBackTarget - RayDistanceBackCurrent;
            float[] distLeftCurrent = new float[RayDistancesLeftTarget.Length];
            float[] distRightCurrent = new float[RayDistancesRightTarget.Length];
            for (int i = 0; i < RayDistancesLeftTarget.Length; i++) {
                distLeftCurrent[i] = RayDistancesLeftTarget[i] - RayDistancesLeftCurrent[i];
                distRightCurrent[i] = RayDistancesRightTarget[i] - RayDistancesRightCurrent[i];
            }

            float distFrontPrevious = RayDistanceFrontTarget - RayDistanceFrontPrevious;
            float distBackPrevious = RayDistanceBackTarget - RayDistanceBackPrevious;
            float[] distLeftPrevious = new float[RayDistancesLeftTarget.Length];
            float[] distRightPrevious = new float[RayDistancesRightTarget.Length];
            for (int i = 0; i < RayDistancesLeftTarget.Length; i++) {
                distLeftPrevious[i] = RayDistancesLeftTarget[i] - RayDistancesLeftPrevious[i];
                distRightPrevious[i] = RayDistancesRightTarget[i] - RayDistancesRightPrevious[i];
            }

            float dotProductLeft = DotProduct(distLeftCurrent, distLeftPrevious);
            float dotProductRight = DotProduct(distRightCurrent, distRightPrevious);
            float dotProductFront = distFrontCurrent * distFrontPrevious;
            float dotProductBack = distBackCurrent * distBackPrevious;

            float distance = dotProductLeft + dotProductRight + dotProductFront + dotProductBack;

            RayDistancesLeftPrevious = RayDistancesLeftCurrent;
            RayDistancesRightPrevious = RayDistancesRightCurrent;
            RayDistanceFrontPrevious = RayDistanceFrontCurrent;
            RayDistanceBackPrevious = RayDistanceBackCurrent;

            return distance;
        }

        float DotProduct(float[] a, float[] b)
        {
            float dotProduct = 0f;

            for (int i = 0; i < a.Length; i++)
                dotProduct += a[i] * b[i];

            return dotProduct;
        }

        void OnTriggerEnter(Collider other)
        {
            if (other.gameObject.CompareTag("envWall")) {
                AddReward(-1000f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Out of bounds");
            }
        }

        void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.CompareTag("Car")) {
                AddReward(-200f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Collision with car");
            }
        }

        private string ArrayToString(float[] array) {
            string str = "[";

            for (int i = 0; i < array.Length; i++) {
                str += array[i].ToString(System.Globalization.CultureInfo.InvariantCulture);
                if (i != array.Length - 1)
                    str += ", ";
            }

            str += "]";

            return str;
        }
    }
}


