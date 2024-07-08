using System.ComponentModel.Design.Serialization;
using System.ComponentModel;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace UnityStandardAssets.Vehicles.Car{
    public class CarTelemetryRC : MonoBehaviour{
        public CarControllerRC carControllerRC;

        [SerializeField] private float steeringAngle;
        [SerializeField] private float throttle;
        [SerializeField] private float brake;
        [SerializeField] private float speed;

        // Update is called once per frame
        void Update()
        {
            steeringAngle = carControllerRC.CurrentSteerAngle;
            throttle = carControllerRC.AccelInput;
            brake = carControllerRC.BrakeInput;
            speed = carControllerRC.CurrentSpeed;
        }
    }

}

