using System;
using UnityEngine;

#pragma warning disable 649
namespace UnityStandardAssets.Vehicles.Car
{
    public class CarControllerRC : MonoBehaviour
    {
        [SerializeField] private WheelCollider[] m_WheelColliders = new WheelCollider[4];
        [SerializeField] private GameObject[] m_WheelMeshes = new GameObject[4];
        [SerializeField] private Vector3 m_CentreOfMassOffset;
        [Range(0, 1)] [SerializeField] private float m_SteerHelper; // 0 is raw physics , 1 the car will grip in the direction it is facing
        [Range(0, 1)] [SerializeField] private float m_TractionControl; // 0 is no traction control, 1 is full interference
        [SerializeField] private float m_FullTorqueOverAllWheels;
        [SerializeField] private float m_MaxSpeed; // à régler en fonction de la voiture réele
        [SerializeField] private float m_MaxSteerAngleDeg;
        [SerializeField] private float m_SlipLimit;
        [SerializeField] private float m_BrakeTorque;
        [SerializeField] private float m_engineBrakeTorque;

        private Quaternion[] m_WheelMeshLocalRotations;
        private Vector3 m_Prevpos, m_Pos;
        private float m_SteerAngle;
        private float m_OldRotation;
        private float m_CurrentTorque;
        private Rigidbody m_Rigidbody;
        private float _currentSpeed;

        public bool Skidding { get; private set; }
        public float BrakeInput { get; private set; }
        public float CurrentSteerAngle{ get { return m_SteerAngle; }}
        public float CurrentSpeed{ get { return _currentSpeed; }}
        public float MaxSpeed{get { return m_MaxSpeed; }}
        public float Revs { get; private set; }
        public float AccelInput { get; private set; }

        // Use this for initialization
        private void Start()
        {
            m_WheelMeshLocalRotations = new Quaternion[4];
            for (int i = 0; i < 4; i++)
            {
                m_WheelMeshLocalRotations[i] = m_WheelMeshes[i].transform.localRotation;
            }
            m_WheelColliders[0].attachedRigidbody.centerOfMass = m_CentreOfMassOffset;

            m_Rigidbody = GetComponent<Rigidbody>();
            m_CurrentTorque = m_FullTorqueOverAllWheels - (m_TractionControl*m_FullTorqueOverAllWheels);
        }

        private void FixedUpdate(){
            CalculateSpeed();
        }

        private void CalculateSpeed(){
            
            Vector3 CarVector = m_Rigidbody.transform.forward;
            Vector3 SpeedVector = m_Rigidbody.velocity;

            // If angle between these vectors is 0, then the car is moving in the same direction as the forward vector
            // Else it's 180, and moving reverse.
            float angle = Vector3.Angle(CarVector, SpeedVector);
            angle = Mathf.Round(angle);

            _currentSpeed = m_Rigidbody.velocity.magnitude * 2.23693629f;

            // Take a larger value to allow a deadzone
            // Sometimes values don't perfectly match probably because of rounding errors or floating point percision
            if(angle > 45){
                _currentSpeed *= -1;
            }            
        }


        private void CapSpeed()
        {
            float speed = m_Rigidbody.velocity.magnitude;

            speed *= 3.6f;
            if (speed > m_MaxSpeed)
                m_Rigidbody.velocity = (m_MaxSpeed/3.6f) * m_Rigidbody.velocity.normalized;
        }


        private void SteerHelper()
        {
            for (int i = 0; i < 4; i++) {
                m_WheelColliders[i].GetGroundHit(out WheelHit wheelhit);
                if (wheelhit.normal == Vector3.zero)
                    return; // wheels arent on the ground so dont realign the rigidbody velocity
            }

            // this if is needed to avoid gimbal lock problems that will make the car suddenly shift direction
            if (Mathf.Abs(m_OldRotation - transform.eulerAngles.y) < 10f) {
                var turnadjust = (transform.eulerAngles.y - m_OldRotation) * m_SteerHelper;
                Quaternion velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
                m_Rigidbody.velocity = velRotation * m_Rigidbody.velocity;
            }

            m_OldRotation = transform.eulerAngles.y;
        }


        public void SetSpeed(float speed) {
            // on applique du torque proportionnel a la difference de vitesse actuelle et la vitesse demandée

            float accel = speed * m_MaxSpeed - CurrentSpeed;
            accel = Mathf.Clamp(accel, -1, 1);

            float thrustTorque = accel * (m_CurrentTorque / 4f);
            for (int i = 0; i < 4; i++) {
                m_WheelColliders[i].motorTorque = thrustTorque;
            }

            // Debug.Log($"Speed: {Math.Round(CurrentSpeed*100)}, Consigne: {speed * m_MaxSpeed}, accel=Consigne-Speed: {accel}, Torque: {thrustTorque}");
        }

        public void SetSteering(float steering){
            steering = Mathf.Clamp(steering, -1, 1);

            for (int i = 0; i < 4; i++) {
                m_WheelColliders[i].GetWorldPose(out Vector3 position, out Quaternion quat);
                m_WheelMeshes[i].transform.SetPositionAndRotation(position, quat);
            }

            m_SteerAngle = steering * m_MaxSteerAngleDeg;
            
            m_WheelColliders[0].steerAngle = m_SteerAngle;
            m_WheelColliders[1].steerAngle = m_SteerAngle;

            SteerHelper();
        }
    }
}
