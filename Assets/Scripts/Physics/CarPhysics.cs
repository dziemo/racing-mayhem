using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SocialPlatforms;

namespace RacingMayhem
{
    public enum AccelerationType { Forward, Rear, Full }

    public class CarPhysics : MonoBehaviour
    {
        [SerializeField]
        private Rigidbody rb;
        [SerializeField]
        private Transform centerOfMass;
        [SerializeField]
        private LayerMask collisionMask;
        [Header("Acceleration")]
        [SerializeField]
        private float carTopSpeed;
        [SerializeField]
        private AnimationCurve powerCurve;
        [SerializeField]
        private float brakeForce;
        [SerializeField]
        [Range(0.01f, 0.99f)]
        private float rollingFrictionFactor;
        [SerializeField]
        private float brakeTreshold;
        [SerializeField]
        private float reverseSpeed;
        [SerializeField]
        private AccelerationType accelerationType;
        [Header("Steering")]
        [SerializeField]
        [Range(0f, 1f)]
        private float tireGripFactor;
        [SerializeField]
        private float tireMass;
        [SerializeField]
        private float maxTireRotation;
        [SerializeField]
        private float tireRotationEase;

        [Header("Suspension")]
        [SerializeField]
        private List<Transform> raycastTransforms = new List<Transform>();
        [SerializeField]
        private List<Transform> tireVisuals = new List<Transform>();
        [SerializeField]
        private float raycastLength;
        [SerializeField]
        private float suspensionRestDist;
        [SerializeField]
        private float springStrength;
        [SerializeField]
        private float springDamper;
        [SerializeField]
        private float suspensionSmoothTime;

        private ICarInput carInput;
        private Vector3 suspensionSmoothingVelocity;

        private int wheelsNotTouching = 0;

        private void Awake()
        {
            rb.centerOfMass = centerOfMass.position;

            if (TryGetComponent(out ICarInput carInput))
            {
                this.carInput = carInput;
            }
        }

        private void FixedUpdate()
        {
            ApplyTireForces();
        }

        private void Brake(float brakeForce)
        {
            for (int i = 0; i < raycastTransforms.Count; i++)
            {
                Transform tireTransform = raycastTransforms[i];
                Vector3 movingDir = tireTransform.forward;
                Vector3 tireWorldVel = rb.GetPointVelocity(tireTransform.position);
                float movingVel = Vector3.Dot(movingDir, tireWorldVel);
                float desiredVelChange = -movingVel * brakeForce;
                float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
                Vector3 force = desiredAccel * tireMass * movingDir;
                rb.AddForceAtPosition(force, tireTransform.position);

                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
        }

        private void ApplyTireForces()
        {
            int wheelsNotTouchingThisFrame = 0;
            for (int i = 0; i < raycastTransforms.Count; i++)
            {
                Transform tireTransform = raycastTransforms[i];

                if (i < 2)
                {
                    tireTransform.localRotation = Quaternion.Lerp(tireTransform.localRotation,
                        Quaternion.Euler(new Vector3(0, maxTireRotation * carInput.HorizontalInput)),
                        tireRotationEase * (1 / (1 + rb.velocity.magnitude * Mathf.Abs(carInput.HorizontalInput)) * Time.fixedDeltaTime));
                }

                if (Physics.Raycast(tireTransform.position, -tireTransform.up, out RaycastHit hitInfo, raycastLength, collisionMask))
                {
                    SuspensionForce(tireTransform, hitInfo, tireVisuals[i]);

                    if (accelerationType == AccelerationType.Forward && i < 2)
                    {
                        ForwardForce(tireTransform, hitInfo);
                    }
                    else if (accelerationType == AccelerationType.Rear && i >= 2)
                    {
                        ForwardForce(tireTransform, hitInfo);
                    }
                    else
                    {
                        ForwardForce(tireTransform, hitInfo);
                    }

                    SteeringForce(tireTransform);
                }
                else
                {
                    wheelsNotTouchingThisFrame++;
                    FullyExtendSuspension(tireVisuals[i]);
                }
            }
            wheelsNotTouching = wheelsNotTouchingThisFrame;
        }

        private void SteeringForce(Transform tireTransform)
        {
            Vector3 steeringDir = tireTransform.right;
            Vector3 tireWorldVel = rb.GetPointVelocity(tireTransform.position);
            float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
            float desiredVelChange = -steeringVel * tireGripFactor;
            float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
            Vector3 force = desiredAccel * tireMass * steeringDir;
            rb.AddForceAtPosition(force, tireTransform.position);
            Debug.DrawRay(tireTransform.position, force.normalized, Color.red);
        }

        private void SuspensionForce(Transform tireTransform, RaycastHit hitInfo, Transform tireVisual)
        {
            Vector3 springDir = Vector3.up;
            Vector3 tireWorldVel = rb.GetPointVelocity(tireTransform.position);
            float offset = suspensionRestDist - hitInfo.distance;
            float velocity = Vector3.Dot(springDir, tireWorldVel);
            float springForce = (offset * springStrength) - (velocity * springDamper);
            Vector3 forceApplied = springDir * springForce;
            rb.AddForceAtPosition(forceApplied, tireTransform.position);
            tireVisual.localPosition = Vector3.SmoothDamp(tireVisual.localPosition,
                new Vector3(tireVisual.localPosition.x, -hitInfo.distance + 0.3f, 0),
                ref suspensionSmoothingVelocity, suspensionSmoothTime);

            Debug.DrawRay(tireTransform.position, forceApplied.normalized, Color.green);
        }

        private void FullyExtendSuspension(Transform tireVisual)
        {
            tireVisual.localPosition = Vector3.SmoothDamp(tireVisual.localPosition,
                new Vector3(tireVisual.localPosition.x, -raycastLength + 0.3f, 0),
                ref suspensionSmoothingVelocity, suspensionSmoothTime);
        }

        private void ForwardForce(Transform tireTransform, RaycastHit hitInfo)
        {
            Vector3 accelDir = Vector3.ProjectOnPlane(tireTransform.forward, hitInfo.normal);

            int notTouchingWheelsFactor = 1;
            if (raycastTransforms.Count - wheelsNotTouching > 0)
            {
                notTouchingWheelsFactor += wheelsNotTouching / (raycastTransforms.Count - wheelsNotTouching);
            }

            if (carInput.VerticalInput > 0)
            {
                float carSpeed = Vector3.Dot(transform.forward, rb.velocity);
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * carInput.VerticalInput * carTopSpeed * notTouchingWheelsFactor;
                Vector3 force = accelDir * availableTorque;
                rb.AddForceAtPosition(force, tireTransform.position);
                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
            else if (carInput.VerticalInput < 0)
            {
                float carSpeed = Vector3.Dot(-transform.forward, rb.velocity);
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / reverseSpeed);
                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * carInput.VerticalInput * reverseSpeed * notTouchingWheelsFactor;
                Vector3 force = accelDir * availableTorque;
                rb.AddForceAtPosition(force, tireTransform.position);
                Debug.DrawRay(tireTransform.position, force.normalized, Color.blue);
            }
            else
            {
                Brake(brakeForce * rollingFrictionFactor);
            }
        }
    }
}